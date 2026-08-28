from __future__ import annotations

from collections.abc import Sequence
import threading
import uuid

import msgpack
import zmq

from zero_franky.protocol import encode_affine, format_exception, normalize_state_fields


RPC_HANDLERS = {}

#: Cap on how long server shutdown waits for each tracker's stop ramp [s].
SHUTDOWN_JOIN_TIMEOUT = 5.0

#: How many stopped sessions keep a readable final status. Bounded because a
#: long-lived server starts unboundedly many trackers.
FINISHED_STATUS_HISTORY = 32


def rpc_handler(method: str):
    def register(fn):
        RPC_HANDLERS[method] = fn
        return fn

    return register


class RobotManager:
    def __init__(self, state_publisher=None, state_fields: Sequence[str] | str | None = None):
        import franky

        self._franky = franky
        self._state_fields = normalize_state_fields(state_fields)
        self._state_publisher = state_publisher
        self._robots: dict[str, Any] = {}
        self._latest_state: dict[str, dict[str, Any]] = {}
        self._tracker_sessions: dict[str, Any] = {}
        self._finished_statuses: dict[str, dict[str, Any]] = {}

    def create_robot(self, fci_hostname: str, kwargs: dict[str, Any] | None = None) -> str:
        robot_id = uuid.uuid4().hex
        self._robots[robot_id] = self._franky.Robot(fci_hostname, **(kwargs or {}))
        return robot_id

    def kinematics_info(self, robot_id: str):
        """Return the fixed EE transform and an initial joint seed for local IK."""
        state = self._robot(robot_id).state()
        return {
            "f_t_ee": encode_affine(state.F_T_EE),
            "q": [float(value) for value in state.q],
        }

    def _robot(self, robot_id: str):
        try:
            return self._robots[robot_id]
        except KeyError as exc:
            raise KeyError(f"Unknown robot id: {robot_id}") from exc

    def recover_from_errors(self, robot_id: str):
        return self._robot(robot_id).recover_from_errors()

    def join_motion(self, robot_id: str, timeout: float | None):
        try:
            return self._robot(robot_id).join_motion(timeout)
        except self._franky.ControlException as e:
            # stop() preempts the active motion; libfranka stores the exception and
            # re-raises it on the next join_motion call. Treat preempt as a clean stop.
            if "preempted" in str(e).lower():
                return False
            raise

    def poll_motion(self, robot_id: str):
        return self._robot(robot_id).poll_motion()

    def stop(self, robot_id: str):
        return self._robot(robot_id).stop()

    def start_joint_tracker(
        self,
        robot_id: str,
        policy_payload: dict[str, Any] | None = None,
        motion_kwargs: dict[str, Any] | None = None,
        period: float | None = 0.001,
        stop_on_policy_error: bool = True,
    ) -> str:
        from zero_franky.tracker_session import TrackerSession, load_policy
        from zero_franky.zmq_server_franky import _franky_joint_reference, franky_motion_kwargs

        robot = self._robot(robot_id)
        motion_options = franky_motion_kwargs(self._franky, motion_kwargs)
        motion = self._franky.JointImpedanceTrackingMotion(**motion_options)
        motion.set_reference(
            _franky_joint_reference(self._franky, robot.current_joint_positions, robot.current_joint_velocities)
        )
        self._register_state_callback(robot_id, motion)
        robot.move(motion, asynchronous=True)
        session = TrackerSession(
            franky=self._franky,
            robot=robot,
            kind="joint",
            policy_factory=load_policy(policy_payload) if policy_payload is not None else None,
            motion=motion,
            period=period,
            stop_on_policy_error=stop_on_policy_error,
        )
        self._tracker_sessions[session.id] = session
        return session.start()

    def start_cartesian_tracker(
        self,
        robot_id: str,
        policy_payload: dict[str, Any] | None = None,
        motion_kwargs: dict[str, Any] | None = None,
        period: float | None = 0.001,
        stop_on_policy_error: bool = True,
    ) -> str:
        from zero_franky.tracker_session import TrackerSession, load_policy
        from zero_franky.zmq_server_franky import _franky_cartesian_reference, franky_motion_kwargs

        robot = self._robot(robot_id)
        motion_options = franky_motion_kwargs(self._franky, motion_kwargs)
        motion = self._franky.CartesianImpedanceTrackingMotion(**motion_options)
        motion.set_reference(_franky_cartesian_reference(self._franky, robot.current_pose.end_effector_pose))
        self._register_state_callback(robot_id, motion)
        robot.move(motion, asynchronous=True)
        session = TrackerSession(
            franky=self._franky,
            robot=robot,
            kind="cartesian",
            policy_factory=load_policy(policy_payload) if policy_payload is not None else None,
            motion=motion,
            period=period,
            stop_on_policy_error=stop_on_policy_error,
        )
        self._tracker_sessions[session.id] = session
        return session.start()

    def start_torque_tracker(self, robot_id: str, motion_kwargs: dict[str, Any] | None = None) -> str:
        from zero_franky.tracker_session import TrackerSession
        from zero_franky.zmq_server_franky import franky_motion_kwargs

        robot = self._robot(robot_id)
        motion = self._franky.SimpleTorqueMotion(**franky_motion_kwargs(self._franky, motion_kwargs))
        self._register_state_callback(robot_id, motion)
        robot.move(motion, asynchronous=True)
        session = TrackerSession(
            franky=self._franky,
            robot=robot,
            kind="torque",
            policy_factory=None,
            motion=motion,
        )
        self._tracker_sessions[session.id] = session
        return session.start()

    def tracker_status(self, session_id: str):
        session = self._tracker_sessions.get(session_id)
        if session is not None:
            return session.status()
        try:
            return self._finished_statuses[session_id]
        except KeyError as exc:
            raise KeyError(f"Unknown tracker session id: {session_id}") from exc

    def stop_tracker(
        self,
        session_id: str,
        join_timeout: float | None = None,
        stop_motion_payload: dict[str, Any] | None = None,
    ):
        """Stop a tracker session, tolerating a session that is already gone.

        Deregistered before stopping so that repeated stops are harmless, the way
        franky's tracker `stop()` is: the context manager calls it on exit even if
        the caller already stopped explicitly. A stop that raises therefore leaves
        no session behind to retry, which is deliberate — the exception reaches the
        client instead of being stranded on a half-stopped session.
        """
        session = self._tracker_sessions.pop(session_id, None)
        if session is None:
            return True
        stop_motion = None
        if stop_motion_payload is not None:
            from zero_franky.zmq_server_franky import build_franky_motion

            stop_motion = build_franky_motion(self._franky, stop_motion_payload)
        try:
            session.stop(join_timeout, stop_motion)
        finally:
            self._remember_finished(session)
        return True

    def _remember_finished(self, session) -> None:
        """Keep a stopped session's final status readable after deregistration.

        So `is_running` answers False after a stop, as franky's does, rather than
        raising `Unknown tracker session id`.
        """
        self._finished_statuses[session.id] = session.status()
        while len(self._finished_statuses) > FINISHED_STATUS_HISTORY:
            self._finished_statuses.pop(next(iter(self._finished_statuses)))

    def set_joint_tracker_reference(
        self,
        session_id: str,
        position: list[float],
        velocity: list[float] | None = None,
        torque_feedforward: list[float] | None = None,
    ):
        self._tracker_session(session_id).set_joint_reference(position, velocity, torque_feedforward)
        return True

    def set_cartesian_tracker_reference(
        self,
        session_id: str,
        target_payload: dict[str, Any],
        target_twist_payload: dict[str, Any] | None = None,
        target_acceleration_payload: dict[str, Any] | None = None,
    ):
        from zero_franky.zmq_server_franky import (
            _franky_affine,
            _franky_robot_velocity,
            _franky_twist_acceleration,
        )

        target = _franky_affine(self._franky, target_payload)
        target_twist = _franky_robot_velocity(self._franky, target_twist_payload) if target_twist_payload else None
        target_acceleration = _franky_twist_acceleration(self._franky, target_acceleration_payload)
        self._tracker_session(session_id).set_cartesian_reference(target, target_twist, target_acceleration)
        return True

    def set_joint_tracker_gains(self, session_id: str, stiffness: list[float], damping: list[float] | None):
        self._tracker_session(session_id).set_joint_gains(stiffness, damping)
        return True

    def set_cartesian_tracker_gains(self, session_id: str, gains_payload: dict[str, Any]):
        # Forwarded unresolved: the session merges it into the motion's current gains.
        self._tracker_session(session_id).set_cartesian_gains(gains_payload)
        return True

    def set_joint_tracker_cartesian_gains(self, session_id: str, gains_payload: dict[str, Any]):
        self._tracker_session(session_id).set_hybrid_cartesian_gains(gains_payload)
        return True

    def set_tracker_posture_stiffness(self, session_id: str, posture_stiffness: Any):
        self._tracker_session(session_id).set_posture_stiffness(posture_stiffness)
        return True

    def set_cartesian_tracker_nullspace_gains(self, session_id: str, gains_payload: dict[str, Any]):
        from zero_franky.zmq_server_franky import _franky_nullspace_gains

        gains = _franky_nullspace_gains(self._franky, gains_payload)
        self._tracker_session(session_id).set_nullspace_gains(gains)
        return True

    def set_session_torque(self, session_id: str, torque: list[float]):
        self._tracker_session(session_id).set_torque(torque)
        return True

    def get_session_torque(self, session_id: str):
        from zero_franky.protocol import encode_rpc_value

        return encode_rpc_value(self._tracker_session(session_id).get_torque())

    def _tracker_session(self, session_id: str):
        try:
            return self._tracker_sessions[session_id]
        except KeyError as exc:
            raise KeyError(f"Unknown tracker session id: {session_id}") from exc

    def shutdown(self) -> None:
        # Bounded, unlike an ordinary stop: shutdown must make progress even if a
        # ramp never completes, so it does not inherit the blocking join.
        for session_id in list(self._tracker_sessions):
            try:
                self.stop_tracker(session_id, SHUTDOWN_JOIN_TIMEOUT)
            except Exception:
                pass
        for robot_id in list(self._robots):
            try:
                self.stop(robot_id)
            except Exception:
                pass

    def close(self) -> None:
        if self._state_publisher is not None:
            self._state_publisher.close()

    def get_last_teleop_state(self, robot_id: str):
        try:
            return self._latest_state[robot_id]
        except KeyError as exc:
            raise RuntimeError("No motion callback data has been received yet") from exc

    def move(self, robot_id: str, motion_payload: dict[str, Any], asynchronous: bool):
        from zero_franky.zmq_server_franky import build_franky_motion

        motion = build_franky_motion(self._franky, motion_payload)
        self._register_state_callback(robot_id, motion)
        return self._robot(robot_id).move(motion, asynchronous=asynchronous)

    def _register_state_callback(self, robot_id: str, motion) -> None:
        if not hasattr(motion, "register_callback"):
            return

        from zero_franky.protocol import encode_callback_state, encode_motion_reference

        robot = self._robot(robot_id)

        def callback(robot_state, time_step, rel_time, abs_time, control_signal):
            payload = encode_callback_state(
                robot_state, time_step, rel_time, abs_time, control_signal, fields=self._state_fields
            )
            payload["robot_id"] = robot_id
            payload["in_control"] = bool(getattr(robot, "is_in_control", True))
            # Wait-free on the motion's side, so safe on the control thread.
            payload["reference"] = encode_motion_reference(motion)
            self._latest_state[robot_id] = payload
            if self._state_publisher is not None:
                self._state_publisher.publish("robot.state", payload)

        motion.register_callback(callback)


class _TrackerUpdateListener:
    """Daemon thread that drains a CONFLATE PULL socket and applies the latest
    tracker reference directly to the active session, bypassing the RPC channel."""

    def __init__(self, bind: str, manager: "RobotManager"):
        ctx = zmq.Context.instance()
        sock = ctx.socket(zmq.PULL)
        sock.setsockopt(zmq.CONFLATE, 1)
        sock.setsockopt(zmq.RCVTIMEO, 200)
        sock.bind(bind)
        self._sock = sock
        self._manager = manager
        self._stop_event = threading.Event()
        thread = threading.Thread(target=self._run, name="zero-franky-tracker-listener", daemon=True)
        thread.start()
        self._thread = thread

    def _run(self):
        try:
            while not self._stop_event.is_set():
                try:
                    raw = self._sock.recv()
                except zmq.Again:
                    continue
                except zmq.ZMQError:
                    return
                msg = msgpack.unpackb(raw, raw=False)
                session_id = msg.get("session_id")
                kind = msg.get("kind")
                try:
                    if kind == "joint":
                        self._manager.set_joint_tracker_reference(
                            session_id,
                            msg["position"],
                            msg.get("velocity"),
                            msg.get("torque_feedforward"),
                        )
                    elif kind == "cartesian":
                        self._manager.set_cartesian_tracker_reference(
                            session_id,
                            msg["target"],
                            msg.get("target_twist"),
                            msg.get("target_acceleration"),
                        )
                    elif kind == "torque":
                        self._manager.set_session_torque(session_id, msg["torque"])
                except Exception:
                    pass
        finally:
            self._sock.close(linger=0)

    def stop(self, timeout: float = 1.0) -> None:
        self._stop_event.set()
        self._thread.join(timeout)


class ZmqRobotServer:
    def __init__(
        self,
        bind: str = "tcp://0.0.0.0:18812",
        pub_bind: str | None = "tcp://0.0.0.0:18813",
        tracker_bind: str | None = None,
        state_fields: Sequence[str] | str | None = None,
        manager: RobotManager | None = None,
    ):
        self._context = zmq.Context.instance()
        self._socket = self._context.socket(zmq.REP)
        self._socket.setsockopt(zmq.RCVTIMEO, 200)
        self._socket.bind(bind)
        if manager is None:
            from zero_franky.pubsub import StatePublisher

            publisher = None if pub_bind is None else StatePublisher(pub_bind)
            manager = RobotManager(publisher, state_fields=state_fields)
        elif state_fields is not None:
            raise ValueError("state_fields must be configured on the supplied RobotManager")
        self._manager = manager
        self._tracker_listener = (
            _TrackerUpdateListener(tracker_bind, self._manager) if tracker_bind is not None else None
        )
        self._stop_event = threading.Event()

    def serve_forever(self):
        try:
            while not self._stop_event.is_set():
                try:
                    self.serve_once()
                except zmq.Again:
                    continue
        finally:
            # Order matters: stop robots/trackers (no RPC can be in flight once the
            # loop above has exited) before tearing down the sockets they publish
            # through, all from this thread so nothing else touches the sockets.
            self._manager.shutdown()
            if self._tracker_listener is not None:
                self._tracker_listener.stop()
            self._manager.close()
            self._socket.close(linger=0)

    def shutdown(self) -> None:
        """Signal serve_forever to stop.

        Safe to call from a different thread than the one running serve_forever,
        including before serve_forever has started: it only sets an event, never
        touching robots/trackers/sockets directly. All teardown happens once,
        inside serve_forever's own finally, in the thread that owns those resources.
        """
        self._stop_event.set()

    def serve_once(self):
        request = msgpack.unpackb(self._socket.recv(), raw=False)
        try:
            result = self.dispatch(request["method"], request.get("params", {}))
            response = {"id": request["id"], "ok": True, "result": result}
        except Exception as exc:
            response = {"id": request.get("id"), "ok": False, "error": format_exception(exc)}
        self._socket.send(msgpack.packb(response, use_bin_type=True))

    def dispatch(self, method: str, params: dict[str, Any]):
        try:
            handler = RPC_HANDLERS[method]
        except KeyError as exc:
            raise NotImplementedError(method) from exc
        return handler(self._manager, params)


@rpc_handler("robot.create")
def handle_robot_create(manager: RobotManager, params: dict[str, Any]):
    return manager.create_robot(params["fci_hostname"], params.get("kwargs"))


@rpc_handler("robot.kinematics_info")
def handle_robot_kinematics_info(manager: RobotManager, params: dict[str, Any]):
    return manager.kinematics_info(params["robot_id"])


@rpc_handler("robot.recover_from_errors")
def handle_robot_recover_from_errors(manager: RobotManager, params: dict[str, Any]):
    return manager.recover_from_errors(params["robot_id"])


@rpc_handler("robot.move")
def handle_robot_move(manager: RobotManager, params: dict[str, Any]):
    return manager.move(params["robot_id"], params["motion"], params.get("asynchronous", False))


@rpc_handler("robot.join_motion")
def handle_robot_join_motion(manager: RobotManager, params: dict[str, Any]):
    return manager.join_motion(params["robot_id"], params.get("timeout"))


@rpc_handler("robot.poll_motion")
def handle_robot_poll_motion(manager: RobotManager, params: dict[str, Any]):
    return manager.poll_motion(params["robot_id"])


@rpc_handler("robot.stop")
def handle_robot_stop(manager: RobotManager, params: dict[str, Any]):
    return manager.stop(params["robot_id"])


@rpc_handler("robot.get_last_teleop_state")
def handle_robot_get_last_teleop_state(manager: RobotManager, params: dict[str, Any]):
    return manager.get_last_teleop_state(params["robot_id"])


@rpc_handler("robot.start_joint_tracker")
def handle_robot_start_joint_tracker(manager: RobotManager, params: dict[str, Any]):
    return manager.start_joint_tracker(
        params["robot_id"],
        params.get("policy"),
        params.get("motion_kwargs"),
        params.get("period", 0.001),
        params.get("stop_on_policy_error", True),
    )


@rpc_handler("robot.start_cartesian_tracker")
def handle_robot_start_cartesian_tracker(manager: RobotManager, params: dict[str, Any]):
    return manager.start_cartesian_tracker(
        params["robot_id"],
        params.get("policy"),
        params.get("motion_kwargs"),
        params.get("period", 0.001),
        params.get("stop_on_policy_error", True),
    )


@rpc_handler("robot.start_torque_tracker")
def handle_robot_start_torque_tracker(manager: RobotManager, params: dict[str, Any]):
    return manager.start_torque_tracker(params["robot_id"], params.get("motion_kwargs"))


@rpc_handler("tracker.status")
def handle_tracker_status(manager: RobotManager, params: dict[str, Any]):
    return manager.tracker_status(params["session_id"])


@rpc_handler("tracker.stop")
def handle_tracker_stop(manager: RobotManager, params: dict[str, Any]):
    return manager.stop_tracker(
        params["session_id"],
        params.get("join_timeout"),
        params.get("stop_motion"),
    )


@rpc_handler("tracker.set_joint_reference")
def handle_tracker_set_joint_reference(manager: RobotManager, params: dict[str, Any]):
    return manager.set_joint_tracker_reference(
        params["session_id"],
        params["position"],
        params.get("velocity"),
        params.get("torque_feedforward"),
    )


@rpc_handler("tracker.set_cartesian_reference")
def handle_tracker_set_cartesian_reference(manager: RobotManager, params: dict[str, Any]):
    return manager.set_cartesian_tracker_reference(
        params["session_id"],
        params["target"],
        params.get("target_twist"),
        params.get("target_acceleration"),
    )


@rpc_handler("tracker.set_joint_gains")
def handle_tracker_set_joint_gains(manager: RobotManager, params: dict[str, Any]):
    return manager.set_joint_tracker_gains(params["session_id"], params["stiffness"], params["damping"])


@rpc_handler("tracker.set_cartesian_gains")
def handle_tracker_set_cartesian_gains(manager: RobotManager, params: dict[str, Any]):
    return manager.set_cartesian_tracker_gains(params["session_id"], params["gains"])


@rpc_handler("tracker.set_joint_cartesian_gains")
def handle_tracker_set_joint_cartesian_gains(manager: RobotManager, params: dict[str, Any]):
    return manager.set_joint_tracker_cartesian_gains(params["session_id"], params["gains"])


@rpc_handler("tracker.set_posture_stiffness")
def handle_tracker_set_posture_stiffness(manager: RobotManager, params: dict[str, Any]):
    return manager.set_tracker_posture_stiffness(params["session_id"], params["posture_stiffness"])


@rpc_handler("tracker.set_nullspace_gains")
def handle_tracker_set_nullspace_gains(manager: RobotManager, params: dict[str, Any]):
    return manager.set_cartesian_tracker_nullspace_gains(params["session_id"], params["gains"])


@rpc_handler("tracker.set_torque")
def handle_tracker_set_torque(manager: RobotManager, params: dict[str, Any]):
    return manager.set_session_torque(params["session_id"], params["torque"])


@rpc_handler("tracker.get_torque")
def handle_tracker_get_torque(manager: RobotManager, params: dict[str, Any]):
    return manager.get_session_torque(params["session_id"])
