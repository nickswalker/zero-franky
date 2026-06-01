from __future__ import annotations

from typing import Any
import threading
import uuid

import msgpack
import zmq


RPC_HANDLERS = {}


def rpc_handler(method: str):
    def register(fn):
        RPC_HANDLERS[method] = fn
        return fn

    return register


class RobotManager:
    def __init__(self, state_publisher=None):
        import franky

        self._franky = franky
        self._state_publisher = state_publisher
        self._robots: dict[str, Any] = {}
        self._latest_state: dict[str, dict[str, Any]] = {}
        self._tracker_sessions: dict[str, Any] = {}

    def create_robot(self, fci_hostname: str, kwargs: dict[str, Any] | None = None) -> str:
        robot_id = uuid.uuid4().hex
        self._robots[robot_id] = self._franky.Robot(fci_hostname, **(kwargs or {}))
        return robot_id

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
        period: float = 0.001,
        stop_on_policy_error: bool = True,
    ) -> str:
        from zero_franky.tracker_session import TrackerSession, load_policy
        from zero_franky.zmq_server_franky import franky_motion_kwargs

        robot = self._robot(robot_id)
        reference_handle = self._franky.JointReferenceHandle()
        reference_handle.set(robot.current_joint_positions, robot.current_joint_velocities)
        motion = self._franky.JointImpedanceTrackingMotion(
            reference_handle,
            **franky_motion_kwargs(self._franky, motion_kwargs),
        )
        self._register_state_callback(robot_id, motion)
        robot.move(motion, asynchronous=True)
        session = TrackerSession(
            franky=self._franky,
            robot=robot,
            kind="joint",
            policy_factory=load_policy(policy_payload) if policy_payload is not None else None,
            reference_handle=reference_handle,
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
        period: float = 0.001,
        stop_on_policy_error: bool = True,
    ) -> str:
        from zero_franky.tracker_session import TrackerSession, load_policy
        from zero_franky.zmq_server_franky import franky_motion_kwargs

        robot = self._robot(robot_id)
        reference_handle = self._franky.CartesianReferenceHandle()
        reference_handle.set(robot.current_pose.end_effector_pose)
        motion = self._franky.CartesianImpedanceTrackingMotion(
            reference_handle,
            **franky_motion_kwargs(self._franky, motion_kwargs),
        )
        self._register_state_callback(robot_id, motion)
        robot.move(motion, asynchronous=True)
        session = TrackerSession(
            franky=self._franky,
            robot=robot,
            kind="cartesian",
            policy_factory=load_policy(policy_payload) if policy_payload is not None else None,
            reference_handle=reference_handle,
            period=period,
            stop_on_policy_error=stop_on_policy_error,
        )
        self._tracker_sessions[session.id] = session
        return session.start()

    def tracker_status(self, session_id: str):
        return self._tracker_session(session_id).status()

    def stop_tracker(self, session_id: str, join_timeout: float | None = 1.0):
        self._tracker_session(session_id).stop(join_timeout)
        self._tracker_sessions.pop(session_id, None)
        return True

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
    ):
        from zero_franky.zmq_server_franky import _franky_affine, _franky_robot_velocity

        target = _franky_affine(self._franky, target_payload)
        target_twist = _franky_robot_velocity(self._franky, target_twist_payload) if target_twist_payload else None
        self._tracker_session(session_id).set_cartesian_reference(target, target_twist)
        return True

    def _tracker_session(self, session_id: str):
        try:
            return self._tracker_sessions[session_id]
        except KeyError as exc:
            raise KeyError(f"Unknown tracker session id: {session_id}") from exc

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

        from zero_franky.protocol import encode_callback_state

        def callback(robot_state, time_step, rel_time, abs_time, control_signal):
            payload = encode_callback_state(robot_state, time_step, rel_time, abs_time, control_signal)
            payload["robot_id"] = robot_id
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
        thread = threading.Thread(target=self._run, name="zero-franky-tracker-listener", daemon=True)
        thread.start()

    def _run(self):
        while True:
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
                    )
            except Exception:
                pass


class ZmqRobotServer:
    def __init__(
        self,
        bind: str = "tcp://0.0.0.0:18812",
        pub_bind: str | None = "tcp://0.0.0.0:18813",
        tracker_bind: str | None = None,
        manager: RobotManager | None = None,
    ):
        self._context = zmq.Context.instance()
        self._socket = self._context.socket(zmq.REP)
        self._socket.bind(bind)
        if manager is None and pub_bind is not None:
            from zero_franky.pubsub import StatePublisher

            manager = RobotManager(StatePublisher(pub_bind))
        self._manager = manager or RobotManager()
        if tracker_bind is not None:
            _TrackerUpdateListener(tracker_bind, self._manager)

    def serve_forever(self):
        while True:
            self.serve_once()

    def serve_once(self):
        request = msgpack.unpackb(self._socket.recv(), raw=False)
        try:
            result = self.dispatch(request["method"], request.get("params", {}))
            response = {"id": request["id"], "ok": True, "result": result}
        except Exception as exc:
            response = {"id": request.get("id"), "ok": False, "error": f"{type(exc).__name__}: {exc}"}
        self._socket.send(msgpack.packb(response, use_bin_type=True))

    def dispatch(self, method: str, params: dict[str, Any]):
        try:
            return RPC_HANDLERS[method](self._manager, params)
        except KeyError as exc:
            raise NotImplementedError(method) from exc


@rpc_handler("robot.create")
def handle_robot_create(manager: RobotManager, params: dict[str, Any]):
    return manager.create_robot(params["fci_hostname"], params.get("kwargs"))


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


@rpc_handler("tracker.status")
def handle_tracker_status(manager: RobotManager, params: dict[str, Any]):
    return manager.tracker_status(params["session_id"])


@rpc_handler("tracker.stop")
def handle_tracker_stop(manager: RobotManager, params: dict[str, Any]):
    return manager.stop_tracker(params["session_id"], params.get("join_timeout", 1.0))


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
    )
