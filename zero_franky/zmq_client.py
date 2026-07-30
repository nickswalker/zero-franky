from __future__ import annotations

from typing import Any
import inspect
import threading
import time

import msgpack
import zmq

from zero_franky.protocol import (
    RpcError,
    RpcRequest,
    encode_affine,
    encode_motion,
    encode_robot_velocity,
    encode_rpc_value,
    encode_twist_acceleration,
)
from zero_franky.types import Affine, RobotPose, Twist, TwistAcceleration


def encode_policy(policy, transport: str = "import") -> dict[str, Any]:
    if transport == "import":
        module = inspect.getmodule(policy)
        qualname = getattr(policy, "__qualname__", None)
        if module is None or qualname is None or "<locals>" in qualname:
            raise ValueError("Import policy transport requires an importable module-level function")
        return {"transport": "import", "module": module.__name__, "qualname": qualname}
    if transport == "cloudpickle":
        import cloudpickle

        return {"transport": "cloudpickle", "payload": cloudpickle.dumps(policy)}
    raise ValueError(f"Unsupported policy transport: {transport}")


def _cartesian_gains_payload(gains, translational_stiffness, rotational_stiffness, damping):
    """Build the Cartesian gains payload, or None when nothing was specified.

    A `gains` object total-replaces and encodes to a payload carrying `stiffness`,
    which is how the server tells the two cases apart. The decomposed form omits
    it so the server overwrites only the named blocks.
    """
    stiffness_given = translational_stiffness is not None or rotational_stiffness is not None
    if gains is not None:
        if stiffness_given or damping is not None:
            raise ValueError(
                "Pass either gains (a full CartesianImpedanceGains, for anisotropic stiffness) "
                "or the isotropic translational_stiffness/rotational_stiffness/damping keywords, not both"
            )
        return encode_rpc_value(gains)
    if not stiffness_given and damping is None:
        return None
    return {
        "stiffness": None,
        "translational_stiffness": translational_stiffness,
        "rotational_stiffness": rotational_stiffness,
        "damping": encode_rpc_value(damping),
    }


class _TrackerProxy:
    """Client-side handle to a tracker running next to the robot.

    Mirrors the surface of `franky`'s in-process tracker classes, so loop bodies
    written against `franky.JointImpedanceTracker` port over unchanged. The
    differences are inherent to running over the network: there is no `tick()`
    (see `period` and the server-side policy loop instead), and `motion` cannot
    be handed back because it lives in the server process.
    """

    _kind: str

    def __init__(self, client: "ZmqRpcClient", session_id: str, *, push_socket=None, robot=None):
        self._client = client
        self._id = session_id
        self._push = push_socket
        self._robot = robot
        self._started_at = time.perf_counter()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        try:
            self.stop()
        except Exception:
            # Mirrors franky's tracker `__exit__`: a cleanup fault must not mask
            # the exception already unwinding the body, especially a
            # KeyboardInterrupt. More likely here than in-process, since the stop
            # travels over RPC and the socket may already be gone.
            if exc_type is not None:
                return False
            raise
        return False

    @property
    def id(self) -> str:
        return self._id

    @property
    def kind(self) -> str:
        return self._kind

    def status(self) -> dict[str, Any]:
        return self._client.call("tracker.status", {"session_id": self._id})

    @property
    def is_running(self) -> bool:
        """Whether the tracker is still active. Costs an RPC round trip."""
        return bool(self.status()["running"])

    @property
    def iterations(self) -> int:
        """Server-side policy loop iteration count. Costs an RPC round trip."""
        return int(self.status()["iterations"])

    @property
    def elapsed_time(self) -> float:
        """Seconds since this proxy was created."""
        return time.perf_counter() - self._started_at

    @property
    def state(self) -> dict[str, Any] | None:
        """Latest robot state, read from the state PUB stream rather than by RPC.

        Unlike `franky`'s `tracker.state` this is the msgpack snapshot dict, not a
        `RobotState`. The background stream starts on first access, so this is
        None until the first snapshot lands; use `robot.wait_for_state()` to
        block for it. Requires the server to have a `pub_bind`.
        """
        if self._robot is None:
            raise RuntimeError("This proxy was constructed without a robot; state is unavailable")
        self._robot.start_state_stream()
        return self._robot.latest_state

    def _state_field(self, key: str):
        state = self.state
        return None if state is None else state.get(key)

    @property
    def current_joint_positions(self) -> list[float] | None:
        """Measured joint positions from the latest state snapshot."""
        return self._state_field("q")

    @property
    def current_joint_velocities(self) -> list[float] | None:
        """Measured joint velocities from the latest state snapshot."""
        return self._state_field("dq")

    @property
    def last_reference(self) -> dict[str, Any] | None:
        """The reference the controller last picked up, from the state stream.

        `set_target` goes over a CONFLATE socket, so this is how a client
        confirms what landed. Carries `type` plus that reference's fields, in the
        types `set_target` takes: `q`/`dq`/`tau_ff`, or `target` as an `Affine`
        with `target_twist`/`target_acceleration`. A torque tracker reports its
        commanded `tau`. None before the first snapshot, or before any reference
        is set.
        """
        reference = self._state_field("reference")
        if reference is None or reference["type"] != "CartesianReference":
            return reference
        twist = reference["target_twist"]
        acceleration = reference["target_acceleration"]
        return {
            **reference,
            "target": Affine(reference["target"]["matrix"]),
            "target_twist": None if twist is None else Twist(twist["linear"], twist["angular"]),
            "target_acceleration": (
                None
                if acceleration is None
                else TwistAcceleration(acceleration["linear"], acceleration["angular"])
            ),
        }

    @property
    def current_pose(self) -> RobotPose | None:
        """The end-effector pose, as `franky`'s `current_pose` is."""
        matrix = self._state_field("O_T_EE")
        return None if matrix is None else RobotPose(Affine(matrix))

    def stop(self, stop_motion: Any = None, join_timeout: float | None = None, rpc_timeout_ms: int = 30_000):
        """Gracefully stop the tracker and wait for the arm to come to rest.

        Matches `franky`'s tracker `stop()`: the server enqueues a
        `TorqueStopMotion` ramp and joins it, so this returns once the arm is at
        rest. `stop_motion` overrides the ramp; a `join_timeout` float caps the
        server-side wait, at the cost of possibly returning mid-ramp.

        Since the join blocks on the robot, this call gets its own longer RPC
        timeout — the default 5 s would otherwise abort a ramp that is proceeding
        normally. Calling it twice is harmless, as the context manager relies on.
        """
        params: dict[str, Any] = {"session_id": self._id, "join_timeout": join_timeout}
        if stop_motion is not None:
            params["stop_motion"] = encode_motion(stop_motion)
        return self._client.call("tracker.stop", params, timeout_ms=rpc_timeout_ms)

    def _send_reference(self, method: str, payload: dict[str, Any]):
        """Push the reference over the conflated socket when available, else RPC."""
        if self._push is not None:
            self._push.send(
                msgpack.packb({"session_id": self._id, "kind": self._kind, **payload}, use_bin_type=True)
            )
            return None
        return self._client.call(method, {"session_id": self._id, **payload})


class JointImpedanceTrackerProxy(_TrackerProxy):
    _kind = "joint"

    def set_target(self, q, dq=None, tau_ff=None):
        """Update the joint target position, optional velocity, and optional feedforward torque."""
        return self._send_reference(
            "tracker.set_joint_reference",
            {
                "position": encode_rpc_value(q),
                "velocity": encode_rpc_value(dq),
                "torque_feedforward": encode_rpc_value(tau_ff),
            },
        )

    def set_gains(self, gains=None, *, stiffness=None, damping=None):
        """Update joint impedance gains, smoothed in the RT loop by the controller.

        Accepts a `JointImpedanceGains` positionally, or `stiffness`/`damping`
        keywords, matching `franky.JointImpedanceTracker.set_gains`. Omitted
        gains keep their current value.

        Damping is all-or-nothing: passing a 7-vector pins it, while omitting it
        (or passing `franky.CRITICAL`) leaves it unpinned so the controller
        re-tracks critical damping against the smoothed stiffness every cycle.
        A stiffness change therefore re-criticals unless damping is passed too.
        """
        if gains is not None:
            if stiffness is not None or damping is not None:
                raise ValueError("Pass either gains or stiffness/damping, not both")
            if not (hasattr(gains, "stiffness") and hasattr(gains, "damping")):
                raise TypeError("Positional argument must be a JointImpedanceGains; pass vectors as stiffness=/damping=")
            stiffness, damping = gains.stiffness, gains.damping
        elif stiffness is None and damping is None:
            # Naming neither gain changes nothing; save the round trip.
            return None
        return self._client.call(
            "tracker.set_joint_gains",
            {
                "session_id": self._id,
                "stiffness": encode_rpc_value(stiffness),
                "damping": encode_rpc_value(damping),
            },
        )

    def set_cartesian_gains(
        self,
        gains=None,
        *,
        translational_stiffness: float | None = None,
        rotational_stiffness: float | None = None,
        damping=None,
    ):
        """Update the hybrid Cartesian gain shaping added on top of the joint-space stiffness.

        Only meaningful when the tracker was started with `cartesian_stiffness`;
        the hybrid path itself is fixed for the lifetime of the motion. Takes the
        same arguments as `CartesianImpedanceTrackerProxy.set_gains` minus the
        nullspace, which a joint tracker has no tasks for.
        """
        payload = _cartesian_gains_payload(gains, translational_stiffness, rotational_stiffness, damping)
        if payload is None:
            return None
        return self._client.call("tracker.set_joint_cartesian_gains", {"session_id": self._id, "gains": payload})


class CartesianImpedanceTrackerProxy(_TrackerProxy):
    _kind = "cartesian"

    def set_target(self, pose, twist=None, acceleration=None):
        """Update the Cartesian target pose and optional twist/acceleration feedforward."""
        return self._send_reference(
            "tracker.set_cartesian_reference",
            {
                "target": encode_affine(pose),
                "target_twist": encode_robot_velocity(twist) if twist is not None else None,
                "target_acceleration": encode_twist_acceleration(acceleration) if acceleration is not None else None,
            },
        )

    def set_gains(
        self,
        gains=None,
        *,
        translational_stiffness: float | None = None,
        rotational_stiffness: float | None = None,
        damping=None,
        posture_stiffness=None,
        nullspace_gains=None,
    ):
        """Update Cartesian impedance gains, smoothed in the RT loop by the controller.

        Mirrors `franky.CartesianImpedanceTracker.set_gains`.
        `translational_stiffness`/`rotational_stiffness` overwrite one stiffness
        block each, leaving the rest of the matrix (including anisotropy) intact.
        `gains` total-replaces with a full `CartesianImpedanceGains` and is
        exclusive with the rest.

        Damping is all-or-nothing: a 6-vector pins it, while omitting it or
        passing `franky.CRITICAL` unpins it so the controller re-tracks critical
        damping. A stiffness change therefore re-criticals unless damping is
        passed too.

        For the nullspace, `posture_stiffness` nudges just the posture task's
        stiffness; `nullspace_gains` replaces the whole `NullspaceGains`. The two
        are mutually exclusive, and both only retune tasks configured at start.
        """
        if nullspace_gains is not None and posture_stiffness is not None:
            raise ValueError("Pass either nullspace_gains or posture_stiffness, not both")

        payload = _cartesian_gains_payload(gains, translational_stiffness, rotational_stiffness, damping)
        if payload is not None:
            self._client.call("tracker.set_cartesian_gains", {"session_id": self._id, "gains": payload})
        if nullspace_gains is not None:
            self.set_nullspace_gains(nullspace_gains)
        elif posture_stiffness is not None:
            self._client.call(
                "tracker.set_posture_stiffness",
                {"session_id": self._id, "posture_stiffness": encode_rpc_value(posture_stiffness)},
            )
        return None

    def set_nullspace_gains(
        self,
        gains=None,
        *,
        posture_stiffness: Any = 0.0,
        posture_damping: Any | None = None,
        posture_max_torque: float | None = None,
        manipulability_gain: float = 0.0,
        manipulability_damping: float = 0.0,
        manipulability_max_torque: float | None = None,
    ):
        """Update the nullspace gains acting on the redundant joint."""
        if gains is not None:
            payload = encode_rpc_value(gains)
        else:
            payload = {
                "posture_stiffness": encode_rpc_value(posture_stiffness),
                "posture_damping": encode_rpc_value(posture_damping),
                "posture_max_torque": posture_max_torque,
                "manipulability_gain": manipulability_gain,
                "manipulability_damping": manipulability_damping,
                "manipulability_max_torque": manipulability_max_torque,
            }
        return self._client.call("tracker.set_nullspace_gains", {"session_id": self._id, "gains": payload})


class TorqueTrackerProxy(_TrackerProxy):
    _kind = "torque"

    def set_torque(self, torque):
        """Command raw joint torques. Call faster than the motion's `signal_timeout`."""
        return self._send_reference("tracker.set_torque", {"torque": encode_rpc_value(torque)})

    def get_torque(self):
        """The torque the server most recently handed to the controller."""
        return self._client.call("tracker.get_torque", {"session_id": self._id})


class ZmqRpcClient:
    def __init__(self, host: str, port: int, timeout_ms: int = 5000):
        self._context = zmq.Context.instance()
        self._socket = self._context.socket(zmq.REQ)
        self._socket.setsockopt(zmq.RCVTIMEO, timeout_ms)
        self._socket.setsockopt(zmq.SNDTIMEO, timeout_ms)
        self._endpoint = f"tcp://{host}:{port}"
        self._timeout_ms = timeout_ms
        self._socket.connect(self._endpoint)

    def call(self, method: str, params: dict[str, Any] | None = None, *, timeout_ms: int | None = None) -> Any:
        """Issue an RPC call, optionally overriding the receive timeout.

        `timeout_ms` is for the calls that legitimately block on the robot rather
        than on the network — joining a motion, ramping a tracker to rest — where
        the default would time out mid-manoeuvre and report a failure that did not
        happen.
        """
        request = RpcRequest.create(method, params)
        effective_timeout_ms = self._timeout_ms if timeout_ms is None else timeout_ms
        try:
            if timeout_ms is not None:
                self._socket.setsockopt(zmq.RCVTIMEO, timeout_ms)
            self._socket.send(msgpack.packb(request.__dict__, use_bin_type=True))
            response = msgpack.unpackb(self._socket.recv(), raw=False)
        except zmq.Again as exc:
            raise TimeoutError(
                f"RPC call {method!r} to {self._endpoint} timed out after {effective_timeout_ms} ms; "
                "is the zero_franky server running and reachable?"
            ) from exc
        except zmq.ZMQError as exc:
            raise ConnectionError(f"RPC call {method!r} to {self._endpoint} failed: {exc}") from exc
        finally:
            if timeout_ms is not None:
                self._socket.setsockopt(zmq.RCVTIMEO, self._timeout_ms)
        if response.get("id") != request.id:
            raise RpcError(f"RPC response id mismatch for {method!r}")
        if not response.get("ok", False):
            raise RpcError(f"{method!r} failed: {response.get('error', 'Unknown RPC error')}")
        return response.get("result")


class RobotProxy:
    def __init__(self, fci_hostname: str, *, client: ZmqRpcClient | None = None, **kwargs):
        from zero_franky.setup import cfg

        self._push_socket = None
        if client is None:
            if not cfg.IS_SETUP:
                raise RuntimeError("Call setup_zero_franky(ip, port) before creating a Robot")
            client = ZmqRpcClient(cfg.IP, cfg.PORT)
            if cfg.TRACKER_PORT is not None:
                push = zmq.Context.instance().socket(zmq.PUSH)
                push.setsockopt(zmq.CONFLATE, 1)
                push.connect(f"tcp://{cfg.IP}:{cfg.TRACKER_PORT}")
                self._push_socket = push
        self._client = client
        self._id = self._client.call("robot.create", {"fci_hostname": fci_hostname, "kwargs": kwargs})
        self._state_condition = threading.Condition()
        self._latest_state: dict[str, Any] | None = None
        self._state_stream_error: Exception | None = None
        self._state_stream_stop = threading.Event()
        self._state_stream_thread: threading.Thread | None = None

    @property
    def id(self) -> str:
        return self._id

    @property
    def latest_state(self) -> dict[str, Any] | None:
        with self._state_condition:
            return self._latest_state

    def wait_for_state(self, timeout: float | None = None) -> dict[str, Any]:
        with self._state_condition:
            self._state_condition.wait_for(
                lambda: self._latest_state is not None or self._state_stream_error is not None,
                timeout=timeout,
            )
            if self._state_stream_error is not None:
                raise self._state_stream_error
            if self._latest_state is None:
                raise TimeoutError("Timed out waiting for robot state")
            return self._latest_state

    def start_state_stream(self, topic: str = "robot.state", timeout_ms: int = 250) -> None:
        if self._state_stream_thread is not None and self._state_stream_thread.is_alive():
            return
        self._state_stream_stop.clear()
        self._state_stream_error = None

        def worker() -> None:
            try:
                subscriber = self.state_subscriber(topic=topic, timeout_ms=timeout_ms)
            except Exception as exc:
                self._set_state_stream_error(exc)
                return

            while not self._state_stream_stop.is_set():
                try:
                    _topic, state = subscriber.recv()
                except Exception as exc:
                    if type(exc).__name__ == "Again":
                        continue
                    self._set_state_stream_error(exc)
                    break
                with self._state_condition:
                    self._latest_state = state
                    self._state_condition.notify_all()

        self._state_stream_thread = threading.Thread(
            target=worker,
            name=f"zero-franky-state-{self._id}",
            daemon=True,
        )
        self._state_stream_thread.start()

    def stop_state_stream(self, join_timeout: float | None = 1.0) -> None:
        self._state_stream_stop.set()
        thread = self._state_stream_thread
        if thread is not None and threading.current_thread() is not thread:
            thread.join(join_timeout)

    def _set_state_stream_error(self, exc: Exception) -> None:
        with self._state_condition:
            self._state_stream_error = exc
            self._state_condition.notify_all()

    def recover_from_errors(self):
        return self._client.call("robot.recover_from_errors", {"robot_id": self._id})

    def move(self, motion, asynchronous: bool = False):
        params = {
            "robot_id": self._id,
            "motion": encode_motion(motion),
            "asynchronous": asynchronous,
        }
        if asynchronous:
            return self._client.call("robot.move", params)

        # Secretly, this'll be async on the server end, so we don't
        # leave the move call RPC open past the configured timeout
        params["asynchronous"] = True
        result = self._client.call("robot.move", params)
        while not self.join_motion(timeout=0.1):
            pass
        return result

    def join_motion(self, timeout: float | None = None) -> bool:
        return bool(self._client.call("robot.join_motion", {"robot_id": self._id, "timeout": timeout}))

    def poll_motion(self) -> bool:
        return bool(self._client.call("robot.poll_motion", {"robot_id": self._id}))

    def stop(self):
        return self._client.call("robot.stop", {"robot_id": self._id})

    def get_last_teleop_state(self):
        return self._client.call("robot.get_last_teleop_state", {"robot_id": self._id})

    def _start_tracker(self, method: str, policy, policy_transport: str, period: float, stop_on_policy_error: bool, motion_kwargs) -> str:
        params = {
            "robot_id": self._id,
            "motion_kwargs": encode_rpc_value(motion_kwargs),
            "period": period,
            "stop_on_policy_error": stop_on_policy_error,
        }
        if policy is not None:
            params["policy"] = encode_policy(policy, policy_transport)
        return self._client.call(method, params)

    def start_joint_impedance_tracker(
        self,
        policy=None,
        *,
        policy_transport: str = "import",
        period: float = 0.001,
        stop_on_policy_error: bool = True,
        **motion_kwargs,
    ) -> JointImpedanceTrackerProxy:
        """Start joint impedance tracking, the networked counterpart to `franky.JointImpedanceTracker`.

        Without a `policy`, the returned proxy is a passthrough: set references
        from client code with `set_target`. With one, the server runs the policy
        loop at `period` next to the robot.
        """
        session_id = self._start_tracker(
            "robot.start_joint_tracker", policy, policy_transport, period, stop_on_policy_error, motion_kwargs
        )
        return JointImpedanceTrackerProxy(self._client, session_id, push_socket=self._push_socket, robot=self)

    def start_cartesian_impedance_tracker(
        self,
        policy=None,
        *,
        policy_transport: str = "import",
        period: float = 0.001,
        stop_on_policy_error: bool = True,
        **motion_kwargs,
    ) -> CartesianImpedanceTrackerProxy:
        """Start Cartesian impedance tracking, the networked counterpart to `franky.CartesianImpedanceTracker`."""
        session_id = self._start_tracker(
            "robot.start_cartesian_tracker", policy, policy_transport, period, stop_on_policy_error, motion_kwargs
        )
        return CartesianImpedanceTrackerProxy(self._client, session_id, push_socket=self._push_socket, robot=self)

    def start_torque_tracker(self, **motion_kwargs) -> TorqueTrackerProxy:
        """Start direct joint-torque control.

        Call ``set_torque`` more frequently than ``signal_timeout`` (50 ms by
        default). The server-side franky watchdog terminates stale streams.
        """
        params = {"robot_id": self._id, "motion_kwargs": encode_rpc_value(motion_kwargs)}
        session_id = self._client.call("robot.start_torque_tracker", params)
        return TorqueTrackerProxy(self._client, session_id, push_socket=self._push_socket, robot=self)

    def state_subscriber(self, topic: str = "robot.state", timeout_ms: int = 1000):
        from zero_franky.pubsub import StateSubscriber
        from zero_franky.setup import cfg

        if cfg.PUB_PORT is None:
            raise RuntimeError("State subscription is disabled; call setup_zero_franky(ip, port) with a server using state PUB")
        return StateSubscriber(cfg.IP, cfg.PUB_PORT, topic=topic, timeout_ms=timeout_ms, robot_id=self._id)
