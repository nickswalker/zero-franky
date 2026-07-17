from __future__ import annotations

from typing import Any
import inspect
import threading

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


class TrackerSessionProxy:
    def __init__(self, client: "ZmqRpcClient", session_id: str, kind: str, *, push_socket=None):
        self._client = client
        self._id = session_id
        self._kind = kind
        self._push = push_socket

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.stop()
        return False

    @property
    def id(self) -> str:
        return self._id

    @property
    def kind(self) -> str:
        return self._kind

    def status(self) -> dict[str, Any]:
        return self._client.call("tracker.status", {"session_id": self._id})

    def stop(self, join_timeout: float | None = 1.0):
        return self._client.call("tracker.stop", {"session_id": self._id, "join_timeout": join_timeout})

    def set_joint_reference(
        self,
        position: list[float],
        velocity: list[float] | None = None,
        torque_feedforward: list[float] | None = None,
    ):
        if self._kind != "joint":
            raise RuntimeError("set_joint_reference is only valid for joint tracker sessions")
        if self._push is not None:
            self._push.send(
                msgpack.packb(
                    {
                        "session_id": self._id,
                        "kind": "joint",
                        "position": position,
                        "velocity": velocity,
                        "torque_feedforward": torque_feedforward,
                    },
                    use_bin_type=True,
                )
            )
            return
        return self._client.call(
            "tracker.set_joint_reference",
            {
                "session_id": self._id,
                "position": position,
                "velocity": velocity,
                "torque_feedforward": torque_feedforward,
            },
        )

    def set_cartesian_reference(self, target, target_twist=None, target_acceleration=None):
        if self._kind != "cartesian":
            raise RuntimeError("set_cartesian_reference is only valid for Cartesian tracker sessions")
        twist = encode_robot_velocity(target_twist) if target_twist is not None else None
        accel = encode_twist_acceleration(target_acceleration) if target_acceleration is not None else None
        if self._push is not None:
            self._push.send(
                msgpack.packb(
                    {
                        "session_id": self._id,
                        "kind": "cartesian",
                        "target": encode_affine(target),
                        "target_twist": twist,
                        "target_acceleration": accel,
                    },
                    use_bin_type=True,
                )
            )
            return
        return self._client.call(
            "tracker.set_cartesian_reference",
            {
                "session_id": self._id,
                "target": encode_affine(target),
                "target_twist": twist,
                "target_acceleration": accel,
            },
        )

    def set_joint_gains(self, stiffness: list[float] | Any, damping: list[float] | None = None):
        if self._kind != "joint":
            raise RuntimeError("set_joint_gains is only valid for joint tracker sessions")
        if damping is None and hasattr(stiffness, "stiffness") and hasattr(stiffness, "damping"):
            damping = stiffness.damping
            stiffness = stiffness.stiffness
        return self._client.call(
            "tracker.set_joint_gains",
            {"session_id": self._id, "stiffness": encode_rpc_value(stiffness), "damping": encode_rpc_value(damping)},
        )

    def set_cartesian_gains(
        self,
        gains=None,
        *,
        stiffness=None,
        damping=None,
        translational_stiffness: float | None = None,
        rotational_stiffness: float | None = None,
        translational_damping: float | None = None,
        rotational_damping: float | None = None,
    ):
        if self._kind != "cartesian":
            raise RuntimeError("set_cartesian_gains is only valid for Cartesian tracker sessions")
        if gains is not None:
            if any(
                value is not None
                for value in (
                    stiffness,
                    damping,
                    translational_stiffness,
                    rotational_stiffness,
                    translational_damping,
                    rotational_damping,
                )
            ):
                raise ValueError("Pass either gains or decomposed Cartesian gain fields, not both")
            payload = encode_rpc_value(gains)
        else:
            payload = {
                "stiffness": encode_rpc_value(stiffness),
                "damping": encode_rpc_value(damping),
                "translational_stiffness": translational_stiffness,
                "rotational_stiffness": rotational_stiffness,
                "translational_damping": translational_damping,
                "rotational_damping": rotational_damping,
            }
        return self._client.call("tracker.set_cartesian_gains", {"session_id": self._id, "gains": payload})

    def set_joint_cartesian_gains(
        self,
        gains=None,
        *,
        stiffness=None,
        damping=None,
        translational_stiffness: float | None = None,
        rotational_stiffness: float | None = None,
        translational_damping: float | None = None,
        rotational_damping: float | None = None,
    ):
        if self._kind != "joint":
            raise RuntimeError("set_joint_cartesian_gains is only valid for joint tracker sessions")
        if gains is not None:
            if any(
                value is not None
                for value in (
                    stiffness,
                    damping,
                    translational_stiffness,
                    rotational_stiffness,
                    translational_damping,
                    rotational_damping,
                )
            ):
                raise ValueError("Pass either gains or decomposed Cartesian gain fields, not both")
            payload = encode_rpc_value(gains)
        else:
            payload = {
                "stiffness": encode_rpc_value(stiffness),
                "damping": encode_rpc_value(damping),
                "translational_stiffness": translational_stiffness,
                "rotational_stiffness": rotational_stiffness,
                "translational_damping": translational_damping,
                "rotational_damping": rotational_damping,
            }
        return self._client.call("tracker.set_joint_cartesian_gains", {"session_id": self._id, "gains": payload})

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
        if self._kind != "cartesian":
            raise RuntimeError("set_nullspace_gains is only valid for Cartesian tracker sessions")
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


class ZmqRpcClient:
    def __init__(self, host: str, port: int, timeout_ms: int = 5000):
        self._context = zmq.Context.instance()
        self._socket = self._context.socket(zmq.REQ)
        self._socket.setsockopt(zmq.RCVTIMEO, timeout_ms)
        self._socket.setsockopt(zmq.SNDTIMEO, timeout_ms)
        self._endpoint = f"tcp://{host}:{port}"
        self._timeout_ms = timeout_ms
        self._socket.connect(self._endpoint)

    def call(self, method: str, params: dict[str, Any] | None = None) -> Any:
        request = RpcRequest.create(method, params)
        try:
            self._socket.send(msgpack.packb(request.__dict__, use_bin_type=True))
            response = msgpack.unpackb(self._socket.recv(), raw=False)
        except zmq.Again as exc:
            raise TimeoutError(
                f"RPC call {method!r} to {self._endpoint} timed out after {self._timeout_ms} ms; "
                "is the zero_franky server running and reachable?"
            ) from exc
        except zmq.ZMQError as exc:
            raise ConnectionError(f"RPC call {method!r} to {self._endpoint} failed: {exc}") from exc
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

    def start_joint_impedance_session(
        self,
        policy=None,
        *,
        policy_transport: str = "import",
        period: float = 0.001,
        stop_on_policy_error: bool = True,
        **motion_kwargs,
    ) -> TrackerSessionProxy:
        params = {
            "robot_id": self._id,
            "motion_kwargs": encode_rpc_value(motion_kwargs),
            "period": period,
            "stop_on_policy_error": stop_on_policy_error,
        }
        if policy is not None:
            params["policy"] = encode_policy(policy, policy_transport)
        session_id = self._client.call("robot.start_joint_tracker", params)
        return TrackerSessionProxy(self._client, session_id, "joint", push_socket=self._push_socket)

    def start_cartesian_impedance_session(
        self,
        policy=None,
        *,
        policy_transport: str = "import",
        period: float = 0.001,
        stop_on_policy_error: bool = True,
        **motion_kwargs,
    ) -> TrackerSessionProxy:
        params = {
            "robot_id": self._id,
            "motion_kwargs": encode_rpc_value(motion_kwargs),
            "period": period,
            "stop_on_policy_error": stop_on_policy_error,
        }
        if policy is not None:
            params["policy"] = encode_policy(policy, policy_transport)
        session_id = self._client.call("robot.start_cartesian_tracker", params)
        return TrackerSessionProxy(self._client, session_id, "cartesian", push_socket=self._push_socket)

    def state_subscriber(self, topic: str = "robot.state", timeout_ms: int = 1000):
        from zero_franky.pubsub import StateSubscriber
        from zero_franky.setup import cfg

        if cfg.PUB_PORT is None:
            raise RuntimeError("State subscription is disabled; call setup_zero_franky(ip, port) with a server using state PUB")
        return StateSubscriber(cfg.IP, cfg.PUB_PORT, topic=topic, timeout_ms=timeout_ms, robot_id=self._id)
