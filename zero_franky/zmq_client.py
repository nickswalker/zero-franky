from __future__ import annotations

from typing import Any
import inspect
import threading

import msgpack
import zmq

from zero_franky.protocol import RpcRequest, encode_affine, encode_motion, encode_robot_velocity, encode_rpc_value


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

    def set_cartesian_reference(self, target, target_twist=None):
        if self._kind != "cartesian":
            raise RuntimeError("set_cartesian_reference is only valid for Cartesian tracker sessions")
        if self._push is not None:
            self._push.send(
                msgpack.packb(
                    {
                        "session_id": self._id,
                        "kind": "cartesian",
                        "target": encode_affine(target),
                        "target_twist": encode_robot_velocity(target_twist) if target_twist is not None else None,
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
                "target_twist": encode_robot_velocity(target_twist) if target_twist is not None else None,
            },
        )


class ZmqRpcClient:
    def __init__(self, host: str, port: int, timeout_ms: int = 5000):
        self._context = zmq.Context.instance()
        self._socket = self._context.socket(zmq.REQ)
        self._socket.setsockopt(zmq.RCVTIMEO, timeout_ms)
        self._socket.setsockopt(zmq.SNDTIMEO, timeout_ms)
        self._socket.connect(f"tcp://{host}:{port}")

    def call(self, method: str, params: dict[str, Any] | None = None) -> Any:
        request = RpcRequest.create(method, params)
        self._socket.send(msgpack.packb(request.__dict__, use_bin_type=True))
        response = msgpack.unpackb(self._socket.recv(), raw=False)
        if response.get("id") != request.id:
            raise RuntimeError(f"RPC response id mismatch for {method}")
        if not response.get("ok", False):
            raise RuntimeError(response.get("error", "Unknown RPC error"))
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
        return self._client.call(
            "robot.move",
            {
                "robot_id": self._id,
                "motion": encode_motion(motion),
                "asynchronous": asynchronous,
            },
        )

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
