from __future__ import annotations

from typing import Any

import msgpack
import zmq


class StatePublisher:
    def __init__(self, bind: str = "tcp://0.0.0.0:18813"):
        self._context = zmq.Context.instance()
        self._socket = self._context.socket(zmq.PUB)
        self._socket.bind(bind)

    def publish(self, topic: str, payload: dict[str, Any]) -> None:
        self._socket.send_multipart(
            [
                topic.encode("utf-8"),
                msgpack.packb(payload, use_bin_type=True),
            ]
        )


class StateSubscriber:
    def __init__(
        self,
        host: str,
        port: int,
        topic: str = "robot.state",
        timeout_ms: int = 1000,
        robot_id: str | None = None,
    ):
        self._robot_id = robot_id
        self._context = zmq.Context.instance()
        self._socket = self._context.socket(zmq.SUB)
        self._socket.setsockopt(zmq.RCVTIMEO, timeout_ms)
        self._socket.setsockopt_string(zmq.SUBSCRIBE, topic)
        self._socket.connect(f"tcp://{host}:{port}")

    def recv(self) -> tuple[str, dict[str, Any]]:
        while True:
            topic, payload = self._socket.recv_multipart()
            decoded_topic = topic.decode("utf-8")
            decoded_payload = msgpack.unpackb(payload, raw=False)
            if self._robot_id is None or decoded_payload.get("robot_id") == self._robot_id:
                return decoded_topic, decoded_payload
