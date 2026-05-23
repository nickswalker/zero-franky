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
    def __init__(self, host: str, port: int, topic: str = "robot.state", timeout_ms: int = 1000):
        self._context = zmq.Context.instance()
        self._socket = self._context.socket(zmq.SUB)
        self._socket.setsockopt(zmq.RCVTIMEO, timeout_ms)
        self._socket.setsockopt_string(zmq.SUBSCRIBE, topic)
        self._socket.connect(f"tcp://{host}:{port}")

    def recv(self) -> tuple[str, dict[str, Any]]:
        topic, payload = self._socket.recv_multipart()
        return topic.decode("utf-8"), msgpack.unpackb(payload, raw=False)
