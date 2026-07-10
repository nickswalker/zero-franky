"""ZMQ server and client proxy for pyRobotiqGripper.

Server side (run on the robot host where the gripper is physically reachable):

    zero-franky gripper serve --com-port /dev/ttyUSB0

Or run it together with the robot RPC server:

    zero-franky server --robotiq --com-port /dev/ttyUSB0

Or copy just this file to the robot (no rest of the package needed) and run:

    python -m zero_franky.robotiq serve --com-port /dev/ttyUSB0

Client side:

    from zero_franky.robotiq import RobotiqGripperProxy

    gripper = RobotiqGripperProxy("robot-hostname-or-ip")
    gripper.activate()
    gripper.open()
    gripper.close()

    # Subscribe to state broadcasts published after each command:
    sub = gripper.state_subscriber()
    topic, state = sub.recv()   # {"position": int|None, "status": {...}}
"""

from __future__ import annotations

import uuid
from collections.abc import Sequence
from dataclasses import dataclass
from typing import Any


DEFAULT_PORT = 18815
PUB_PORT_OFFSET = 1          # state PUB on DEFAULT_PORT + 1 = 18816
DEFAULT_BIND_HOST = "0.0.0.0"
DEFAULT_GRIPPER_MAX_WIDTH_M = 0.085


# ---------------------------------------------------------------------------
# Shared helpers
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class RobotiqConnectionConfig:
    com_port: str = "auto"
    device_id: int = 9
    connection_type: str = "RTU"
    tcp_host: str = "127.0.0.1"
    tcp_port: int = 54321
    debug: bool = False


def _clamp_byte(value: float | int, name: str) -> int:
    value_int = int(round(value))
    if not 0 <= value_int <= 255:
        raise ValueError(f"{name} must be in [0, 255], got {value!r}")
    return value_int


def _width_m_to_position(width_m: float, max_width_m: float) -> int:
    if max_width_m <= 0.0:
        raise ValueError(f"max_width_m must be positive, got {max_width_m!r}")
    width_m = min(max(float(width_m), 0.0), max_width_m)
    return int(round((1.0 - (width_m / max_width_m)) * 255.0))


def _position_to_width_m(position: float | int, max_width_m: float) -> float:
    position = _clamp_byte(position, "position")
    return (1.0 - (position / 255.0)) * max_width_m


# ---------------------------------------------------------------------------
# PUB/SUB (self-contained so the file can be dropped on the robot as-is)
# ---------------------------------------------------------------------------

class GripperStatePublisher:
    TOPIC = "gripper.state"

    def __init__(self, bind: str) -> None:
        import zmq

        ctx = zmq.Context.instance()
        self._socket = ctx.socket(zmq.PUB)
        self._socket.bind(bind)

    def publish(self, payload: dict[str, Any]) -> None:
        import msgpack

        self._socket.send_multipart([
            self.TOPIC.encode(),
            msgpack.packb(payload, use_bin_type=True),
        ])


class GripperStateSubscriber:
    def __init__(self, host: str, port: int, timeout_ms: int = 1000) -> None:
        import zmq

        ctx = zmq.Context.instance()
        sock = ctx.socket(zmq.SUB)
        sock.setsockopt(zmq.RCVTIMEO, timeout_ms)
        sock.setsockopt_string(zmq.SUBSCRIBE, GripperStatePublisher.TOPIC)
        sock.connect(f"tcp://{host}:{port}")
        self._socket = sock

    def recv(self) -> tuple[str, dict[str, Any]]:
        import msgpack

        topic, payload = self._socket.recv_multipart()
        return topic.decode(), msgpack.unpackb(payload, raw=False)


# ---------------------------------------------------------------------------
# Server side
# ---------------------------------------------------------------------------

RPC_HANDLERS: dict[str, Any] = {}


def rpc_handler(method: str):
    def register(fn):
        RPC_HANDLERS[method] = fn
        return fn
    return register


class GripperManager:
    """Owns the pyrobotiqgripper instance; called only from the server process."""

    def __init__(self, config: RobotiqConnectionConfig, publisher: GripperStatePublisher | None = None) -> None:
        try:
            import pyrobotiqgripper
        except ModuleNotFoundError as exc:
            raise RuntimeError(
                "Robotiq server support is not installed; install zero-franky[robotiq]"
            ) from exc

        self._gripper = pyrobotiqgripper.RobotiqGripper(
            com_port=config.com_port,
            device_id=config.device_id,
            connection_type=config.connection_type,
            tcp_host=config.tcp_host,
            tcp_port=config.tcp_port,
            debug=config.debug,
        )
        self._publisher = publisher

    def _publish_state(self) -> None:
        if self._publisher is None:
            return
        try:
            self._publisher.publish({
                "position": self._gripper.position(refreshStatus=True),
                "status": dict(self._gripper.status(refreshStatus=False)),
            })
        except Exception:
            pass

    def activate(self, reset: bool, start: bool, refresh_status: bool) -> None:
        self._gripper.activate(reset=reset, start=start, refreshStatus=refresh_status)
        self._publish_state()

    def start(self, refresh_status: bool) -> None:
        self._gripper.start(refreshStatus=refresh_status)
        self._publish_state()

    def reset(self) -> None:
        self._gripper.reset()

    def stop(self) -> None:
        self._gripper.stop()

    def open(self, speed: int, force: int, wait: bool, read_status: bool, refresh_status: bool) -> None:
        self._gripper.open(
            speed=speed, force=force, wait=wait,
            readStatus=read_status, refreshStatus=refresh_status,
        )
        self._publish_state()

    def close(self, speed: int, force: int, wait: bool, read_status: bool, refresh_status: bool) -> None:
        self._gripper.close(
            speed=speed, force=force, wait=wait,
            readStatus=read_status, refreshStatus=refresh_status,
        )
        self._publish_state()

    def move(self, position: int, speed: int, force: int, wait: bool, read_status: bool, refresh_status: bool) -> None:
        self._gripper.move(
            position=position, speed=speed, force=force, wait=wait,
            readStatus=read_status, refreshStatus=refresh_status,
        )
        self._publish_state()

    def move_mm(self, width_mm: float, speed: int, force: int, wait: bool, read_status: bool, refresh_status: bool) -> None:
        self._gripper.move_mm(
            positionmm=width_mm, speed=speed, force=force, wait=wait,
            readStatus=read_status, refreshStatus=refresh_status,
        )
        self._publish_state()

    def calibrate_mm(self, close_mm: float, open_mm: float) -> None:
        self._gripper.calibrate_mm(closemm=close_mm, openmm=open_mm)

    def position(self, refresh_status: bool) -> int | None:
        return self._gripper.position(refreshStatus=refresh_status)

    def position_mm(self, refresh_status: bool) -> float:
        return self._gripper.position_mm(refreshStatus=refresh_status)

    def status(self, refresh_status: bool) -> dict[str, Any]:
        return dict(self._gripper.status(refreshStatus=refresh_status))

    def object_detection(self, refresh_status: bool) -> int:
        return self._gripper.objectDetection(refreshStatus=refresh_status)

    def disconnect(self) -> None:
        self._gripper.disconnect()


class ZmqGripperServer:
    def __init__(
        self,
        bind: str = f"tcp://{DEFAULT_BIND_HOST}:{DEFAULT_PORT}",
        pub_bind: str | None = None,
        manager: GripperManager | None = None,
    ) -> None:
        import zmq

        ctx = zmq.Context.instance()
        self._socket = ctx.socket(zmq.REP)
        self._socket.bind(bind)
        publisher = GripperStatePublisher(pub_bind) if pub_bind is not None else None
        if manager is not None and publisher is not None:
            manager._publisher = publisher
        self._manager = manager  # type: ignore[assignment]

    def serve_forever(self) -> None:
        while True:
            self.serve_once()

    def serve_once(self) -> None:
        import msgpack

        request = msgpack.unpackb(self._socket.recv(), raw=False)
        try:
            result = self._dispatch(request["method"], request.get("params", {}))
            response = {"id": request["id"], "ok": True, "result": result}
        except Exception as exc:
            response = {"id": request.get("id"), "ok": False, "error": f"{type(exc).__name__}: {exc}"}
        self._socket.send(msgpack.packb(response, use_bin_type=True))

    def _dispatch(self, method: str, params: dict[str, Any]) -> Any:
        handler = RPC_HANDLERS.get(method)
        if handler is None:
            raise NotImplementedError(method)
        return handler(self._manager, params)


@rpc_handler("gripper.activate")
def _h_activate(m: GripperManager, p: dict) -> None:
    m.activate(p.get("reset", True), p.get("start", True), p.get("refresh_status", True))


@rpc_handler("gripper.start")
def _h_start(m: GripperManager, p: dict) -> None:
    m.start(p.get("refresh_status", True))


@rpc_handler("gripper.reset")
def _h_reset(m: GripperManager, p: dict) -> None:
    m.reset()


@rpc_handler("gripper.stop")
def _h_stop(m: GripperManager, p: dict) -> None:
    m.stop()


@rpc_handler("gripper.open")
def _h_open(m: GripperManager, p: dict) -> None:
    m.open(p.get("speed", 255), p.get("force", 255), p.get("wait", True), p.get("read_status", True), p.get("refresh_status", False))


@rpc_handler("gripper.close")
def _h_close(m: GripperManager, p: dict) -> None:
    m.close(p.get("speed", 255), p.get("force", 255), p.get("wait", True), p.get("read_status", True), p.get("refresh_status", False))


@rpc_handler("gripper.move")
def _h_move(m: GripperManager, p: dict) -> None:
    m.move(p["position"], p.get("speed", 255), p.get("force", 255), p.get("wait", True), p.get("read_status", True), p.get("refresh_status", False))


@rpc_handler("gripper.move_mm")
def _h_move_mm(m: GripperManager, p: dict) -> None:
    m.move_mm(p["width_mm"], p.get("speed", 255), p.get("force", 255), p.get("wait", True), p.get("read_status", True), p.get("refresh_status", False))


@rpc_handler("gripper.calibrate_mm")
def _h_calibrate_mm(m: GripperManager, p: dict) -> None:
    m.calibrate_mm(p.get("close_mm", 0.0), p.get("open_mm", 85.0))


@rpc_handler("gripper.position")
def _h_position(m: GripperManager, p: dict) -> int | None:
    return m.position(p.get("refresh_status", True))


@rpc_handler("gripper.position_mm")
def _h_position_mm(m: GripperManager, p: dict) -> float:
    return m.position_mm(p.get("refresh_status", True))


@rpc_handler("gripper.status")
def _h_status(m: GripperManager, p: dict) -> dict[str, Any]:
    return m.status(p.get("refresh_status", True))


@rpc_handler("gripper.object_detection")
def _h_object_detection(m: GripperManager, p: dict) -> int:
    return m.object_detection(p.get("refresh_status", True))


@rpc_handler("gripper.disconnect")
def _h_disconnect(m: GripperManager, p: dict) -> None:
    m.disconnect()


# ---------------------------------------------------------------------------
# Client side
# ---------------------------------------------------------------------------

class _ZmqRpcClient:
    def __init__(self, host: str, port: int, timeout_ms: int = 5000) -> None:
        import zmq

        ctx = zmq.Context.instance()
        sock = ctx.socket(zmq.REQ)
        sock.setsockopt(zmq.RCVTIMEO, timeout_ms)
        sock.setsockopt(zmq.SNDTIMEO, timeout_ms)
        sock.connect(f"tcp://{host}:{port}")
        self._socket = sock

    def call(self, method: str, params: dict[str, Any] | None = None) -> Any:
        import msgpack

        req_id = uuid.uuid4().hex
        self._socket.send(msgpack.packb({"id": req_id, "method": method, "params": params or {}}, use_bin_type=True))
        response = msgpack.unpackb(self._socket.recv(), raw=False)
        if response.get("id") != req_id:
            raise RuntimeError(f"RPC response id mismatch for {method}")
        if not response.get("ok", False):
            raise RuntimeError(response.get("error", "Unknown RPC error"))
        return response.get("result")

    def close(self) -> None:
        self._socket.close(linger=0)


class RobotiqGripperProxy:
    """Network proxy for pyRobotiqGripper through a ZMQ REQ/REP server."""

    def __init__(
        self,
        server_host: str,
        server_port: int = DEFAULT_PORT,
        *,
        pub_port: int | None = None,
        auto_activate: bool = False,
    ) -> None:
        self.server_host = server_host
        self.server_port = server_port
        self._pub_port = pub_port if pub_port is not None else server_port + PUB_PORT_OFFSET
        self._client = _ZmqRpcClient(server_host, server_port)
        if auto_activate:
            self.activate()

    def __enter__(self) -> "RobotiqGripperProxy":
        return self

    def __exit__(self, *_exc: object) -> None:
        self.disconnect()

    def state_subscriber(self, timeout_ms: int = 1000) -> GripperStateSubscriber:
        """Return a subscriber for state broadcasts published after each command."""
        return GripperStateSubscriber(self.server_host, self._pub_port, timeout_ms=timeout_ms)

    def disconnect(self) -> None:
        try:
            self._client.call("gripper.disconnect")
        finally:
            self._client.close()

    def activate(self, reset: bool = True, start: bool = True, refresh_status: bool = True) -> None:
        self._client.call("gripper.activate", {"reset": reset, "start": start, "refresh_status": refresh_status})

    def start(self, refresh_status: bool = True) -> None:
        self._client.call("gripper.start", {"refresh_status": refresh_status})

    def reset(self) -> None:
        self._client.call("gripper.reset", {})

    def stop(self) -> None:
        self._client.call("gripper.stop", {})

    def open(
        self,
        speed: int = 255,
        force: int = 255,
        wait: bool = True,
        read_status: bool = True,
        refresh_status: bool = False,
    ) -> None:
        self._client.call("gripper.open", {
            "speed": _clamp_byte(speed, "speed"),
            "force": _clamp_byte(force, "force"),
            "wait": wait,
            "read_status": read_status,
            "refresh_status": refresh_status,
        })

    def close(
        self,
        speed: int = 255,
        force: int = 255,
        wait: bool = True,
        read_status: bool = True,
        refresh_status: bool = False,
    ) -> None:
        self._client.call("gripper.close", {
            "speed": _clamp_byte(speed, "speed"),
            "force": _clamp_byte(force, "force"),
            "wait": wait,
            "read_status": read_status,
            "refresh_status": refresh_status,
        })

    def move_position(
        self,
        position: int,
        speed: int = 255,
        force: int = 255,
        wait: bool = True,
        read_status: bool = True,
        refresh_status: bool = False,
    ) -> None:
        """Move using native 0=open, 255=closed units."""
        self._client.call("gripper.move", {
            "position": _clamp_byte(position, "position"),
            "speed": _clamp_byte(speed, "speed"),
            "force": _clamp_byte(force, "force"),
            "wait": wait,
            "read_status": read_status,
            "refresh_status": refresh_status,
        })

    def move_mm(
        self,
        width_mm: float,
        speed: int = 255,
        force: int = 255,
        wait: bool = True,
        read_status: bool = True,
        refresh_status: bool = False,
    ) -> None:
        """Move by calibrated opening width in millimeters."""
        self._client.call("gripper.move_mm", {
            "width_mm": width_mm,
            "speed": _clamp_byte(speed, "speed"),
            "force": _clamp_byte(force, "force"),
            "wait": wait,
            "read_status": read_status,
            "refresh_status": refresh_status,
        })

    def move_width(
        self,
        width_m: float,
        speed: int = 255,
        force: int = 255,
        wait: bool = True,
        max_width_m: float = DEFAULT_GRIPPER_MAX_WIDTH_M,
    ) -> None:
        """Move by opening width in meters without requiring calibration."""
        self.move_position(
            _width_m_to_position(width_m, max_width_m),
            speed=speed,
            force=force,
            wait=wait,
        )

    def calibrate_mm(self, close_mm: float = 0.0, open_mm: float = 85.0) -> None:
        self._client.call("gripper.calibrate_mm", {"close_mm": close_mm, "open_mm": open_mm})

    def position(self, refresh_status: bool = True) -> int | None:
        return self._client.call("gripper.position", {"refresh_status": refresh_status})

    def position_mm(self, refresh_status: bool = True) -> float:
        return self._client.call("gripper.position_mm", {"refresh_status": refresh_status})

    def position_width(
        self,
        refresh_status: bool = True,
        max_width_m: float = DEFAULT_GRIPPER_MAX_WIDTH_M,
    ) -> float | None:
        pos = self.position(refresh_status=refresh_status)
        return None if pos is None else _position_to_width_m(pos, max_width_m)

    def status(self, refresh_status: bool = True) -> dict[str, Any]:
        return self._client.call("gripper.status", {"refresh_status": refresh_status})

    def object_detection(self, refresh_status: bool = True) -> int:
        return self._client.call("gripper.object_detection", {"refresh_status": refresh_status})


def connect(
    server_host: str,
    server_port: int = DEFAULT_PORT,
    **kwargs: Any,
) -> RobotiqGripperProxy:
    return RobotiqGripperProxy(server_host=server_host, server_port=server_port, **kwargs)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main(argv: Sequence[str] | None = None, prog: str | None = None) -> None:
    import argparse

    parser = argparse.ArgumentParser(prog=prog, description="Robotiq gripper ZMQ server/client")
    subparsers = parser.add_subparsers(dest="command", required=True)

    # --- serve ---
    sp = subparsers.add_parser("serve", help="Run the ZMQ gripper server (robot side)")
    sp.add_argument("--bind-host", default=DEFAULT_BIND_HOST)
    sp.add_argument("--port", type=int, default=DEFAULT_PORT)
    sp.add_argument("--no-pub", action="store_true", help="Disable state PUB socket")
    sp.add_argument("--com-port", default="auto")
    sp.add_argument("--device-id", type=int, default=9)
    sp.add_argument("--connection-type", default="RTU", choices=["RTU", "RTU_VIA_TCP"])
    sp.add_argument("--tcp-host", default="127.0.0.1")
    sp.add_argument("--tcp-port", type=int, default=54321)

    # --- client commands ---
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--host", required=True, help="ZMQ server host")
    common.add_argument("--port", type=int, default=DEFAULT_PORT)
    common.add_argument("--speed", type=int, default=255)
    common.add_argument("--force", type=int, default=255)

    subparsers.add_parser("activate", parents=[common])
    subparsers.add_parser("open", parents=[common])
    subparsers.add_parser("close", parents=[common])
    subparsers.add_parser("status", parents=[common])
    p = subparsers.add_parser("move-width", parents=[common])
    p.add_argument("--width-m", type=float, required=True)
    p = subparsers.add_parser("move-position", parents=[common])
    p.add_argument("--position", type=int, required=True)

    args = parser.parse_args(argv)

    if args.command == "serve":
        config = RobotiqConnectionConfig(
            com_port=args.com_port,
            device_id=args.device_id,
            connection_type=args.connection_type,
            tcp_host=args.tcp_host,
            tcp_port=args.tcp_port,
        )
        bind = f"tcp://{args.bind_host}:{args.port}"
        pub_bind = None if args.no_pub else f"tcp://{args.bind_host}:{args.port + PUB_PORT_OFFSET}"
        print(f"robotiq ZMQ RPC server on {bind}", flush=True)
        manager = GripperManager(config)
        server = ZmqGripperServer(bind=bind, pub_bind=pub_bind, manager=manager)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            print("robotiq server stopped", flush=True)
        return

    with RobotiqGripperProxy(server_host=args.host, server_port=args.port) as gripper:
        if args.command == "activate":
            gripper.activate()
        elif args.command == "open":
            gripper.open(speed=args.speed, force=args.force)
        elif args.command == "close":
            gripper.close(speed=args.speed, force=args.force)
        elif args.command == "status":
            print(gripper.status())
        elif args.command == "move-width":
            gripper.move_width(width_m=args.width_m, speed=args.speed, force=args.force)
        elif args.command == "move-position":
            gripper.move_position(position=args.position, speed=args.speed, force=args.force)


if __name__ == "__main__":
    main()
