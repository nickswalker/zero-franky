from __future__ import annotations

import argparse
from collections.abc import Sequence


DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 18812
STATE_PORT_OFFSET = 1
TRACKER_PORT_OFFSET = 2


def tcp_endpoint(host: str, port: int) -> str:
    return f"tcp://{host}:{port}"


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the zero_franky ZeroMQ robot server")
    parser.add_argument("--host", default=DEFAULT_HOST, help=f"Interface to bind RPC/PUB sockets on [{DEFAULT_HOST}]")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help=f"RPC port [{DEFAULT_PORT}]")
    parser.add_argument("--no-pub", action="store_true", help="Disable state PUB and tracker update sockets")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    bind = tcp_endpoint(args.host, args.port)
    pub_bind = None if args.no_pub else tcp_endpoint(args.host, args.port + STATE_PORT_OFFSET)
    tracker_bind = None if args.no_pub else tcp_endpoint(args.host, args.port + TRACKER_PORT_OFFSET)

    from zero_franky.zmq_server import ZmqRobotServer

    print(f"zero_franky RPC server on {bind}", flush=True)
    if pub_bind is None:
        print("zero_franky state pub/tracker disabled", flush=True)
    else:
        print(f"zero_franky state pub on {pub_bind}", flush=True)
        print(f"zero_franky tracker updates on {tracker_bind}", flush=True)

    server = ZmqRobotServer(bind=bind, pub_bind=pub_bind, tracker_bind=tracker_bind)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("zero_franky server stopped", flush=True)
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
