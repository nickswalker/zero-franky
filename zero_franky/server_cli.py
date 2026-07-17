from __future__ import annotations

import argparse
import signal
from collections.abc import Sequence


DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 18812
STATE_PORT_OFFSET = 1
TRACKER_PORT_OFFSET = 2


def tcp_endpoint(host: str, port: int) -> str:
    return f"tcp://{host}:{port}"


def add_server_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--host", default=DEFAULT_HOST, help=f"Interface to bind RPC/PUB sockets on [{DEFAULT_HOST}]")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help=f"RPC port [{DEFAULT_PORT}]")
    parser.add_argument("--no-pub", action="store_true", help="Disable state PUB and tracker update sockets")


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the zero_franky ZeroMQ robot server")
    add_server_arguments(parser)
    return parser.parse_args(argv)


def build_server(args: argparse.Namespace):
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

    return ZmqRobotServer(bind=bind, pub_bind=pub_bind, tracker_bind=tracker_bind)


def run(args: argparse.Namespace) -> int:
    """Run a server built from `args` in the current (main) thread.

    Installs a SIGTERM handler on top of the normal KeyboardInterrupt (SIGINT)
    handling so process managers that send SIGTERM also get a clean shutdown:
    active tracker sessions/robots are stopped and sockets closed before exit.
    Only call this from the main thread; `signal.signal` requires it.
    """
    server = build_server(args)

    def handle_sigterm(_signum, _frame):
        raise KeyboardInterrupt

    previous_sigterm = signal.signal(signal.SIGTERM, handle_sigterm)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("zero_franky server stopped", flush=True)
    finally:
        signal.signal(signal.SIGTERM, previous_sigterm)
        server.shutdown()
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    return run(parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
