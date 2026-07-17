"""zero-franky command line interface.

    zero-franky server [options]     run the robot RPC server (add --robotiq to
                                      also run the Robotiq gripper server)
    zero-franky gripper <subcommand> run or query the Robotiq gripper server
                                      standalone (serve/activate/open/close/
                                      status/move-width/move-position)

Run `zero-franky <command> --help` for command-specific options.
"""

from __future__ import annotations

import argparse
import signal
import subprocess
import sys
import threading
import time
from collections.abc import Sequence

from zero_franky import server_cli

USAGE = __doc__.split("\n\n", 1)[1] if __doc__ else ""

GRIPPER_DEFAULT_PORT = 18815


def _build_server_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="zero-franky server",
        description="Run the zero_franky robot RPC server",
    )
    server_cli.add_server_arguments(parser)
    parser.add_argument("--robotiq", action="store_true", help="Also run the Robotiq gripper server")
    robotiq_args = parser.add_argument_group("--robotiq options")
    robotiq_args.add_argument("--gripper-port", type=int, default=GRIPPER_DEFAULT_PORT)
    robotiq_args.add_argument("--gripper-publish-hz", type=float, default=30.0)
    robotiq_args.add_argument("--com-port", default="auto")
    robotiq_args.add_argument("--device-id", type=int, default=9)
    robotiq_args.add_argument("--connection-type", default="RTU", choices=["RTU", "RTU_VIA_TCP"])
    robotiq_args.add_argument("--tcp-host", default="127.0.0.1")
    robotiq_args.add_argument("--tcp-port", type=int, default=54321)
    return parser


def gripper_command(args: argparse.Namespace) -> list[str]:
    command = [
        sys.executable, "-m", "zero_franky.robotiq", "serve",
        "--bind-host", args.host,
        "--port", str(args.gripper_port),
        "--com-port", args.com_port,
        "--device-id", str(args.device_id),
        "--connection-type", args.connection_type,
        "--tcp-host", args.tcp_host,
        "--tcp-port", str(args.tcp_port),
        "--publish-hz", str(args.gripper_publish_hz),
    ]
    if args.no_pub:
        command.append("--no-pub")
    return command


def _terminate(process: subprocess.Popen) -> None:
    if process.poll() is None:
        process.terminate()
    try:
        process.wait(timeout=3.0)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait()


def run_server(args: argparse.Namespace) -> int:
    if not args.robotiq:
        return server_cli.run(args)

    server = server_cli.build_server(args)
    gripper = subprocess.Popen(gripper_command(args))

    def stop_on_signal(_signum, _frame):
        raise KeyboardInterrupt

    previous_term = signal.signal(signal.SIGTERM, stop_on_signal)
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    try:
        while server_thread.is_alive():
            return_code = gripper.poll()
            if return_code is not None:
                print(f"robotiq gripper server exited with code {return_code}", flush=True)
                return return_code if return_code != 0 else 1
            time.sleep(0.2)
        return 0
    except KeyboardInterrupt:
        return 0
    finally:
        signal.signal(signal.SIGTERM, previous_term)
        server.shutdown()
        server_thread.join(timeout=3.0)
        _terminate(gripper)


def main(argv: Sequence[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv or argv[0] in ("-h", "--help"):
        print(USAGE, file=sys.stderr if not argv else sys.stdout)
        return 2 if not argv else 0

    command, rest = argv[0], argv[1:]
    if command == "server":
        args = _build_server_parser().parse_args(rest)
        return run_server(args)
    if command == "gripper":
        from zero_franky.robotiq import main as gripper_main

        return gripper_main(rest, prog="zero-franky gripper") or 0

    print(f"zero-franky: unknown command {command!r}\n", file=sys.stderr)
    print(USAGE, file=sys.stderr)
    return 2


if __name__ == "__main__":
    raise SystemExit(main())