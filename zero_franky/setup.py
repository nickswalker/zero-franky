from __future__ import annotations

from dataclasses import dataclass


@dataclass
class Config:
    IS_SETUP: bool = False
    IP: str | None = None
    PORT: int | None = None
    PUB_PORT: int | None = None
    TRACKER_PORT: int | None = None

cfg = Config()


def setup_zero_franky(
    ip,
    port,
) -> None:
    """Configure the default server connection used by :class:`zero_franky.Robot`.

    ``port`` is the RPC port; state and tracker transports use ``port + 1`` and
    ``port + 2``.
    """
    cfg.IS_SETUP = True
    cfg.IP = ip
    cfg.PORT = port
    cfg.PUB_PORT = port + 1
    cfg.TRACKER_PORT = port + 2
