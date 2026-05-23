from dataclasses import dataclass


@dataclass
class Config:
    IS_SETUP: bool = False
    IP: str | None = None
    PORT: int | None = None
    PUB_PORT: int | None = None
    TRACKER_PORT: int | None = None

cfg = Config()
_DEFAULT_PUB_PORT = object()


def setup_zero_franky(
    ip,
    port,
    pub_port: int | None | object = _DEFAULT_PUB_PORT,
) -> None:
    resolved_pub_port = port + 1 if pub_port is _DEFAULT_PUB_PORT else pub_port
    cfg.IS_SETUP = True
    cfg.IP = ip
    cfg.PORT = port
    cfg.PUB_PORT = resolved_pub_port
    cfg.TRACKER_PORT = None if resolved_pub_port is None else resolved_pub_port + 1
