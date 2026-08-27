from .setup import setup_zero_franky
from .types import CRITICAL
from .zmq_client import RobotProxy as Robot

__all__ = ["CRITICAL", "Robot", "setup_zero_franky"]
