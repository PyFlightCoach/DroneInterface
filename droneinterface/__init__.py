from loguru import logger
import sys
logger.disable("droneinterface")


def enable_logging(level='INFO'):
    logger.enable("droneinterface")
    logger.remove(0)
    logger.add(sys.stdout, colorize=True, level=level, format="<level>{message}</level>")

def configure_logging(*args, **kwargs):
    logger.enable("droneinterface")
    logger.remove(0)
    logger.add(*args, **kwargs)

from .messages import mavlink, wrappermap
from pathlib import Path
import shutil
from .scheduling import Timeout, TooOld, NeverReceived, AwaitCondition, Watcher
from .connection import Connection, LastMessage
from .vehicle import Vehicle

def empty_tmpdir():
    shutil.rmtree(Path("logtmp"))
    Path("logtmp").mkdir()
