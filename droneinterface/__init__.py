from loguru import logger
import sys
logger.disable("droneinterface")


def enable_logging(level='INFO'):
    logger.enable("droneinterface")
    logger.remove(0)
    logger.add(
    sys.stdout, colorize=True, level=level,
        format="<green>{name}, {module}, {thread}, {time:HH:mm:ss}</green> <level>{message}</level>"
    )

def configure_logging(*args, **kwargs):
    logger.enable("droneinterface")
    logger.remove(0)
    logger.add(*args, **kwargs)

from .messages import mavlink
from pathlib import Path
import shutil
from .connection import Connection, LastMessage



class Base:
    def _msg(self, msg):
        return f"{str(self).ljust(80)[:80]}: {msg}"

from .vehicle import Vehicle

def empty_tmpdir():
    shutil.rmtree(Path("logtmp"))
    Path("logtmp").mkdir()
