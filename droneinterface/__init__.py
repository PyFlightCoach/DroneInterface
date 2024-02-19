import logging
logger = logging.getLogger('droneinterface')

pmllogger = logging.getLogger('pymavlink')
pmllogger.setLevel(logging.CRITICAL)

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
