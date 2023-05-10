from .messages import mavlink
import logging
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
