from flightanalysis import State, Box
from pymavlink import mavutil
from geometry import GPS, Point
from .connection import Connection
from typing import Tuple
from . import mavlink


class Ovserver:
    def __init__(self, conn: Connection) -> None:
        self.conn = conn
                
    