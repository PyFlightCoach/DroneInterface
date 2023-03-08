from flightanalysis import State
from pymavlink import mavutil
from geometry import GPS
from .connection import Connection


class Vehicle:
    def __init__(self, conn: Connection) -> None:
        self.conn = conn
    
    def imu_home(self) -> GPS:
        pass
        

