"""This is an opinionated interpretation of a vehicles state"""
from .messages import wrappers, mdefs, MessageWrapper
from .connection import Connection
from .combinators import append_combinators
from pymavlink import mavutil
from flightanalysis import Box, State, FlightLine


class Vehicle(Connection):
    def __init__(self, master: mavutil.mavfile, sysid: int, box: Box=None) -> None:
        super().__init__(master, sysid)
        home = self.homeposition.home
        self.flightline = FlightLine.from_box(
            Box("world", home, 0) if box is None else box, 
            home
        )
        
        append_combinators(self)
        
        
    @staticmethod
    def connect(constr: str, sysid: int, box: Box=None):
        
        return Vehicle(
            mavutil.mavlink_connection(constr), 
            sysid,
            box
        ).wait_for_boot()
    
    