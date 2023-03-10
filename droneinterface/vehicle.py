"""This is an opinionated interpretation of a vehicles state"""
from .messages import wrappers, mdefs
from .connection import Connection
from pymavlink import mavutil
from flightanalysis import Box


class Vehicle(Connection):
    def __init__(self, master: mavutil.mavfile, sysid: int, box: Box) -> None:
        super().__init__(master, sysid)
        self.box = box
        
    @staticmethod
    def connect(constr: str, sysid: int, box: Box):
        return Vehicle(
            mavutil.mavlink_connection(constr), 
            sysid,
            box
        ).wait_for_boot()
    
    