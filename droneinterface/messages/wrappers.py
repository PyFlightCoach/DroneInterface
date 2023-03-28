from __future__ import annotations
from droneinterface.messages import mavlink
from geometry import Point, GPS, Quaternion
from pymavlink import mavutil
import numpy as np
from typing import List
from .wrapper_factory import wrapper_factory, wrappers



HomePosition = wrapper_factory(
    "HomePosition",
    mavlink.MAVLINK_MSG_ID_HOME_POSITION,
    mavlink.MAVLink_home_position_message,
    [
        ("home", GPS, "latitude", "longitude"),
        ("alt", float, "altitude"),
        ("position", Point, "x", "y", "z"),
        ("approach", Point, *["approach_{d}" for d in list("xyz")]),
        ("q", Quaternion, ["q"])
    ]
    
)



Heartbeat = wrapper_factory(
    "Heartbeat",
    mavlink.MAVLINK_MSG_ID_HEARTBEAT,
    mavlink.MAVLink_heartbeat_message,
    [
        ("autopilot", int, "autopilot"),
        ("mode", int, "autopilot"),
    ]
)

class Heartbeat(MessageWrapper):
    id = mavlink.MAVLINK_MSG_ID_HEARTBEAT
    def __init__(self, timestamp, autopilot, mode, system_status ) -> None:
        super().__init__(timestamp)
        self.autopilot  = autopilot
        self.mode = mode
        self.system_status = system_status
        self.initialised = not self.mode in [None, 'INITIALISING', 'MAV']
        
    @staticmethod
    def parse(msg: mavlink.MAVLink_heartbeat_message) -> Heartbeat:
        return Heartbeat(
            msg._timestamp,
            msg.autopilot,
            mavutil.mode_mapping_bynumber(msg.type)[msg.custom_mode],
            msg.system_status
        )
        