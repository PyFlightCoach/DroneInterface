"""These classes wrap mavlink messages and convert them into useful entities, 
for example seperate x,y,z properties go into pfc-geometry Point objects and modes are interpreted."""

from droneinterface.messages import mavlink
from geometry import Point, GPS, Quaternion
from pymavlink import mavutil
import numpy as np

wrappers = {}
   
class MessageWrapper():
    id = None
    def __init__(self, msg: mavlink.MAVLink_message) -> None:
        assert msg.id == self.__class__.id
        self.time = msg._timestamp

    def __init_subclass__(cls):
        # could just build it later with __sublcasses__ or something but this seems neat
        wrappers[cls.id] = cls    

    def __str__(self):
        return f"id:{self.id}, name:{self.__class__.__name__}, time:{self.time}"


class HomePosition(MessageWrapper):
    id = mavlink.MAVLINK_MSG_ID_HOME_POSITION
    def __init__(self, msg: mavlink.MAVLink_home_position_message) -> None:
        super().__init__(msg)
        self.home = GPS(msg.latitude / 1e7, msg.longitude / 1e7)
        self.alt = msg.altitude / 1000
        self.position = Point(msg.x, msg.y, msg.z)
        self.approach = Point(msg.approach_x, msg.approach_y, msg.approach_z)
        self.q = Quaternion(*msg.q)


class Heartbeat(MessageWrapper):
    id = mavlink.MAVLINK_MSG_ID_HEARTBEAT
    def __init__(self, msg: mavlink.MAVLink_heartbeat_message) -> None:
        super().__init__(msg)
        self.autopilot = msg.autopilot
        self.mode = mavutil.mode_mapping_bynumber(msg.type)[msg.custom_mode]
        self.system_status = msg.system_status
        self.initialised = self.system_status >= 3
        # The above seems to work but dronekit uses this for some reason:
        # self.initialised = not self.mode in [None, 'INITIALISING', 'MAV']
        
            
class AttitudeQuaternion(MessageWrapper):
    id = mavlink.MAVLINK_MSG_ID_ATTITUDE_QUATERNION
    def __init__(self, msg: mavlink.MAVLink_attitude_quaternion_message) -> None:
        super().__init__(msg)
        self.att = Quaternion(msg.q1, msg.q2, msg.q3, msg.q4)
        self.rvel = Point(msg.rollspeed, msg.pitchspeed, msg.yawspeed)
        self.repr_offset = Quaternion(*msg.repr_offset_q)
        
        
class LocalPositionNED(MessageWrapper):
    id = mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED
    def __init__(self, msg: mavlink.MAVLink_local_position_ned_message) -> None:
        super().__init__(msg)
        self.position = Point(msg.x, msg.y, msg.z)
        self.velocity = Point(msg.vx, msg.vy, msg.vz)

class LocalPositionNEDCov(MessageWrapper):
    id = mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED_COV
    def __init__(self, msg: mavlink.MAVLink_local_position_ned_cov_message) -> None:
        super().__init__(msg)
        self.position = Point(msg.x, msg.y, msg.z)
        self.velocity = Point(msg.vx, msg.vy, msg.vz)
        self.acceleration = Point(msg.ax, msg.ay, msg.az)
        self.covariance = msg.cov


class GlobalPositionInt(MessageWrapper):
    id = mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT
    def __init__(self, msg: mavlink.MAVLink_global_position_int_message) -> None:
        super().__init__(msg)
        self.position = GPS(msg.lat / 1e7, msg.lon / 1e7)
        self.alt = msg.alt / 1000
        self.agl = msg.relative_alt / 1000
        self.velocity = Point(msg.vx, msg.vy, msg.vz)/100
        self.heading = np.radians(msg.hdg / 10)

