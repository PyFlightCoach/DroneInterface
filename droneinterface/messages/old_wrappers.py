"""These classes wrap mavlink messages and convert them into useful entities, 
for example seperate x,y,z properties go into pfc-geometry Point objects and modes are interpreted."""
from __future__ import annotations
from droneinterface.messages import mavlink
from geometry import Point, GPS, Quaternion
from pymavlink import mavutil
import numpy as np
from typing import List

wrappers = {}

   
class MessageWrapper():
    id = None
    def __init__(self, timestamp) -> None:
        self.time = timestamp
   
    def __init_subclass__(cls):
        wrappers[cls.id] = cls    

    def __str__(self) -> str:
        return f"id:{self.id}, name:{self.__class__.__name__}, time:{self.time}"



class HomePosition(MessageWrapper):
    id = mavlink.MAVLINK_MSG_ID_HOME_POSITION
    def __init__(self, timestamp, home: GPS, alt: float, position: Point, approach: Point, q: Quaternion) -> None:
        super().__init__(timestamp)
        self.home = home
        self.alt = alt
        self.position = position
        self.approach = approach
        self.q = q
        
    @staticmethod
    def parse(msg: mavlink.MAVLink_home_position_message) -> HomePosition:
        assert HomePosition.id == msg.id
        return HomePosition(
            msg._timestamp,
            GPS(msg.latitude / 1e7, msg.longitude / 1e7),
            msg.altitude / 1000,
            Point(msg.x, msg.y, msg.z),
            Point(msg.approach_x, msg.approach_y, msg.approach_z),
            Quaternion(*msg.q)
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
        
            
class AttitudeQuaternion(MessageWrapper):
    id = mavlink.MAVLINK_MSG_ID_ATTITUDE_QUATERNION
    def __init__(self, timestamp, att, rvel, repr_offset) -> None:
        super().__init__(timestamp)
        self.att = att
        self.rvel = rvel
        self.repr_offset = repr_offset
    
    @staticmethod
    def parse(msg: mavlink.MAVLink_attitude_quaternion_message) -> AttitudeQuaternion:
        return AttitudeQuaternion(
            msg._timestamp,
            Quaternion(msg.q1, msg.q2, msg.q3, msg.q4),
            Point(msg.rollspeed, msg.pitchspeed, msg.yawspeed),
            Quaternion(*msg.repr_offset_q)    
        )
        
        
        
class LocalPositionNED(MessageWrapper):
    id = mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED
    def __init__(self, timestamp, position, velocity) -> None:
        super().__init__(timestamp)
        self.position: Point = position
        self.velocity: Point = velocity
        
    @staticmethod
    def parse(msg: mavlink.MAVLink_local_position_ned_message) -> LocalPositionNED:
        return LocalPositionNED(
            msg._timestamp,
            Point(msg.x, msg.y, msg.z),
            Point(msg.vx, msg.vy, msg.vz)
        )

class LocalPositionNEDCov(MessageWrapper):
    id = mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED_COV
    def __init__(self, timestamp, position, velocity, acceleration, covariance) -> None:
        super().__init__(timestamp)
        self.position: Point = position
        self.velocity: Point = velocity
        self.acceleration: Point = acceleration
        self.covariance: float = covariance
        
    @staticmethod
    def parse(msg: mavlink.MAVLink_local_position_ned_cov_message) -> LocalPositionNEDCov:
        return LocalPositionNEDCov(
            msg._timestamp,
            Point(msg.x, msg.y, msg.z),
            Point(msg.vx, msg.vy, msg.vz),
            Point(msg.ax, msg.ay, msg.az),
            msg.cov
        )

class GlobalPositionInt(MessageWrapper):
    id = mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT
    def __init__(self, timestamp, position, alt, agl, velocity, heading):
        super().__init__(timestamp)
        self.position: Point = position
        self.alt: float = alt
        self.agl: float = agl
        self.velocity: Point = velocity
        self.heading: float = heading
        
    @staticmethod
    def parse(msg: mavlink.MAVLink_global_position_int_message) -> GlobalPositionInt:
        return GlobalPositionInt(
            msg._timestamp,
            GPS(msg.lat / 1e7, msg.lon / 1e7),
            msg.alt / 1000,
            msg.relative_alt / 1000,
            Point(msg.vx, msg.vy, msg.vz)/100,
            np.radians(msg.hdg / 10),
        )
        
class HighResIMU(MessageWrapper):
    id = mavlink.MAVLINK_MSG_ID_HIGHRES_IMU
    def __init__(self, timestamp, acc, gyro, mag, abs_pressure, diff_pressure, press_alt, temperature, imu_id) -> None:
        super().__init__(timestamp)
        self.acc: Point = acc
        self.gyro: Point = gyro
        self.mag: Point = mag
        self.abs_pressure: float = abs_pressure
        self.diff_pressure: float = diff_pressure
        self.press_alt: float = press_alt
        self.temperature: float = temperature
        self.imu_id: int = imu_id
        
    @staticmethod
    def parse(msg: mavlink.MAVLink_highres_imu_message) -> HighResIMU:
        return HighResIMU(
            msg._timestamp,
            Point(*attr_search(suf="acc")(msg)),
            Point(*attr_search(suf="gyro")(msg)),
            Point(*attr_search(suf="mag")(msg)),
            msg.abs_pressure,
            msg.diff_pressure,
            msg.pressure_alt,
            msg.temperature,
            msg.id
        )

        
class ScaledIMU(MessageWrapper):
    id = mavlink.MAVLINK_MSG_ID_SCALED_IMU
    def __init__(self, timestamp, acc, gyro, mag, temperature) -> None:
        super().__init__(timestamp)
        self.acc: Point = acc
        self.gyro: Point = gyro
        self.mag: Point = mag
        self.temperature: float = temperature
    
    @staticmethod
    def parse(msg: mavlink.MAVLink_scaled_imu_message) -> ScaledIMU:
        return ScaledIMU(
            msg._timestamp,
            Point(*attr_search(suf="acc")(msg)) / 981,
            Point(*attr_search(suf="gyro")(msg)) / 1000,
            Point(*attr_search(suf="mag")(msg)) / 1000,
            msg.temperature    
        )
        
        
    
class PositionTargetGlobal(MessageWrapper):
    id = mavlink.MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT
    def __init__(self, timestamp, coordinate_frame, type_mask, position, alt, velocity, acceleration, yaw, yaw_rate) -> None:
        super().__init__(timestamp)
        self.coordinate_frame = coordinate_frame
        self.type_mask = type_mask
        self.position = position
        self.alt = alt
        self.velocity = velocity
        self.acceleration = acceleration
        self.yaw = yaw
        self.yaw_rate = yaw_rate
    
    @staticmethod
    def parse(msg: mavlink.MAVLink_position_target_global_int_message) -> PositionTargetGlobal:
        return PositionTargetGlobal(
            msg._timestamp,
            msg.coordinate_frame,
            msg.type_mask,
            GPS(msg.lat_int / 1e7, msg.lon_int / 1e7),
            msg.alt,
            Point(*attr_search(pre="v")(msg)),
            Point(*attr_search(pre="af")(msg)),
            msg.yaw,
            msg.yaw_rate
        )
        
class SetPositionTargetGlobal(PositionTargetGlobal):
    id = mavlink.MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT
    def __init__(self, timestamp, target_system, target_component, *args, **kwargs) -> None:
        super().__init__(timestamp, *args, **kwargs)
        self.target_system = target_system
        self.target_component = target_component
    

    @staticmethod
    def parse(msg: mavlink.MAVLink_set_position_target_global_int_message) -> SetPositionTargetGlobal:
        return SetPositionTargetGlobal(
            target_system=msg.target_system,
            target_component=msg.target_component,
            **PositionTargetGlobal.parse(msg).__dict__
        )
        
        
class PositionTargetLocal(MessageWrapper):
    id = mavlink.MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED
    def __init__(self, timestamp, position, velocity, acceleration, yaw, yaw_rate) -> None:
        super().__init__(timestamp)
        self.position: Point = position
        self.velocity: Point = velocity
        self.acceleration: Point = acceleration
        self.yaw: float = yaw
        self.yaw_rate: float = yaw_rate
    
    @staticmethod
    def parse(msg: mavlink.MAVLink_position_target_local_ned_message) -> PositionTargetLocal:
        return PositionTargetLocal(
            msg._timestamp,
            Point(*attr_search()(msg)),
            Point(*attr_search(pre="v")(msg)),
            Point(*attr_search(pre="af")(msg)),
            msg.yaw,
            msg.yaw_rate
        )
        
        
class SetPositionTargetLocal(PositionTargetGlobal):
    id = mavlink.MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED
    def __init__(self, timestamp, target_system, target_component, *args, **kwargs) -> None:
        super().__init__(timestamp, *args, **kwargs)
        self.target_system = target_system
        self.target_component = target_component
    

    @staticmethod
    def parse(msg: mavlink.MAVLink_set_position_target_local_ned_message) -> SetPositionTargetLocal:
        return SetPositionTargetLocal(
            target_system=msg.target_system,
            target_component=msg.target_component,
            **PositionTargetLocal.parse(msg).__dict__
        )
        