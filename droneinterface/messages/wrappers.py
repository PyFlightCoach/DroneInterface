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
    [
        ("home", GPS, ["latitude", "longitude"], lambda v : v/1e7, lambda v : v*1e7),
        ("position", Point, ["x", "y", "z"]),
        ("approach", Point, [f"approach_{d}" for d in list("xyz")]),
        ("q", Quaternion, ["q"])
    ]
)


Heartbeat = wrapper_factory(
    "Heartbeat",
    mavlink.MAVLINK_MSG_ID_HEARTBEAT,
    [],
    dict(
        mode = property(lambda self: mavutil.mode_mapping_bynumber(self.type)[self.custom_mode]),
        initialised = property(lambda self: not self.mode in [None, 'INITIALISING', 'MAV'])
    )
)

AttitudeQuaternion = wrapper_factory(
    "AttitudeQuaternion",
    mavlink.MAVLINK_MSG_ID_ATTITUDE_QUATERNION,
    [
        ("att", Quaternion, ["q1", "q2", "q3", "q4"]),
        ("rvel", Point, ["rollspeed", "pitchspeed", "yawspeed"]),
        ("repr_offset", Quaternion, ["repr_offset_q"])
    ]
)

LocalPositionNED = wrapper_factory(
    "LocalPositionNED",
    mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
    [
        ("position", Point, ["x", "y", "z"]),
        ("velocity", Point, ["vx", "vy", "vz"])
    ]
)

LocalPositionNEDCov = wrapper_factory(
    "LocalPositionNEDCov",
    mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED_COV,
    [
        ("position", Point, ["x", "y", "z"]),
        ("velocity", Point, ["vx", "vy", "vz"]),
        ("acceleration", Point, ["ax", "ay", "az"])
    ],
)

GlobalPositionInt = wrapper_factory(
    "GlobalPositionInt",
    mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
    [
        ("position", GPS, ["lat", "lon"], lambda v : v/1e7, lambda v : v*1e7),
        ("alt", lambda v : v, ["alt"], lambda v : v/1e3, lambda v : v*1e3),
        ("agl", lambda v : v, ["relative_alt"], lambda v : v/1e3, lambda v : v*1e3),
        ("velocity", Point, ["vx", "vy", "vz"], lambda v : v/100, lambda v : v*100),
        ("heading", lambda v : v, ["hdg"], lambda v : np.radians(v/10), lambda v : np.degrees(v)*10),
    ]
)
        

HighResIMU = wrapper_factory(
    "HighResIMU",
    mavlink.MAVLINK_MSG_ID_HIGHRES_IMU,
    [
        ("acc", Point, ["xacc", "yacc", "zacc"]),
        ("gyro", Point, ["xgyro", "ygyro", "zgyro"]),
        ("mag", Point, ["xmag", "ymag", "zmag"]),
    ]
)

ScaledIMU = wrapper_factory(
    "ScaledIMU",
    mavlink.MAVLINK_MSG_ID_SCALED_IMU,
    [
        ("acc", Point, ["xacc", "yacc", "zacc"]),
        ("gyro", Point, ["xgyro", "ygyro", "zgyro"]),
        ("mag", Point, ["xmag", "ymag", "zmag"])
    ]
)

PositionTargetGlobal = wrapper_factory(
    "PositionTargetGlobal",
    mavlink.MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT,
    [
        ("position", GPS, ["lat", "lon"], lambda v : v/1e7, lambda v : v*1e7),
        ("alt", lambda v : v, ["alt"], lambda v : v/1e3, lambda v : v*1e3),
        ("velocity", Point, ["vx", "vy", "vz"]),
        ("acceleration", Point, ["ax", "ay", "az"])
    ]
)

PositionTargetLocal = wrapper_factory(
    "PositionTargetLocal",
    mavlink.MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED,
    [
        ("position", Point, ["x", "y", "z"]),
        ("velocity", Point, ["vx", "vy", "vz"]),
        ("acceleration", Point, ["ax", "ay", "az"])
    ]
)

