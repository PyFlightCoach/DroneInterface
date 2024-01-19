from __future__ import annotations
from droneinterface.messages import mavlink
from geometry import Point, GPS, Quaternion
from pymavlink import mavutil
import numpy as np
from typing import List, Dict
from .wrapper_factory import wrapper_factory, wrappers


# TODO something like this might be better:
#        (
#            "home", 
#            lambda m: GPS(m.latitude/1e7, m.longitude/1e7, m.altitude/1e3), 
#            dict(
#                latitude = lambda g: g.lat * 1e7,
#                longitude = lambda g: g.lat * 1e7,
#                altitude  = lambda g: g.lat * 1e7,
#            )
#        )


HomePosition = wrapper_factory(
    "HomePosition",
    mavlink.MAVLINK_MSG_ID_HOME_POSITION,
    [
        ("home", GPS, ["latitude", "longitude", "altitude"], [1/1e7, 1/1e7, 1/1e3], [1e7, 1e7, 1e3]),
        ("position", Point, ["x", "y", "z"]),
        ("approach", Point, [f"approach_{d}" for d in list("xyz")]),
        ("q", Quaternion, ["q"])
    ]
)


GlobalOrigin = wrapper_factory(
    "GlobalOrigin",
    mavlink.MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN,
    [
        ("position", GPS, ["latitude", "longitude", "altitude"], [1/1e7, 1/1e7, 1/1e3], [1e7, 1e7, 1e3]),
    ]
)


Heartbeat = wrapper_factory(
    "Heartbeat",
    mavlink.MAVLINK_MSG_ID_HEARTBEAT,
    [],
    dict(
        mode = property(lambda self: mavutil.mode_mapping_bynumber(self.type)[self.custom_mode]),
        initialised = property(lambda self: not self.mode in [None, 'INITIALISING', 'MAV']),
        #mav_mode=property(lambda self : mavlink.enums["MAV_MODE"][self.custom_mode]),
        armed=property(lambda self: (self.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0)
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
        ("position", GPS, ["lat", "lon", "alt"], [1/1e7, 1/1e7, 1/1e3], [1e7, 1e7, 1e3]),
        ("agl", lambda v : v, ["relative_alt"], 1/1e3, 1e3),
        ("velocity", Point, ["vx", "vy", "vz"], 1/100, 100),
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
    mavlink.MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT,
    [
        ("position", GPS, ["lat_int", "lon_int", "alt"], [1/1e7, 1/1e7, 1], [1e7, 1e7, 1]),
        ("velocity", Point, ["vx", "vy", "vz"]),
        ("acceleration", Point, ["afx", "afy", "afz"])
    ]
)

PositionTargetLocal = wrapper_factory(
    "PositionTargetLocal",
    mavlink.MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED,
    [
        ("position", Point, ["x", "y", "z"]),
        ("velocity", Point, ["vx", "vy", "vz"]),
        ("acceleration", Point, ["afx", "afy", "afz"])
    ]
)


GPSRawInt = wrapper_factory(
    "GPSRawInt",
    mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,
    [
        ("position", GPS, ["lat", "lon", "alt"], [1/1e7, 1/1e7, 1], [1e7, 1e7, 1]),
    ],
    dict(
        gps_fix=property(lambda self : mavlink.enums["GPS_FIX_TYPE"][self.fix_type])
    )
)

EKFStatus = wrapper_factory(
    "EKFStatus",
    mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT,
    [],
    dict(
        is_good=property(lambda self: (self.flags & mavlink.EKF_PRED_POS_HORIZ_ABS) > 0)
    )
)


BatteryStatus = wrapper_factory(
    "BatteryStatus",
    mavlink.MAVLINK_MSG_ID_BATTERY_STATUS,
    [
        ("voltage", lambda v: v, ["voltages"], lambda v: v[0] / 1000,  lambda v : [v * 1000] + [0 for _ in range(9)]),
        ("current", lambda v: v, ["current_battery"], 1/100, 100),
    ],
    dict(
        watts = property(lambda self: self.voltage * self.current)
    )
)





_ignore = list(np.zeros((18), dtype=int))
_release = list(np.full(9, 2**16)) + list(np.full(9, 2**16-1))
def set_channels(targ_sys, targ_comp, channels: Dict[int, int], others:str="ignore") -> RCOverride:
    oth = _release if others == "release" else _ignore

    return RCOverride(0, targ_sys, targ_comp, \
        *[(channels[i] if i in channels else oth[i]) for i in range(18) ]
    )


RCOverride = wrapper_factory(
    "RCOverride",
    mavlink.MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE,
    [],
    dict(
        set_channels = staticmethod(set_channels), 
        set_channel = staticmethod(lambda ts, tc, ch, val, others="ignore" : set_channels(ts, tc, {ch: val}, others)),
        release_channels = staticmethod(lambda ts, tc, channels: set_channels(ts, tc, {ch: _release[ch] for ch in channels})),
    )
)


#wrap all the remaining messages in wrappers that do nothing:
for msgid, msgcls in mavlink.mavlink_map.items():
    if not msgid in wrappers:
        wrapper_factory(
            ''.join(word.title() for word in msgcls.__name__[7:-7].split("_")),
            msgid, []
        )

