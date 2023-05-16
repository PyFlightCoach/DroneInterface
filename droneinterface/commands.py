from . import mavlink
from geometry import GPS

command_map = dict()


def command(fun):
    command_map[fun.__name__] = fun


@command
def arm():
    return (mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1, 21196)

@command
def disarm():
    return (mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 21196)
        
@command
def  set_airspeed(speed: float):
    return (mavlink.MAV_CMD_DO_CHANGE_SPEED, 0, speed, -2)

@command
def set_groundspeed(speed: float):
    return (mavlink.MAV_CMD_DO_CHANGE_SPEED, 1, speed, -2)

@command
def set_home(pos: GPS, alt: float, yaw: float):
    return (mavlink.MAV_CMD_DO_SET_HOME, 0, 0,  yaw, pos.lat, pos.long, alt)

@command
def set_servo(id: int, pwm: int):
    return (mavlink.MAV_CMD_DO_SET_SERVO, id, pwm)

@command
def request_message(id: int):
    return (mavlink.MAV_CMD_REQUEST_MESSAGE, id)

@command
def set_message_rate(id: int, rate: float):
    return (mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, id, (int(1e6) / rate) if rate > 0 else -1)

@command
def set_mode(mode: int):
    return (mavlink.MAV_CMD_DO_SET_MODE, mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode)

@command
def set_rc_channel(instance, pwm: int):
    return (mavlink.RC_CHANNELS_OVERRIDE, instance, pwm)

