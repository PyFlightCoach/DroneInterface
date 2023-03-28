from . import mavlink
from geometry import GPS

command_map = dict()


def command(fun):
    command_map[fun.__name__] = fun


@command
def arm():
    return (mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)

@command
def disarm():
    return (mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0)
        
@command
def set_airspeed(speed: float):
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
    return (mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, id, rate)
