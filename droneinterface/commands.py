from . import mavlink
from geometry import GPS

command_map = {
    mavlink.MAVLINK_MSG_ID_COMMAND_LONG: dict(),
    mavlink.MAVLINK_MSG_ID_COMMAND_INT: dict(),
    mavlink.MAVLINK_MSG_ID_COMMAND_ACK: dict(),
    mavlink.MAVLINK_MSG_ID_MISSION_ITEM_INT: dict(),
}

def pad_zeros(arr, length):
    return [arr[i] if i < len(arr) else 0 for i in range(length)]

def _command(fun, msgid, length):
    def outer(*args, **kwargs):
        return pad_zeros(fun(*args, **kwargs), length)
    command_map[msgid][fun.__name__] = outer
    return outer

def longcommand(fun):
    return _command(fun, mavlink.MAVLINK_MSG_ID_COMMAND_LONG, 9)

def navcommand(fun):
    return _command(fun, mavlink.MAVLINK_MSG_ID_MISSION_ITEM_INT, 13)

def intcommand(fun):
    return _command(fun, mavlink.MAVLINK_MSG_ID_COMMAND_INT, 9)

@longcommand
def arm():
    return (mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21196)

@longcommand
def disarm():
    return (mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 21196)
        
@longcommand
def set_airspeed(speed: float):
    return (mavlink.MAV_CMD_DO_CHANGE_SPEED, 0, 0, speed, -2)

@longcommand
def set_groundspeed(speed: float):
    return (mavlink.MAV_CMD_DO_CHANGE_SPEED, 0, 1, speed, -2)

@longcommand
def set_home(pos: GPS, yaw: float):
    return (mavlink.MAV_CMD_DO_SET_HOME, 0, 0, 0,  yaw, pos.lat[0], pos.long[0], pos.alt[0])

@longcommand
def set_servo(id: int, pwm: int):
    return (mavlink.MAV_CMD_DO_SET_SERVO, 0, id, pwm)

@longcommand
def request_message(id: int):
    return (mavlink.MAV_CMD_REQUEST_MESSAGE, 0, id)

@longcommand
def set_message_rate(id: int, rate: float):
    return (mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, id, (int(1e7) / rate) if rate > 0 else -1)

@longcommand
def set_mode(mode: int):
    return (mavlink.MAV_CMD_DO_SET_MODE, 0, mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode)

@navcommand
def nav_waypoint(seq: int, pos: GPS, frame: int=5, current: int = 2, autocontinue=False, hold=0, accept_radius=20, pass_radius=0, yaw=0):
    return (seq, frame, mavlink.MAV_CMD_NAV_WAYPOINT, current, autocontinue, hold, accept_radius, pass_radius, yaw, int(pos.lat[0] * 1e7), int(pos.long[0] * 1e7), pos.alt[0])

@intcommand
def do_reposition(pos: GPS, speed=-1, radius=0, yaw=0):
    return (speed, 1, radius, yaw, int(pos.lat[0] * 1e7), int(pos.long[0] * 1e7), pos.alt[0])

all_commands = {k: v for m in command_map.values() for k, v in m.items()}

