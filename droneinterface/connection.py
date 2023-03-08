from pymavlink import mavutil
from . import mavlink
from typing import Union


class Connection:
    def __init__(self, master: mavutil.mavfile, sysid: int) -> None:
        self.master = master
        self.sysid = sysid
        
    def request_msg(self, id):
        self.master.mav.send(mavlink.MAVLink_command_long_message(
                self.sysid,
                self.master.target_component,
                mavlink.MAV_CMD_REQUEST_MESSAGE,
                0, id, 0, 0, 0, 0, 0, 0    
        ))
        
    def receive_msg(self, id):
        while True:
            response = self.master.recv_match(
                type = mavlink.mavlink_map[id], 
                blocking=True
            )
            if response.get_srcSystem() == self.sysid:
                return response
        