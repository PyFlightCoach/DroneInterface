from pymavlink import mavutil


from pymavlink.dialects.v10 import *

class Connection:
    def __init__(self, master: mavutil.mavfile, sysid: int) -> None:
        self.master = master
        self.sysid = sysid
        
        
    def request_msg(self,msg_id):
        self.master.mav.command_long_send(
            self.sysid,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            0,
            msg_id, 0, 0, 0, 0, 0, 0
        )
        
        
    def read_msg(self, msg_id):
        while True:
            self.request_msg(msg_id)
            message = self.master.recv_match(type = 'LOCAL_POSITION_NED',blocking=True)
            if message.get_srcSystem() == self.sysid:
                return message
    