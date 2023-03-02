from pymavlink import mavutil
from .messages import MesDef, MesDefs, Message, Messages
from typing import Union


class Connection:
    def __init__(self, master: mavutil.mavfile, sysid: int) -> None:
        self.master = master
        self.sysid = sysid
        
        
    def request_msg(self, message: Union[MesDefs, MesDef]):
        message = MesDefs([message]) if isinstance(message, MesDef) else message
            
        for m in message:
            self.master.mav.command_long_send(
                self.sysid,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                0,
                m.id, 0, 0, 0, 0, 0, 0
            )
    def send_message(message: Union[Message, Messages]):
        pass
        
    def receive_msg(self, message: Union[MesDefs, MesDef]):
        
        while True:
            response = self.master.recv_match(type = message.msgname, blocking=True)
            if response.get_srcSystem() == self.sysid:
                return response
        
    