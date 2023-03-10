"""This wraps the mavlink connection so you can request single messages or subscribe to streams of them
If wrappers do not exist for the requested message then the original message object is returned"""

from pymavlink import mavutil
from . import mavlink
from typing import Union, List
from time import time, sleep
from .messages import wrappers, mdefs
from threading import Thread
from copy import deepcopy



class Connection:
    def __init__(self, master: mavutil.mavfile, sysid: int) -> None:
        self.master = master
        self.sysid = sysid
    
    @staticmethod
    def connect(constr: str, sysid: int):
        return Connection(
            mavutil.mavlink_connection(constr), 
            sysid
        ).wait_for_boot()
        
    def wait_for_boot(self):
        #print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))    
        while not self.receive_msg(mavlink.MAVLINK_MSG_ID_HEARTBEAT).initialised:
            pass
        return self

    def request_msg(self, id: int, rate:int=0):
        self.master.mav.send(mavlink.MAVLink_command_long_message(
                self.sysid,
                self.master.target_component,
                mavlink.MAV_CMD_REQUEST_MESSAGE if rate==0 else mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0, id, 
                0 if rate<=0 else 1e6/rate, 
                0, 0, 0, 0, 0    
        ))
        
    def receive_msg(self, id: int, timeout=0.2):
        finish = time() + timeout
        while time() < finish:
            response = self.master.recv_match(
                type = mavlink.mavlink_map[id].msgname, 
                blocking=True
            )
            if response.get_srcSystem() == self.sysid:
                if id in wrappers:
                    return wrappers[id](response)
                else:
                    return response
        else:
            pass
    
    def get_message(self, id, timeout=0.2):
        self.request_msg(id)
        return self.receive_msg(id, timeout)
    
    
    def subscribe(self, ids: List[int], rate=10):
        for id in ids:
            self.request_msg(id, 1e6/rate)
        
        #TODO this needs to identify which message requests are not in use by other observers and only close those.
        def close():
            for id in ids:
                self.request_msg(id, -1)
        
        observer = Observer(self, ids, close)
        return observer.start()
    
    
    
class Observer():
    def __init__(self, conn: Connection, ids: List[int], on_close) -> None:
        self.conn = conn
        self.ids = ids
        self.data = {id: None for id in ids}
        self.on_close =on_close
        self._alive = False
        self.thr = None
        self._wrapper_names = {wrappers[id].__name__.lower(): id for id in ids}
    
    def __getattr__(self, name: str):
        if name in mdefs.data:
            return self.data[getattr(mdefs, name).id]
        elif name.lower() in self._wrapper_names:
            return self.data[self._wrapper_names[name.lower()]]
        raise AttributeError(f"Attribute {name} not found")
    
    def __getitem__(self, i: int):
        return self.data[i]
    
    def run(self):
        self._alive = True
        while self._alive:
            msg = self.conn.master.recv_match(blocking=True)
            if not msg:
                continue
            elif hasattr(msg, "id") and msg.get_srcSystem() == self.conn.sysid:
                if msg.id in self.ids:
                    self.data[msg.id] = wrappers[msg.id](msg)    
    
    def start(self):
        self.thr = Thread(target=self.run, daemon=True)
        self.thr.start()
        return self
        
    def stop(self):
        self._alive = False
        self.thr.join()
        
