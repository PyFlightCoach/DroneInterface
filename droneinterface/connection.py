from pymavlink import mavutil
from . import mavlink
from typing import Union, List
from time import time, sleep
from .messages.message_wrappers import wrappers
from threading import Thread


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
        while not self.get_message(mavlink.MAVLINK_MSG_ID_HEARTBEAT).initialised:
            pass
        return self

            
    def request_msg(self, id: int):
        self.master.mav.send(mavlink.MAVLink_command_long_message(
                self.sysid,
                self.master.target_component,
                mavlink.MAV_CMD_REQUEST_MESSAGE,
                0, id, 0, 0, 0, 0, 0, 0    
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
        self.master.mav.request_data_stream_send(0, 0, mavlink.MAV_DATA_STREAM_ALL, rate, 1)
        
        observer = Observer(self, rate, ids, 
            lambda : self.master.mav.request_data_stream_send(0, 0, mavlink.MAV_DATA_STREAM_ALL, rate, 0)
        )
        return observer.start()
    
    
    
class Observer():
    def __init__(self, conn: Connection, rate:int, ids: List[int], on_close) -> None:
        self.conn = conn
        self.timestep = 1 / rate
        self.ids = ids
        self.data = {id: None for id in ids}
        self.on_close =on_close
        self._alive = False
        self.thr = None
        
    def run(self):
        self._alive = True
        while self._alive:
            next_read = time() + self.timestep
            self.data = {id: self.conn.receive_msg(id) for id in self.ids}
            sleeptime = next_read - time()
            if sleeptime > 0:
                sleep(sleeptime)
        else:
            self.on_close()
    
    def start(self):
        self.thr = Thread(target=self.run, daemon=True)
        self.thr.start()
        return self
        
    def stop(self):
        self._alive = False
        self.thr.join()
        
