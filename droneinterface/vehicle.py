"""This wraps the mavlink Vehicle so you can request single messages or subscribe to streams of them
If wrappers do not exist for the requested message then the original message object is returned
"""
from __future__ import annotations
from pymavlink import mavutil
from . import mavlink
from typing import Union, List
from time import time, sleep
from .messages import wrappers, mdefs
from threading import Thread, Event
import logging
from .commands import command_map
from flightanalysis import Box, State, FlightLine
from geometry import Coord
from .combinators import append_combinators
import inspect
from . import Base

wrappermap ={wr.__name__.lower(): wr for wr in wrappers.values()}

class TimeoutError(Exception):
        pass


class Vehicle(Base):
    def __init__(self, master: mavutil.mavfile, sysid: int, compid:int, flightline: FlightLine=None) -> None:
        self.master = master
        self.sysid = sysid
        self.compid = compid
        if flightline is None:
            self.flightline = FlightLine.home()
        else:
            self.flightline = flightline
        append_combinators(self)


    def __str__(self):
        return f"Vehicle(add={self.master.address}, sysid={self.sysid})"
    
    @staticmethod
    def connect(constr: str, sysid: int, compid:int=1, box: Box=None, **kwargs) -> Vehicle:
        logging.info(f"Connecting to {constr}, sys {sysid}, comp {compid} ")
        
        conn = Vehicle(
            mavutil.mavlink_connection(constr, **kwargs), 
            sysid, compid
        ).wait_for_boot()
        
        origin = conn.GlobalOrigin.position

        if box is None:
            box = Box("home", origin, 0)

        return conn.update(flightline=FlightLine.from_box(box, origin))

    def update(self, **kwargs) -> Vehicle:
        args = inspect.getfullargspec(self.__class__.__init__)[0]
        args.remove("self")
        return Vehicle(
                *[(kwargs[a] if (a in kwargs) else getattr(self, a)) for a in args]
            )

    def __getattr__(self, name):
        name = name.lower()
        if name in wrappermap:
            return self.get_message(wrappermap[name].id, 0.2)
        elif name in command_map:
            return lambda *args : self.send_command(*command_map[name](*args))
        raise AttributeError(f"{name} not found in message wrappers")
    
    def wait_for_boot(self, timeout=20) -> Vehicle:
        logging.info(self._msg("Waiting for boot"))
        end = time() + timeout
        with self.subscribe([0, 24, 193], 4, False, False) as checks:
            while True:
                results = {
                    "IMU initialised": checks.heartbeat.initialised,
                    "GPS fix": checks.GPSRawInt.fix_type > 2,
                    "EKF happy": checks.EKFStatus.is_good
                }
                if all(list(results.values())):
                    break
                logging.debug(self._msg(f"Waiting for {[k for k, v in results.items() if not v]}"))
                sleep(0.5)

        logging.info(self._msg("Booted"))
        return self

    def send_command(self, command: int, *params):
        self.send_message(wrappers[76](
            time(), self.sysid, self.master.target_component, command, 0,
            *[params[i] if i < len(params) else 0 for i in range(8)]
        ))
        
    def send_message(self, msg):
        logging.debug(self._msg(f"Sending message {str(msg)}"))
        self.master.mav.send(msg if isinstance(msg, mavlink.MAVLink_message) else msg.encoder())
    
    def receive_msg(self, id: int, timeout=0.2):
        logging.debug(self._msg(f"Waiting for message {id}"))
        finish = time() + timeout
        while time() < finish:
            response: mavlink.MAVLink_message = self.master.recv_match(
                type = mavlink.mavlink_map[id].msgname, 
                blocking=True,
                timeout=timeout
            ) # TODO check the target component
            if response is None:
                break
            logging.debug(self._msg(f"Received {str(response)}"))
            if response.get_srcSystem() == self.sysid and response.get_srcComponent() == self.compid:
                if id in wrappers:
                    return wrappers[id].parse(response)
                else:
                    return response
        
        logging.error(self._msg(f"Timeout waiting for message id {id}"))
        raise TimeoutError(f"Timeout after {timeout}s waiting for message {id}")
    
    def get_message(self, id, timeout=0.2):
        self.request_message(id)
        msg = self.receive_msg(id, timeout)
        logging.debug(self._msg(f"Received message: {str(msg)}"))
        return msg
    
    def subscribe(self, ids: List[int], rate=10, set_rates=True, cleanup=False, wait=True) -> Observer:
        if set_rates:
            for id in ids:
                self.set_message_rate(id, 1e6/rate )
        
        #TODO this needs to identify which message requests are not in use by other observers and only close those.
        # or perhaps there is some way to identify which messages were started by request_msg?
        def close():
            for id in ids:
                self.set_message_rate(id, 0)
        
        return Observer(self, ids, close if cleanup else lambda : None, rate).start(wait)
        

class Observer(Base):
    def __init__(self, conn: Vehicle, ids: List[int], on_close, rate) -> None:
        self.conn = conn
        self.ids = ids
        self.data = {id: None for id in ids}
        self.on_close = on_close
        self._alive = False
        self.thr = None
        self._wrapper_names = {wrappers[id].__name__.lower(): id for id in ids}
        self.rate = rate
        self._last_message = None
        self.new_message = Event()
        append_combinators(self, self.ids)

    @property
    def last_message(self):
        self.new_message.clear()
        return self._last_message

    @property
    def next_message(self):
        self.new_message.wait()
        return self.last_message

    def __getattr__(self, name: str):
        if name in mdefs.data:
            return self.data[getattr(mdefs, name).id]
        elif name.lower() in self._wrapper_names:
            return self.data[self._wrapper_names[name.lower()]]
        elif not name in mdefs and not name.lower() in wrappers:
            return getattr(self.conn, name)
        elif len(name) > 6: 
            if name[:6] == "next_":
                self.new_message.wait()
                res = getattr(self, f"get_{name[6:]}")()
                self.new_message.clear()
                return res
        raise AttributeError(f"{name} does not exist in {self.__class__.__name__}")
    
    def __getitem__(self, i: int):
        return self.data[i]
    
    def _process_message(self, msg):
        if hasattr(msg, "id") and msg.get_srcSystem() == self.conn.sysid:
            if msg.id in self.ids:
                self._last_message = wrappers[msg.id].parse(msg)
                self.data[msg.id] = self._last_message
                self.new_message.set()
                
    def run(self):
        self._alive = True
        while self._alive:
            try:
                self._process_message(self.conn.master.recv_match(blocking=True))
            except Exception as ex:
                logging.debug(ex)
            
    def __enter__(self):
        return self
    
    def __exit__(self, xc_type, exc_value, exc_tb):
        self.stop()
    
    def __str__(self):
        return f"Observer(conn={self.conn}, messages={self.ids})"
    
    def data_is_populated(self):
        return all([not v is None for v in self.data.values()])

    def missing_messages(self):
        return [wrappers[i].__name__ for i in self.ids if self.data[i] is None]

    def start(self, wait=True):
        logging.debug(self._msg("Starting"))
        self.thr = Thread(target=self.run, daemon=True)
        self.thr.start()
        if wait:
            while not self.data_is_populated():
                logging.debug(self._msg(f"Waiting for data {self.missing_messages()}"))
                sleep(1/self.rate)
        logging.debug(self._msg("Running"))
        return self

    def stop(self):
        self._alive = False
        self.thr.join()
        self.on_close()
        logging.debug(self._msg("Stopped"))
        
