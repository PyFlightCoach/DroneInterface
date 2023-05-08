"""This wraps the mavlink Vehicle so you can request single messages or subscribe to streams of them
If wrappers do not exist for the requested message then the original message object is returned
"""
from __future__ import annotations
from pymavlink import mavutil
from . import mavlink
from typing import Union, List
from time import time, sleep
from .messages import wrappers, mdefs, wrappermap
from threading import Thread, Event
import logging
from .commands import command_map
from flightanalysis import Box, State, FlightLine
from geometry import Coord
from .combinators import append_combinators
import inspect
from . import Base
from . import Connection, LastMessage
from pathlib import Path



class TimeoutError(Exception):
        pass


class Vehicle(Base):
    def __init__(self, conn: Connection, sysid: int, compid:int, flightline: FlightLine=None) -> None:
        super().__init__()
        self.conn: Connection = conn
        self.sysid = sysid
        self.compid = compid
        if flightline is None:
            self.flightline = FlightLine.home()
        else:
            self.flightline = flightline
        append_combinators(self)


    def __str__(self):
        return f"Vehicle(add={self.conn.master.address}, sysid={self.sysid}, compid={self.compid})"
    
    @staticmethod
    def connect(constr: str, sysid: int, compid:int=1, outdir: Path=None, box: Box=None, **kwargs) -> Vehicle:
        logging.info(f"Connecting to {constr}, sys {sysid}, comp {compid} ")
        conn = Connection(
            mavutil.mavlink_connection(constr, **kwargs), 
            Connection.create_folder(outdir)
        )
        conn.start()
        _veh = Vehicle(
            conn, 
            sysid, compid
        ).wait_for_boot()
        
        origin = _veh.get_GlobalOrigin().position

        if box is None:
            box = Box("home", origin, 0)

        return _veh.update(flightline=FlightLine.from_box(box, origin))

    def update(self, **kwargs) -> Vehicle:
        args = inspect.getfullargspec(self.__class__.__init__)[0]
        args.remove("self")
        return Vehicle(
                *[(kwargs[a] if (a in kwargs) else getattr(self, a)) for a in args]
            )

    def __getattr__(self, name):
        name = name.lower()
        if "_" in name:
            _spl = name.split("_")
            if _spl[1] in wrappermap:
                if _spl[0] == "last":
                    return self.last_message(wrappermap[_spl[1]].id)
                elif _spl[0] == "get":
                    return lambda timeout=0.5: self.get_message(wrappermap[_spl[1]].id, timeout)
        if name in command_map:
            return lambda *args : self.send_command(*command_map[name](*args))
        raise AttributeError(f"{name} not found in message wrappers or command map")
    
    def wait_for_boot(self) -> Vehicle:
        logging.info(self._msg("Waiting for boot"))

        tests = {
            "IMU initialised": lambda : self.last_heartbeat.initialised,
            "GPS fix": lambda : self.last_GPSRawInt.fix_type > 2,
            "EKF happy": lambda : self.last_EKFStatus.is_good
        }
        for name, test in tests.items():
            while True:
                try:
                    if not test():
                        continue
                    break
                except Exception as e:
                    pass
                logging.debug(self._msg(f"Waiting for {name}"))
                sleep(0.5)

        logging.info(self._msg("Booted"))
        return self

    def send_command(self, command: int, *params):
        self.send_message(wrappers[76](
            time(), self.sysid, self.conn.master.target_component, command, 0,
            *[params[i] if i < len(params) else 0 for i in range(8)]
        ))
        
    def send_message(self, msg):
        logging.debug(self._msg(f"Sending message {str(msg)}"))
        self.conn.master.mav.send(msg if isinstance(msg, mavlink.MAVLink_message) else msg.encoder())
    
    def msg_key(self, id):
        return f"{self.sysid}_{self.compid}_{id}"

    def last_message(self, id):
        key = self.msg_key(id)
        if id in wrappers and key in self.conn.msgs:
            return wrappers[id].parse(self.conn.msgs[key].last_message)
    
    def next_message(self, id, timeout=0.2):
        end = time() + timeout
        while time() < end:
            msg = self.conn.msg
            if msg.id == id:
                return wrappers[id].parse(msg.last_message)

    def get_message(self, id, timeout=0.2):
        self.request_message(id)
        msg = self.next_message(id, timeout)
        logging.debug(self._msg(f"Received message: {str(msg)}"))
        return msg
    
    def subscribe(self, ids: List[int], rate: int):
        return Observer(self, ids, rate)
        

class RateHistory:
    def __init__(self, veh, id, desired_rate):
        self.key = veh.msg_key(id)
        if not self.key in veh.conn.msgs:
            veh.get_message(id)
        self.last_message = veh.conn.msgs[veh.msg_key(id)]
        
        self.initial_rate = self.current_rate()
        self.desired_rate = max(self.initial_rate, desired_rate)
                
    def current_rate(self):
        return self.last_message.rate

    def set_rate(self, veh: Vehicle):
        if self.current_rate() < self.desired_rate:
            veh.set_message_rate(self.last_message.id, self.desired_rate)

    def reset_rate(self, veh: Vehicle):
        veh.set_message_rate(self.last_message.id, self.initial_rate)


class Observer(Thread):
    def __init__(self, veh: Vehicle, ids: List[int], rate: int) -> None:
        super().__init__(daemon=True)
        self.veh = veh
        self.desired_rate = rate
        self.base_rates = {id: RateHistory(self.veh, id, rate) for id in ids}
        

    def __getattr__(self, name):
        return getattr(self.veh, name)
    
    def run(self):
        while not self._is_stopped:
            for rh in self.base_rates.values():
                rh.set_rate(self.veh)
            sleep(1)

    def stop(self):
        self._is_stopped = True
        self.join()
        for rh in self.base_rates.values():
            rh.reset_rate(self.veh)

    def __enter__(self):
        self.start()
        return self
    
    def __exit__(self, xc_type, exc_value, exc_tb):
        self.stop()
        