"""This wraps the mavlink Vehicle so you can request single messages or subscribe to streams of them
If wrappers do not exist for the requested message then the original message object is returned
"""
from __future__ import annotations
from collections.abc import Callable, Iterable, Mapping
from pymavlink import mavutil
from . import mavlink
from typing import Any, Union, List
from time import time, sleep
from .messages import wrappers, mdefs, wrappermap
from threading import Thread, Event
import logging
from .commands import command_map
from flightanalysis import Box, State, FlightLine
from geometry import Coord, GPS, Point
from .combinators import append_combinators
import inspect
from . import Base
from . import Connection, LastMessage
from pathlib import Path
from functools import partial


class TimeoutError(Exception):
        pass


class Vehicle(Base):
    def __init__(self, conn: Connection, sysid: int, compid:int, box: Box=None) -> None:
        super().__init__()
        self.conn: Connection = conn
        self.sysid = sysid
        self.compid = compid
        if not self.sysid in self.conn.msgs:
            self.conn.msgs[self.sysid] = {}
        self.msgs = self.conn.msgs[self.sysid]

        append_combinators(self)

        self.wait_for_boot()
        self.origin = self.get_GlobalOrigin(None, None).position
        
        if box is None:
            self.box = Box("origin", self.origin, 0.0)
        self.flightline = FlightLine.from_box(self.box, self.origin)


    def __str__(self):
        return f"Vehicle(add={self.conn.master.address}, sysid={self.sysid}, compid={self.compid})"
    
    @staticmethod
    def connect(constr: str, sysid: int, compid:int=1, outdir: Path=None, box: Box=None, store_messages="none", n=3, **kwargs) -> Vehicle:
        logging.info(f"Connecting to {constr}, sys {sysid}, comp {compid} ")
        conn = Connection(
            mavutil.mavlink_connection(constr, **kwargs), 
            None if (outdir is None or store_messages=="none") else Connection.create_folder(outdir),
            store_messages,
            n
        )

        conn.start()
        return Vehicle(conn, sysid, compid, box).wait_for_boot()
        

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
                if _spl[0] in ["last", "get", "next", "_last", "_get", "_next"]:
                    return lambda *args, **kwargs: getattr(self, f"{_spl[0]}_message")(wrappermap[_spl[1]].id, *args, **kwargs)    
        if name in command_map:
            return lambda *args : self.send_command(*command_map[name](*args))
        raise AttributeError(f"{name} not found in message wrappers or command map")
    
    def wait_for_boot(self) -> Vehicle:
        logging.info(self._msg("Waiting for boot"))

        tests = {
            "IMU initialised": lambda : self.last_heartbeat().initialised,
            "GPS fix": lambda : self.last_GPSRawInt().fix_type > 2,
            "EKF happy": lambda : self.last_EKFStatus().is_good
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
    
    def schedule(self, method, rate) -> Repeater:
        return Repeater(method, rate)

    def _last_message(self, id, max_age=None) -> LastMessage:
        """return the last message if it exists and it is less old than max_age. otherwise return None."""
        if id in self.msgs:
            lm = self.msgs[id]
            if max_age is None:
                return lm
            if lm.last_time + max_age > time():
                return lm

    def _next_message(self, id, timeout=0.2) -> LastMessage:
        """Wait timeout seconds for the next message"""
        event = self.conn.add_waiter(self.sysid, id)
        event.wait(timeout)

        return self._last_message(id, None)
    
    def _get_message(self, id, timeout=0.2, max_age=0.1) -> LastMessage:
        """get a message. 
        first try younger than max_age, 
        then wait if its likely to turn up in less than timout, 
        then request and wait timeout"""
        lm = self._last_message(id, max_age)
        if not lm is None:
            return lm
        elif not max_age is None:
            lm = self._last_message(id, None)
        if lm is None or timeout is None:
            self.request_message(id)
        else:
            if lm.rate < (1/timeout):
                self.request_message(id)          

        msg = self._next_message(id, timeout)
        logging.debug(self._msg(f"Received message: {str(msg)}"))
        return msg
    
    def _message(self, method: str, id, *args, **kwargs):
        msg = getattr(self, f"_{method}_message")(id, *args, **kwargs)
        return msg.wrapper() if not msg is None else None

    last_message = lambda self, id, *args, **kwargs: self._message("last", id, *args, **kwargs)
    next_message = lambda self, id, *args, **kwargs: self._message("next", id, *args, **kwargs)
    get_message = lambda self, id, *args, **kwargs: self._message("get", id, *args, **kwargs)

    def subscribe(self, ids: List[int], rate: int):
        return Observer(self, ids, rate)
    
    def async_messages(self, method: str, ids, *args, **kwargs):

        ths = [MessageWaiter(getattr(self, f"{method}_message"), id, *args, **kwargs) for id in ids]
        
        while any([th.is_alive() for th in ths]):
            pass
        return [th.result for th in ths]


class MessageWaiter(Thread):
    def __init__(self, target, *args, **kwargs) -> None:
        super().__init__(daemon=True)
        self.target = partial(target, *args, **kwargs)
        self.result = None
        self.start()

    def run(self):
        self.result = self.target()



class RateHistory:
    def __init__(self, veh, id, desired_rate):
        self.id = id
            
        self._last_message = veh._get_message(id, 2.0, None)
        
        self.initial_rate = max(self.current_rate(), 1.0)
        self.desired_rate = desired_rate
                
    def current_rate(self):
        return self._last_message.rate

    def set_rate(self, veh: Vehicle):
        rate = self.current_rate()
        if rate < self.desired_rate:
            logging.debug(f"increasing rate for msg {self.id} from {rate} to {self.desired_rate}")
            veh.set_message_rate(self._last_message.id, self.desired_rate * 1.5)

    def reset_rate(self, veh: Vehicle):
        veh.set_message_rate(self._last_message.id, self.initial_rate)


class Observer(Thread):
    def __init__(self, veh: Vehicle, ids: List[int], rate: int) -> None:
        super().__init__(daemon=True)
        self.veh = veh
        self.base_rates = {id: RateHistory(self.veh, id, rate) for id in ids}
        

    def __getattr__(self, name):
        return getattr(self.veh, name)
    
    def run(self):
        while not self._is_stopped:
            for rh in self.base_rates.values():
                rh.set_rate(self.veh)
            sleep(5)

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


class Repeater(Thread):
    def __init__(self, method, rate):
        super().__init__(daemon=True)
        self.method = method
        self.rate = rate

    def run(self):
        while not self._is_stopped:
            self.method()
            sleep(1/self.rate)
    
    def stop(self):
        self._is_stopped = True
        self.join() 

    def __enter__(self):
        self.start()
        return self
    
    def __exit__(self, xc_type, exc_value, exc_tb):
        self.stop()
