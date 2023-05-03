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
from . import Connection
from pathlib import Path

wrappermap ={wr.__name__.lower(): wr for wr in wrappers.values()}

class TimeoutError(Exception):
        pass


class Vehicle(Base):
    def __init__(self, conn: Connection, sysid: int, compid:int, flightline: FlightLine=None) -> None:
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
        if name[:4] == "last":
            return self.last_message(wrappermap[name[5:]].id)
        elif name[:3] == "get":
            return lambda timeout=0.5: self.get_message(wrappermap[name[4:]].id, timeout)
        elif name in command_map:
            return lambda *args : self.send_command(*command_map[name](*args))
        raise AttributeError(f"{name} not found in message wrappers")
    
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
        
    def get_message(self, id, timeout=0.2):
        try:
            return self.last_message(id)
        except Exception as e:
            self.request_message(id)
            msg = self.await_message(id, timeout)
            logging.debug(self._msg(f"Received message: {str(msg)}"))
            return msg
