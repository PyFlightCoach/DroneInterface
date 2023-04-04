"""This wraps the mavlink Vehicle so you can request single messages or subscribe to streams of them
If wrappers do not exist for the requested message then the original message object is returned
"""
from __future__ import annotations
from pymavlink import mavutil
from . import mavlink
from typing import Union, List
from time import time
from .messages import wrappers, mdefs
from threading import Thread
import logging
from .commands import command_map
from flightanalysis import Box, State, FlightLine
from geometry import Coord
from .combinators import append_combinators
import inspect


wrappermap ={wr.__name__.lower(): wr for wr in wrappers.values()}

class TimeoutError(Exception):
        pass


class Vehicle:
    def __init__(self, master: mavutil.mavfile, sysid: int, flightline: FlightLine=None) -> None:
        self.master = master
        self.sysid = sysid
        if flightline is None:
            self.flightline = FlightLine.home()
        else:
            self.flightline = flightline
        append_combinators(self)


    def __str__(self):
        return f"Vehicle(add={self.master.address}, sysid={self.sysid})"
    
    @staticmethod
    def connect(constr: str, sysid: int, box: Box=None) -> Vehicle:
        logging.info(f"Connecting to {constr}, source {sysid}")
        conn = Vehicle(
            mavutil.mavlink_connection(constr), 
            sysid
        ).wait_for_boot()

        return conn if box is None else conn.update(flightline=FlightLine(
                    Box("home", conn.homeposition.home, 0),
                    box
            ))

    def update(self, **kwargs) -> Vehicle:
        args = inspect.getfullargspec(self.__class__.__init__)[0]
        args.remove("self")
        return Vehicle(
                *[kwargs[a] if a in kwargs else getattr(self, a) for a in args]
            )

    def __getattr__(self, name):
        name = name.lower()
        if name in wrappermap:
            return self.get_message(wrappermap[name].id, 0.2)
        elif name in command_map:
            return lambda *args : self.send_command(*command_map[name](*args))
        raise AttributeError(f"{name} not found in message wrappers")
    
    def wait_for_boot(self) -> Vehicle:
        logging.info("Waiting for boot")
        #print("Heartbeat from system (system %u component %u)" % (the_Vehicle.target_system, the_Vehicle.target_component))    
        while not self.receive_msg(mavlink.MAVLINK_MSG_ID_HEARTBEAT, timeout=2.0).initialised:
            pass
        logging.info(f"Booted")
        return self

    def send_command(self, command: int, *params):
        #MAV_CMD_COMPONENT_ARM_DISARM
        self.send_message(wrappers[76](
            time(), self.sysid, self.master.target_component, command, 0,
            *[params[i] if i < len(params) else 0 for i in range(8)]
        ))
        
    def send_message(self, msg):
        logging.info(f"sending message {str(msg)}")
        self.master.mav.send(msg if isinstance(msg, mavlink.MAVLink_message) else msg.encoder())
    
    def receive_msg(self, id: int, timeout=0.2):
        logging.debug(f"waiting for message {id}")
        finish = time() + timeout
        while time() < finish:
            response = self.master.recv_match(
                type = mavlink.mavlink_map[id].msgname, 
                blocking=True,
                timeout=timeout
            ) # TODO check the target component
            if response is None:
                break
            logging.debug(f"received {str(response)}")
            if response.get_srcSystem() == self.sysid:
                if id in wrappers:
                    return wrappers[id].parse(response)
                else:
                    return response
        
        logging.error(f"Timeout waiting for message id {id}")
        raise TimeoutError(f"Timeout after {timeout}s waiting for message {id}")
    
    def get_message(self, id, timeout=0.2):
        self.request_message(id)
        msg = self.receive_msg(id, timeout)
        logging.info(f"received message: {str(msg)}")
        return msg
    
    def subscribe(self, ids: List[int], rate=10) -> Observer:
        for id in ids:
            self.set_message_rate(id, 1e6/rate )
        
        #TODO this needs to identify which message requests are not in use by other observers and only close those.
        # or perhaps there is some way to identify which messages were started by request_msg?
        def close():
            for id in ids:
                self.set_message_rate(id, 0)
        return Observer(self, ids, close).start()
        

class Observer():
    def __init__(self, conn: Vehicle, ids: List[int], on_close) -> None:
        self.conn = conn
        self.ids = ids
        self.data = {id: None for id in ids}
        self.on_close = on_close
        self._alive = False
        self.thr = None
        self._wrapper_names = {wrappers[id].__name__.lower(): id for id in ids}
        append_combinators(self, self.ids)

    def __getattr__(self, name: str):
        if name in mdefs.data:
            return self.data[getattr(mdefs, name).id]
        elif name.lower() in self._wrapper_names:
            return self.data[self._wrapper_names[name.lower()]]
        else:
            return getattr(self.conn, name)
    
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
                    self.data[msg.id] = wrappers[msg.id].parse(msg)    
    
    def __enter__(self):
        return self
    
    def __exit__(self, xc_type, exc_value, exc_tb):
        self.stop()
    
    def __str__(self):
        return f"Observer(conn={self.conn}, messages={self.ids})"
    
    def start(self):
        logging.info(f"starting observer,  {self}")
        self.thr = Thread(target=self.run, daemon=True)
        self.thr.start()
        return self
        
    def stop(self):
        self._alive = False
        self.thr.join()
        self.on_close()
        logging.info(f"closed observer,  {self}")
        
