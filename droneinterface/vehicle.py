"""This wraps the mavlink Vehicle so you can request single messages or subscribe to streams of them
If wrappers do not exist for the requested message then the original message object is returned
"""
from __future__ import annotations
from typing import Union
from time import time
from .messages import wrappers, wrappermap
from loguru import logger
from .commands import command_map, all_commands
from flightdata import Origin
from .combinators import append_combinators
import inspect
from . import mavlink
from . import Connection, LastMessage
from pathlib import Path
from droneinterface.scheduling import Observer, Repeater, MessageWaiter, Timeout, TooOld, NeverReceived
from .scheduling import AwaitCondition

class Vehicle:
    def __init__(self, conn: Connection, sysid: int, compid:int, origin: Origin=None) -> None:
        super().__init__()
        self.conn: Connection = conn
        self.sysid = sysid
        self.compid = compid
        if self.sysid not in self.conn.msgs:
            self.conn.msgs[self.sysid] = {}
        self.msgs = self.conn.msgs[self.sysid]
        self.origin = origin
    
        append_combinators(self)

    def __str__(self):
        return f"Vehicle(add={self.conn.master.address}, sysid={self.sysid}, compid={self.compid})"
    
    @staticmethod
    def connect(constr: str, sysid: int=1, compid:int=1, wfb=True, origin: Origin=None, **kwargs) -> Vehicle:
        logger.info(f"Connecting to {constr}, sys {sysid}, comp {compid} ")
        conn = Connection.connect(constr, **kwargs)
        conn.start()

        veh = Vehicle(conn, sysid, compid)
        if wfb:
            veh = veh.wait_for_boot()
            origin = Origin("origin", veh.get_GlobalOrigin(None, None).position, 0.0) if origin is None else origin
            veh = veh.update(origin=origin)

        return veh
        
    @staticmethod
    def from_folder(outdir:Path, sysid: int, compid:int=1, origin: Origin=None):
        conn = Connection(outdir=outdir)
        veh = Vehicle(conn, sysid, compid)
        origin = Origin("origin", veh.last_globalorigin().position, 0.0) if origin is None else origin

        return veh.update(
            origin=origin
        )

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
                elif _spl[0] == 'send':
                    return lambda *args, **kwargs: self.send_message(wrappermap[_spl[1]](time(), self.sysid, self.compid, *args, **kwargs))
        if name in all_commands:
            return lambda *args, **kwargs : self.send_command(name, *args, **kwargs)
        
        if name in ['parameters', 'msgs']:
            return getattr(self.conn, name)[self.sysid]
        raise AttributeError(f"{name} not found in message wrappers or command map")
    
    def wait_for_test(self, test, timeout=None):
        start = time()
        check_t = lambda : time() < start + timeout  # noqa: E731
        if timeout is None:
            check_t = lambda : True  # noqa: E731
        while check_t() and self.conn.is_alive():
            if test():
                return
        raise Timeout(f"Timeout after {timeout} seconds waiting for test {test}")

    def wait_for_boot(self) -> Vehicle:
        logger.info("Waiting for boot")

        tests = {
            "IMU initialised": lambda : self.last_heartbeat().initialised,
            "GPS fix": lambda : self.last_GPSRawInt().fix_type > 2,
            "EKF happy": lambda : self.last_EKFStatus().is_good
        }
        for name, test in tests.items():
            informed = False
            while True:
                try:
                    if test():
                        if informed:
                            logger.debug(f"Got {name}") 
                        break
                except Exception:
                    pass
                if not informed:
                    logger.debug(f"Waiting for {name}")
                    informed = True

        logger.info("Booted")
        return self

    def send_command(self, name, *args, **kwargs):
        for msgid, cmap in command_map.items():
            if name in cmap:
                self.send_message(
                    wrappers[msgid](
                        time(), self.sysid, self.conn.master.target_component, 
                        *cmap[name]( *args, **kwargs)
                    )
                )
                break
        
    def send_message(self, msg):
        logger.debug(f"Sending message {str(msg)}")
        self.conn.master.mav.send(msg if isinstance(msg, mavlink.MAVLink_message) else msg.encoder())
    
    def get_parameter(self, name, timeout=1, use_cache = True):
        
        if not use_cache:
            if name in self.conn.parameters[self.sysid]:
                del self.conn.parameters[self.sysid][name]       
        
        waiter = AwaitCondition(lambda : name in self.conn.parameters[self.sysid], timeout)
        if name not in self.conn.parameters[self.sysid]:
            self.send_paramrequestread(name.encode('utf8'), -1)
        waiter.join()
        return self.conn.parameters[self.sysid][name]

    def set_parameter(self, name, value, timeout=1):
        ptype = self.get_parameter(name, timeout, True)[1]
        del self.conn.parameters[self.sysid][name]
        while True:
            try:
                ac = AwaitCondition(lambda : name in self.conn.parameters[self.sysid], timeout)
                self.send_paramset(name.encode('utf8'), value, ptype)
                ac.join()
                break
            except Timeout:
                pass
            

    def schedule(self, method, rate) -> Repeater:
        return Repeater(method, rate)

    def _last_message(self, id, max_age=None) -> LastMessage:
        """return the last message if it exists and it is less old than max_age. otherwise return None."""
        if id in self.msgs:
            lm = self.msgs[id]
            if max_age is None:
                return lm
            elif lm.last_time + max_age > time():
                return lm
            else:
                raise TooOld(f' Last message id {id} ({wrappermap[id].__class__name}) is more than {max_age} seconds old.')
        else:
            raise NeverReceived(f'Message {id} ({wrappermap[id].__class__name}) has never been received.')
        
    def _next_message(self, id, timeout=0.2) -> LastMessage:
        """Wait timeout seconds for the next message"""
        event = self.conn.add_waiter(self.sysid, id)
        if event.wait(timeout):
            return self._last_message(id, None)
        else:
            raise Timeout(f'timeout after {timeout} seconds waiting for message {id}')
        
    def _get_message(self, id, timeout=0.2, max_age=0.1) -> LastMessage:
        """get a message. 
        first try younger than max_age, 
        then wait for the next one, 
        request periodically while waiting"""
        timeout = 9999 if timeout is None else timeout
        try:
            return self._last_message(id, max_age)
        except Exception:
            if max_age is not None:
                try:
                    return self._last_message(id, None)
                except Exception:
                    pass
        
        stop = time() + timeout

        msgwaiter = self.conn.add_waiter(self.sysid, id)

        lastrequest=0

        msg=None
        while time() < stop:
            if msgwaiter.is_set():
                msg = self._last_message(id, None)
                break
            if time() > lastrequest + min(timeout, 1):
                if msg is None:
                    self.request_message(id)
                else:
                    if msg.rate < (1/(min(timeout, 2))):
                        self.request_message(id)
                lastrequest = time()
        else:
            logger.debug(f"Failed to receive msg {str(id)} after {timeout} seconds")
            msg=None
        
        self.conn.remove_waiter(self.sysid, id)

        logger.debug(f"Received message: {str(msg)}")
        return msg
    
    def _message(self, method: str, id, *args, **kwargs):
        msg = getattr(self, f"_{method}_message")(id, *args, **kwargs)
        return msg.wrapper() if msg is not None else None

    def last_message(self, id, *args, **kwargs):
            return self._message("last", id, *args, **kwargs)
    def next_message(self, id, *args, **kwargs):
            return self._message("next", id, *args, **kwargs)
    def get_message(self, id, *args, **kwargs):
            return self._message("get", id, *args, **kwargs)

    def subscribe(self, ids: Union[list[int], int], rate: int):
        return Observer(
            self, 
            [ids] if isinstance(ids, int) else ids, 
            rate
        )
    
    def parallel_messages(self, method: str, ids, *args, **kwargs):
        
        ths = [MessageWaiter(getattr(self, f"{method}_message"), id, *args, **kwargs) for id in ids]
        
        while any([th.is_alive() for th in ths]):
            pass
        return [th.result for th in ths]




