from __future__ import annotations
from pymavlink import mavutil, DFReader
from typing import Union, List, Dict
from time import time, sleep
from datetime import datetime
from threading import Thread, Event
from loguru import logger
from pathlib import Path
from tempfile import TemporaryDirectory
import numpy as np
import pandas as pd
import sys
from .last_message import LastMessage


class Connection(Thread):
    def __init__(
            self, 
            master: mavutil.mavfile=None, 
            outdir: Path=None, 
            store_messages: Union[List[int], str]="all", 
            append=True, 
            n=2,
            timeout=5
        ):
        super().__init__(daemon=True)
        self.master = master
        self.outdir = Path(TemporaryDirectory().name) if outdir is None else outdir
        self.outdir.mkdir(exist_ok=True)
        
        self.msgs: dict[int: dict[str: LastMessage]] = {}   #first key system id, second key message id
        self.parameters: dict[int: dict[str: float]] = {}   #first key system id, second key parameter name
        if any(Path(self.outdir).iterdir()):
            if not append:
                raise Exception("Outdir is not empty. Provide an empty directory or set append=True")
            else:
                for f in outdir.glob("*.csv"):
                    sysid = int(f.name.split("_")[0])
                    if sysid not in self.msgs:
                        self.msgs[sysid] = {}
                    lm = LastMessage.build_csv(f)
                    self.msgs[sysid][lm.id] = lm
        self.n = n
        self.timeout = timeout
        self.store_messages = store_messages
        if isinstance(self.store_messages, list):
            self.check_store = lambda id: id in self.store_messages
        elif self.store_messages == "all":
            self.check_store = lambda msgid: True
        else:
            self.check_store = lambda msgid: False

        self.source = "DF" if isinstance(master, DFReader.DFReader_binary) else "MAV"
        if self.source == "DF":
            self.builder = LastMessage.build_bin
            self._system_from_message = lambda msg: 1
            self._id_from_message = lambda msg : msg.get_type()            
        else:
            self.builder = LastMessage.build_mavlink
            self._system_from_message = lambda msg : msg.get_srcSystem()
            self._id_from_message = lambda msg : msg.__class__.id

        self.waiters = {}

    @staticmethod
    def connect(constr, outdir: Path=None, store_messages: Union[List[int], str]="all", append=True, n=2, timeout=5, **kwargs):
        return Connection(
            mavutil.mavlink_connection(constr, **kwargs),
            None if (outdir is None or store_messages=="none") else Connection.create_folder(outdir),
            store_messages, append, n, timeout
        )
    
    def __str__(self):
        return f"Connection({self.master.address})"

    def __getattr__(self, name):
        if name in self.msgs:
            return self.msgs[name]
        raise AttributeError(f"{name} not found in {self}")

    def run(self):
        last_t = None
        while self.is_alive():
            try:
                msg = self.master.recv_msg()
                if msg is None or not hasattr(msg, 'id'):
                    if self.source == "DF":
                        break
                    else:
                        if self.timeout is not None:
                            if last_t is not None:
                                if time() - last_t > self.timeout:
                                    raise Exception(f"Connection lost, timeout after {self.timeout} seconds.")
                        continue
                last_t = time()
                system_id = self._system_from_message(msg)
                if system_id not in self.msgs:
                    self.msgs[system_id] = {}
                if system_id not in self.waiters:
                    self.waiters[system_id] = {}
                msg_index = self.msgs[system_id]
                
                msg_id = self._id_from_message(msg)
                if msg_id not in msg_index:
                    msg_index[msg_id] = self.builder(
                        msg, 
                        (self.outdir / f"{system_id}_{msg_id}.csv") if self.check_store(msg_id) else None,
                        self.n
                    )
                
                msg_index[msg_id].receive_message(msg)

                #the events would be better as part of the LastMessage instances, 
                # but cant be as the msg_index dict is not pre-populated
                waiter_index = self.waiters[system_id]
                if msg_id in waiter_index:
                    waiter_index[msg_id].set()
                
                if system_id not in self.parameters:
                    self.parameters[system_id] = {}
                if msg_id == 22: # mavlink.MAVLINK_MSG_ID_PARAM_VALUE
                    self.parameters[system_id][msg.param_id] = (msg.param_value, msg.param_type)
                
            except Exception as ex:
                logger.exception(f'Connection Error {ex}')

    def add_waiter(self, systemid, msgid) -> Event:
        if systemid not in self.waiters:
            self.waiters[systemid] = {}
        waiter_index = self.waiters[systemid]
        if msgid in waiter_index:
            waiter_index[msgid].clear()
        else:
            waiter_index[msgid] = Event()
        return waiter_index[msgid]

    def remove_waiter(self, systemid, msgid):
        if systemid in self.waiters:
            if msgid in self.waiters[systemid]:
                del self.waiters[systemid][msgid]

    @staticmethod    
    def create_folder(path: Path):
        outdir = Path(path) / f"Conn_{datetime.now():%Y_%m_%d_%H_%M_%S}"
        outdir.mkdir()
        return outdir

    @staticmethod
    def parse_date(path: Path):
        if path.is_dir():
            if len(path.name) > 5:
                if path.name[:5] == "Conn_":
                    return datetime.strptime(path.name[5:], '%Y_%m_%d_%H_%M_%S')

    @staticmethod
    def parse_bin(path:str, store_messages: list):
        conn = Connection(
            mavutil.mavlink_connection(path),
            store_messages=store_messages,
        )
        conn.start()
        while conn.is_alive():
            sys.stdout.write("\r%d%%" % conn.master.percent)
            sys.stdout.flush()

        return conn.join_messages(store_messages)

    def join_messages(self, ids: list, systemid:int=1) -> pd.DataFrame:
        joined_log = self.msgs[systemid][ids[0]].all_messages()
        joined_log.columns = [f"{ids}_{c}" for c in joined_log.columns]
        for id in ids[1:]:
            if id=='PARM':
                continue
            df = self.msgs[systemid][id].all_messages()
            df.columns = [f"{id}_{c}" for c in df.columns]

            joined_log = pd.merge_asof(joined_log, df, on='timestamp')
            
        return joined_log

    def rates(self, ids: list=None, systemid=1):
        if ids is None:
            ids = self.msgs[systemid].keys()
        return [self.msgs[systemid][id].rate if id in self.msgs[systemid] else 0 for id in ids]


if __name__ == "__main__":
    logger.basicConfig(level=logger.INFO)

    conn = Connection(
        mavutil.mavlink_connection('tcp:127.0.0.1:5760'),
        Connection.create_folder(Path("log_tmp"))
    )
    conn.start()
    
    while True:
        logger.info({k: int(v.frequency) for k, v in conn.msgs.items()})
        sleep(1)
    

