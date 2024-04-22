from __future__ import annotations
from pymavlink import mavutil, DFReader
from typing import Union
from time import time, sleep
from datetime import datetime
from threading import Thread, Event
from loguru import logger
from pathlib import Path
from tempfile import TemporaryDirectory
import pandas as pd
import sys
from .last_message import LastMessage
from .scheduling.exceptions import Timeout


class Connection(Thread):
    def __init__(
            self, 
            master: mavutil.mavfile=None, 
            outdir: Path=None, 
            store_messages: Union[list[int], str]="all", 
            append=True, 
            n=2,
            timeout=5
        ):
        super().__init__(daemon=True)
        self.master = master
        self.outdir = Path(TemporaryDirectory().name) if outdir is None else outdir
        self.outdir.mkdir(exist_ok=True)
        
        self.systems: list[int] = []
        self.msgs: dict[int: dict[str: LastMessage]] = {}   #first key system id, second key message id
        
        self.waiters = {}
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
        self.last_t = None

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

    @staticmethod
    def connect(constr, outdir: Path=None, store_messages: Union[list[int], str]="all", append=True, n=2, timeout=5, **kwargs):
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
        while self.is_alive():
            try:
                msg = self.master.recv_msg()
                if msg is None or not hasattr(msg, 'id'):
                    if self.source == "DF":
                        break
                    else:
                        if self.timeout is not None:
                            if self.last_t is not None:
                                if time() - self.last_t > self.timeout:
                                    raise Timeout(f"Connection lost, timeout after {self.timeout} seconds.")
                        continue
                self.last_t = time()
                system_id, msg = self.receive_message(msg)
                
                waiter_index = self.waiters[system_id]
                if msg.id in waiter_index:
                    waiter_index[msg.id].set()

            except Exception as ex:
                logger.exception(f'Connection Error {ex}')
                if isinstance(ex, Timeout):
                    break
    def receive_message(self, msg):
        system_id = self.add_system(self._system_from_message(msg))

        msg_id = self._id_from_message(msg)
        
        if msg_id not in self.msgs[system_id]:
            self.msgs[system_id][msg_id] = self.builder(
                msg, 
                (self.outdir / f"{system_id}_{msg_id}.csv") if self.check_store(msg_id) else None,
                self.n
            )
        
        return system_id, self.msgs[system_id][msg_id].receive_message(msg)
    
    def add_system(self, system_id):
        if system_id not in self.systems:
            self.systems.append(system_id)
            self.msgs[system_id] = {}
            self.waiters[system_id] = {}
        return system_id

    def add_waiter(self, systemid, msgid) -> Event:
        if msgid in self.waiters[systemid]:
            self.waiters[systemid][msgid].clear()
        else:
            self.waiters[systemid][msgid] = Event()
        return self.waiters[systemid][msgid]

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
    

