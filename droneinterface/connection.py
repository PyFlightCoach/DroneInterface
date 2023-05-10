from __future__ import annotations
from pymavlink import mavutil, DFReader
from typing import Union, List, Dict, IO
from time import time, sleep
from datetime import datetime
from threading import Thread, Event
import logging
from flightanalysis import Box, State, FlightLine
from geometry import Coord
from droneinterface import mavlink
import numpy as np
from pathlib import Path
from tempfile import TemporaryDirectory
import pandas as pd
from numbers import Number
from functools import partial
from io import StringIO
import traceback
import sys
from collections import deque
from .messages import wrappers, mdefs, wrappermap


class LastMessage:
    def __init__(self, id, key, colmap: dict, rev_colmap: dict, outfile: Path=None, n=3):
        self.id = id
        self.key = key
        self.last_message = None
        self.times = deque(maxlen=n)
        self.last_time = None
        self.n = n
        self.count = 0
        self.colmap = colmap 
        self.rev_colmap = rev_colmap
        self.outfile = outfile
        if self.outfile is not None:
            
            with open(self.outfile, "w") as f:
                print(",".join(list(self.colmap.keys())), file=f)
            self.io = open(self.outfile, "a")

    @staticmethod
    def build_mavlink(key, msg, outfile: Path=None, n=3):
        colmap = {"timestamp": lambda msg: msg._timestamp}
        rev_colmap = {}
        for fname, l in zip(msg.__class__.ordered_fieldnames, msg.__class__.lengths):
            if l == 1:
                colmap[fname] = partial(lambda fname, msg: getattr(msg, fname), fname)
                rev_colmap[fname] = partial(lambda fname, data: data[fname], fname)
            else:
                for i in range(l):
                    colmap[f"{fname}_{i}"] = partial(lambda fname, i, msg: getattr(msg, fname)[i], fname, i)
                rev_colmap[fname] = partial(lambda fname, l, data: [data[f"{fname}_{i}"] for i in range(l)], fname, l)
        
        return LastMessage(
            msg.__class__.id, 
            key, 
            colmap, 
            rev_colmap, 
            outfile, 
            n
        )

    @staticmethod
    def build_bin(key, msg, outfile: Path=None, n=3):
        colmap = {"timestamp": lambda msg: msg._timestamp}
        rev_colmap = dict()
        for fname in msg._fieldnames:
            colmap[fname] = partial(lambda fname, msg: getattr(msg, fname), fname)
            rev_colmap[fname] = partial(lambda fname, data: data[fname], fname)

        return LastMessage(
            msg.get_type(), 
            key, 
            colmap, 
            rev_colmap, 
            outfile, 
            n
        )

    def receive_message(self, msg):        
        self.last_message = msg
        self.times.append(msg._timestamp)        
        self.last_time = time()
        self.count += 1
        if self.outfile is not None:
            data = [str(v(msg)) for v in self.colmap.values()]
            print(",".join(data), file=self.io)

        return self

    def next_message(self, timeout=2):
        t0 = self._times[0]
        end = time() + timeout
        while time() < end:
            if self.times[0] > t0:
                return self.last_message

    @property
    def rate(self):
        rate = np.round(1 / np.mean(np.diff(self.times)), 1)
        return 0 if np.isnan(rate) else rate

    def all_messages(self) -> pd.DataFrame:
        return pd.read_csv(self.outfile).set_index("timestamp")
    

    def wrapper(self):
        return wrappers[self.id].parse(self.last_message)

class Connection(Thread):
    def __init__(self, master: mavutil.mavfile, outdir: Path=None, store_messages: Union[List[int], str]="all", n=10):
        super().__init__(daemon=True)
        self.master = master
        self.outdir = Path(TemporaryDirectory().name) if outdir is None else outdir
        if not self.outdir.exists():
            self.outdir.mkdir(exist_ok=True)
        self.msgs: Dict[str: LastMessage] = {}
        self.n = n

        self.store_messages = store_messages
        if isinstance(self.store_messages, list):
            self.check_store = lambda key: key in self.store_messages
        elif self.store_messages == "all":
            self.check_store = lambda msgid: True
        else:
            self.check_store = lambda msgid: False

        self.source = "DF" if isinstance(master, DFReader.DFReader_binary) else "MAV"
        if self.source == "DF":
            self.builder = LastMessage.build_bin
            self._generate_key = self._generate_bin_key
            
        else:
            self.builder = LastMessage.build_mavlink
            self._generate_key = self._generate_mavlink_key

    def __str__(self):
        return f"Connection({self.master.address})"

    def __getattr__(self, name):
        if name in self.msgs:
            return self.msgs[name]
        raise AttributeError(f"{name} not found in {self}")

    def _generate_mavlink_key(self, msg):
        return f"{msg.get_srcSystem()}_{msg.get_srcComponent()}_{msg.__class__.id}"

    def _generate_bin_key(self, msg):
        return msg.get_type()

    def run(self):
        while self.is_alive():
            try:
                msg = self.master.recv_msg()
                if msg is None:
                    if self.source == "DF":
                        break
                    else:
                        continue
                key = self._generate_key(msg)
                
                if not key in self.msgs:
                    self.msgs[key] = self.builder(
                        key, 
                        msg, 
                        (self.outdir / f"{key}.csv") if self.check_store(key) else None,
                        self.n
                    )
                
                self.msg = self.msgs[key].receive_message(msg)
            except Exception as ex:
                logging.debug(traceback.format_exc())

    @staticmethod    
    def create_folder(path: Path):
        outdir = Path(path) / f"Conn_{datetime.now():%Y_%M_%d_%H_%M_%S}"
        outdir.mkdir()
        return outdir

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

    def join_messages(self, keys: list) -> pd.DataFrame:
        keys = [k for k in keys if k in self.msgs.keys()]

        joined_log = self.msgs[keys[0]].all_messages()
        joined_log.columns = [f"{keys[0]}_{c}" for c in joined_log.columns]
        for k in keys[1:]:
            if k=='PARM':
                continue
            df = self.msgs[k].all_messages()
            df.columns = [f"{k}_{c}" for c in df.columns]

            joined_log = pd.merge_asof(
                joined_log, 
                df, 
                on='timestamp'
            )
            
        return joined_log

    def rates(self, keys: list=None):
        if keys is None:
            keys = self.msgs.keys()
        return [self.msgs[k].rate if k in self.msgs else 0 for k in keys]



if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)

    conn = Connection(
        mavutil.mavlink_connection('tcp:127.0.0.1:5760'),
        Connection.create_folder(Path("log_tmp"))
    )
    conn.start()
    
    import plotly.express as px
    
    while True:
        logging.info({k: int(v.frequency) for k, v in conn.msgs.items()})
        sleep(1)
    

