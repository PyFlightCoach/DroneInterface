from __future__ import annotations
from typing import Union, List, Dict
from time import time
import numpy as np
from pathlib import Path
import pandas as pd
from functools import partial
from collections import deque
from .messages import wrappers

from . import mavlink

class LastMessage:
    def __init__(self, id, colmap: dict, rev_colmap: dict, outfile: Path=None, n=3):
        self.id = id
        self.history = deque(maxlen=n)
        self.last_time = None
        self.colmap = colmap 
        self.rev_colmap = rev_colmap
        self.rate = 0
        self.outfile = outfile
        if self.outfile is not None:
            if not self.outfile.exists():
                with open(self.outfile, "w") as f:
                    print(",".join(list(self.colmap.keys())), file=f)
            else:
                df = pd.read_csv(self.outfile)
                for i in range(n):
                    if i < len(df):
                        self.history.appendleft(self.create_message(df.iloc[-(i+1)]))
            self.io = open(self.outfile, "a")

    def __repr__(self):
        return f'LastMessage(age={time()-self.last_time}, rate={self.rate}, msg={self.last_message})'
    
    @property
    def times(self) -> List[float]:
        return [m._timestamp for m in self.history]

    @property
    def last_message(self):
        return self.history[-1]

    @staticmethod
    def _build_mavlink(msgcls, outfile: Path=None, n=3):
        colmap = {"timestamp": lambda msg: msg._timestamp}
        rev_colmap = {}
        for fname, l in zip(msgcls.ordered_fieldnames, msgcls.lengths):
            if l == 1:
                colmap[fname] = partial(lambda fname, msg: getattr(msg, fname), fname)
                rev_colmap[fname] = partial(lambda fname, data: data[fname], fname)
            else:
                for i in range(l):
                    colmap[f"{fname}_{i}"] = partial(lambda fname, i, msg: getattr(msg, fname)[i], fname, i)
                rev_colmap[fname] = partial(lambda fname, l, data: [data[f"{fname}_{i}"] for i in range(l)], fname, l)
        
        return LastMessage(
            msgcls.id, 
            colmap, 
            rev_colmap, 
            outfile, 
            n
        )

    @staticmethod
    def build_mavlink(msg, outfile: Path=None, n=3):
        return LastMessage._build_mavlink(msg.__class__, outfile, n)

    @staticmethod
    def build_csv(outfile: Path):
        msgid = int(outfile.stem.split("_")[1])         
        return LastMessage._build_mavlink(mavlink.mavlink_map[msgid], outfile)

    @staticmethod
    def build_bin(msg, outfile: Path=None, n=3):
        colmap = {"timestamp": lambda msg: msg._timestamp}
        rev_colmap = dict()
        for fname in msg._fieldnames:
            colmap[fname] = partial(lambda fname, msg: getattr(msg, fname), fname)
            rev_colmap[fname] = partial(lambda fname, data: data[fname], fname)

        return LastMessage(
            msg.get_type(), 
            colmap, 
            rev_colmap, 
            outfile, 
            n
        )

    def receive_message(self, msg):        
        self.history.append(msg)
        t = time()
        n = len(self.history)
        if not self.last_time is None:
            self.rate = self.rate * (n - 1) / n +  1 / (n * (t - self.last_time))
        self.last_time = t
        if self.outfile is not None:
            data = [str(v(msg)) for v in self.colmap.values()]
            print(",".join(data), file=self.io)
            self.io.flush()

    def create_message(self, data:pd.Series):
        msg =  mavlink.mavlink_map[self.id](**{k:rmap(data) for k, rmap in self.rev_colmap.items()})
        msg._timestamp = data.name
        return msg

   # @property
   # def rate(self):
   #     rate = np.round(1 / np.mean(np.diff(self.times)), 0)
   #     return 0 if np.isnan(rate) else rate

    def all_messages(self) -> pd.DataFrame:
        return pd.read_csv(self.outfile).set_index("timestamp")

    def wrapper(self, i=-1):
        return wrappers[self.id].parse(self.history[i])
