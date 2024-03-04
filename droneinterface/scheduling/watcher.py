"""Call a function repeatedly in a thread and append the result to a deque"""
from threading import Thread
from .. import logger
from time import time
from . import Timeout
from collections import deque
import numpy as np 
import pandas as pd


class Watcher(Thread):
    def __init__(self, fun, maxlen, timeout=None) -> None:
        super().__init__(daemon=True)
        self.fun = fun
        self.times = deque(maxlen=maxlen)
        self.data = deque(maxlen=maxlen)
        self.timeout = timeout
        self.start_time = None
        self.maxlen = maxlen
        self.start()

    def run(self):
        while not self._is_stopped:
            try:
                self.data.append(self.fun())
                if self.start_time is None:
                    self.start_time = time()
                self.times.append(time() - self.start_time)
            except Exception as ex:
                logger.debug(ex)
            
            self.check_timeout()

    def last_result_age(self):
        return time() - (self.start_time + self.times[-1])

    def stop(self):
        self._is_stopped = True
        self.join()

    def check_timeout(self):
        if self.timeout is not None:
            if time() - self.start_time > self._timeout:
                self.stop()
                raise Timeout(f"Timeout after {self.timeout} seconds watching {self.fun}")
    
    def dataframe(self, **kwargs):
        data = self.data.copy()
        times = list(self.times)
        l = min(len(data), len(times))
        df = pd.DataFrame(list(data)[:l], **kwargs)
        df.index = times[:l]
        return df

    def reset(self):
        self.times = deque(maxlen=self.maxlen)
        self.data = deque(maxlen=self.maxlen)
        self.timeout = self.timeout
        self.start_time = None
        
        