"""Call a function repeatedly and become true when the function returns true"""
from threading import Thread
from .. import logger
from time import time
from . import Timeout

class AwaitCondition(Thread):
    def __init__(self, fun, timeout=None) -> None:
        super().__init__(daemon=True)
        self.fun = fun
        self.result = False
        self.timeout = timeout
        self.start_time = time()
        self.start()

    def run(self):
        while not self.result and not self._is_stopped:
            try:
                self.result = self.fun()
                if self.result:
                    logger.debug('Await Condition Met!')
            except Exception as ex:
                logger.debug(ex)
            
            self.check_timeout()


    def stop(self):
        self._is_stopped = True
        self.join()

    def check_timeout(self):
        if self.timeout is not None:
            if time() - self.start_time > self._timeout:
                self.stop()
                raise Timeout(f"Timeout after {self.timeout} seconds waiting for test {self.fun}")