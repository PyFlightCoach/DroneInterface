
from threading import Thread
from .. import logger
from time import time



class Repeater(Thread):
    def __init__(self, method, rate):
        super().__init__(daemon=True)
        self.method = method
        self.rate = rate

    def run(self):
        last_call = 0
        while not self._is_stopped:
            if time() - last_call >= 1 / self.rate:
                self.method()
                last_call = time()
        logger.info("Repeater Stopped")
    
    def stop(self):
        self._is_stopped = True
        self.join() 

    def __enter__(self):
        self.start()
        return self
    
    def __exit__(self, xc_type, exc_value, exc_tb):
        self.stop()

