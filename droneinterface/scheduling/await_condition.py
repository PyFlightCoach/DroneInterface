"""Call a function repeatedly and become true when the function returns true"""
from threading import Thread
from .. import logger


class AwaitCondition(Thread):
    def __init__(self, fun) -> None:
        super().__init__(daemon=True)
        self.fun = fun
        self.result = False
        self.start()

    def run(self):
        while not self.result and not self._is_stopped:
            try:
                self.result = self.fun()
                if self.result:
                    logger.debug('Await Condition Met!')
            except Exception as ex:
                logger.debug(ex)

    def stop(self):
        self._is_stopped = True
        self.join()
