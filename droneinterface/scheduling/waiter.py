from threading import Thread
from functools import partial


class MessageWaiter(Thread):
    def __init__(self, target, *args, **kwargs) -> None:
        super().__init__(daemon=True)
        self.target = partial(target, *args, **kwargs)
        self.result = None
        self.start()

    def run(self):
        self.result = self.target()
