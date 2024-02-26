
from threading import Thread
from .rate_history import RateHistory


class Observer(Thread):
    def __init__(self, veh, ids: list[int], rate: int) -> None:
        super().__init__(daemon=True)
        self.veh = veh
        self.base_rates = {id: RateHistory(self.veh, id, rate) for id in ids}
        
    def __getattr__(self, name):
        return getattr(self.veh, name)
    
    def run(self):
        while not self._is_stopped:
            for rh in self.base_rates.values():
                rh.set_rate(self.veh)

    def stop(self):
        self._is_stopped = True
        self.join()
        for rh in self.base_rates.values():
            rh.reset_rate(self.veh)

    def __enter__(self):
        self.start()
        return self
    
    def __exit__(self, xc_type, exc_value, exc_tb):
        self.stop()
