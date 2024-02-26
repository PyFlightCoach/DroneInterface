from ..import logger


class RateHistory:
    def __init__(self, veh, id, desired_rate):
        self.id = id
        self._last_message = veh._get_message(id, 2.0, None)
        self.initial_rate = max(self.current_rate(), 1.0)
        self.desired_rate = desired_rate
                
    def current_rate(self):
        if self._last_message is None: 
            return 0
        else:
            return self._last_message.rate

    def set_rate(self, veh):
        rate = self.current_rate()
        if rate < self.desired_rate:
            logger.debug(f"increasing rate for msg {self.id} from {rate} to {self.desired_rate}")
            veh.set_message_rate(self.id, self.desired_rate * 1.5)

    def reset_rate(self, veh):
        veh.set_message_rate(self.id, self.initial_rate)
