from .messages import mavlink
import logging


class Base:
    def _msg(self, msg):
        return f"{str(self).ljust(80)[:80]}: {msg}"
        