from droneinterface.vehicle import Vehicle, Repeater
from droneinterface.messages import RCOverride
from pathlib import Path
import numpy as np
import pandas as pd

#sim_vehicle.py -L Montserrat --console --map

#vehicle = Vehicle.connect('udp:0.0.0.0:14550', 4, outdir = Path("log_tmp"))
vehicle = Vehicle.connect('tcp:0.0.0.0:5763', 1, outdir = Path("log_tmp"))

class Record:
    def __init__(self):
        pass


class Recording:
    #TODO replace this with an optimiser
    def __init__(self, command, positions=[1300, 1500, 1700], duration=10, settle=2):
        self.command = command
        self.data = {p: [] for p in positions}
        self.duration = duration
        self.settle = settle
        self.keys = ["timestamp", "current", "voltage", "ground_speed"]

        self.cr = -1
        self.started = 0

    def update(self, st, bs):
        if self.cr < len(self.data):
            if (st.t[0] - self.started ) > self.duration:
                self.cr +- 1
                self.command(list(self.data.keys())[self.cr])
                self.started = st.t[0]
            elif st.t[0] > self.settle:
                self.data[list(self.data.keys())[self.cr]].append([
                    st.t[0],
                    bs.current,
                    bs.voltage,
                    abs(st.vel)[0]
                ])

    def dfs(self):
        return {k: pd.DataFrame(v, columns=self.keys) for k, v in self.data.items()}



def set_flap(pwm, location="inbd") -> Repeater:    
    pwm = max(pwm, 1100)
    pwm = min(pwm, 1900)
    repeater = Repeater(
        lambda : vehicle.send_message(RCOverride.set_channel(
            vehicle.sysid, vehicle.compid,
            5 if location == "otbd" else 7, 
            pwm
        )),
        1
    )
    repeater.start()
    return repeater

