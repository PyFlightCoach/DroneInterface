from droneinterface.vehicle import Vehicle
from droneinterface.messages import mavlink, RCOverride
from time import time
from flightplotting import plotsec
import logging
import sys
from geometry import GPS, PZ, Point
import sys
from pathlib import Path
import numpy as np
import pandas as pd
from flightanalysis import State
from typing import List

logging.basicConfig(level=logging.INFO)

#sim_vehicle.py -L Montserrat --console --map
vehicle = Vehicle.connect('tcp:127.0.0.1:5763', 1, outdir = Path("log_tmp"))


class FlapRecording:
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


def set_flap(pwm):
    vehicle.send_message(RCOverride.set_channel(
        vehicle.sysid, vehicle.compid,
        8, pwm
    ))


if __name__ == "__main__":
    last_wp = 0
    records: List[FlapRecording] = []
    newrec = False

    with vehicle.subscribe(
        vehicle.state.ids + [168, 147, 74], 
        10
    ):
        while True:
            if not vehicle.conn.msgs[1][0].wrapper().mode == "AUTO":
                continue

            next_wp = vehicle.last_missionCurrent().seq
            if next_wp > last_wp or last_wp == 0:
                last_wp = next_wp
                target = vehicle.next_PositionTargetGlobal(None)
                target_point = vehicle.box.gps_to_point(target.position) + PZ(target.alt) 
                logging.info(f"reached wp {last_wp}")
                newrec = True           

            st = vehicle.last_state()
            path = target_point - st.pos 
            
            angle = Point.angle_between(
                Point(1,1,0) * path, 
                Point(1,1,0) * st.att.inverse().transform_point(st.vel) # world frame velocity vector
            )[0]

            if np.degrees(angle) < 2 and abs((st.pos.z[0] - target.alt)) < 10:
                if newrec:
                    newrec=False
                    time_to_next_wp = abs(path)[0] / abs(st.vel)[0]

                    if time_to_next_wp > 60:
                        if len(records) > 0:
                            logging.info(f"finished recording {len(records)}")
                            print({k: df.mean() for k, df in records[-1].dfs().items()})

                        records.append(FlapRecording(
                            command = set_flap,
                            positions=[1300, 1500, 1700],
                            duration=(time_to_next_wp-10)/3
                        ))

                        logging.info(f"start recording {len(records)}, duration={time_to_next_wp}")



                if len(records) > 0:    
                    records[-1].update(st, vehicle.last_BatteryStatus())


#RC_CHANNELS_OVERRIDE
#MAV_CMD_DO_CHANGE_SPEED
