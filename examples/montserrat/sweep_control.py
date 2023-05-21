import logging
from geometry import PZ, Point
import numpy as np
import pandas as pd
from typing import List
import titan
from math import floor
from droneinterface.vehicle import Vehicle, Repeater
from pathlib import Path
from flightdata import Flight

class ControlSweep:
    #TODO replace this with an optimiser
    #TODO better to run this in a thread
    def __init__(self, command, positions, duration, outdir : Path,  sfac=0.2):
        self.command = command
        self.repeater: Repeater = None
        self.positions = positions
        self.data = {p: [] for p in positions}
        self.duration = duration / len(self.positions)
        self.outdir = outdir
        if not self.outdir is None:
            self.outdir.mkdir(exist_ok=True)
        self.settle = self.duration * sfac
        self.lr = -1
        self.started = 0

    def get_cr(self, t):
        if self.started == 0:
            self.started = t
        return floor((t - self.started) / self.duration)

    def is_finished(self, cr):
        return cr >= len(self.data)

    def stop(self):
        if not self.repeater is None:
            self.repeater.stop()
            self.repeater = None

    def update(self, *args, **kwargs):
        cr = self.get_cr(st.t[0])

        if self.is_finished(cr):
            if not self.repeater is None:
                self.stop()
                
                df = self.summary()
                print(df)
                if not self.outdir is None:
                    df.to_csv(self.outdir / "summary.csv")
            return None

        if cr > self.lr:
            self.stop()
            self.save_record(self.lr)

            logging.info(f"Setting Control to {self.positions[cr]}")
            self.repeater = self.command(list(self.data.keys())[cr])
            self.lr = cr

        if st.t[0] > self.settle:
            self.data[list(self.data.keys())[cr]].append(self.read_data(*args, **kwargs))

    def read_data(self, st, bs, w, hud, rc):
        return {
            "timestamp": st.t[0],
            "current": bs.current,
            "voltage": bs.voltage,
            "ground_speed": hud.groundspeed,
            "airspeed": hud.airspeed,
            "wind_direction": w.direction,
            "wind_speed": w.speed,
            "servo_9": rc.servo9_raw,
            "servo_10": rc.servo9_raw
        }

    @staticmethod
    def summary_from_flight(fl: Flight):        
        return pd.DataFrame({
            "timestamp": fl.time_flight,
            "current": fl.current_0,
            "voltage": fl.battery_0,
            "ground_speed": abs(Point(fl.data.loc[:,["velocity_x", "velocity_y", "velocity_z"]])),
            "airspeed": fl.airspeed_0,
            "wind_direction": np.degrees(np.arctan2(fl.data.wind_x, fl.data.wind_y)),
            "wind_speed": abs(Point(fl.data.wind_x, fl.data.wind_y, np.zeros(len(fl.data)))),
            "servo_9": fl.data.servos_8,
            "servo_10": fl.data.servos_9
        })

    def save_record(self, r_id):
        if r_id >= 0 and r_id < len(self.positions) and not self.outdir is None:
            self.df(r_id).to_csv(self.outdir / f"{self.positions[r_id]}.csv")

    def df(self, r_id) -> pd.DataFrame:
        return pd.DataFrame(list(self.data.values())[r_id])

    def dfs(self):
        return {k: pd.DataFrame(v) for k, v in self.data.items()}

    def summary(self):
        dfs = self.dfs()
        df =  pd.DataFrame({k: v.mean() for k, v in dfs.items()}).T
        df["first_timestamp"] = {k:v.timestamp.iloc[0] for k, v in dfs.items()}
        df["last_timestamp"] = {k:v.timestamp.iloc[-1] for k, v in dfs.items()}
        return df


if __name__ == "__main__":

    logging.basicConfig(level=logging.INFO)

    vehicle = titan.titan("flight_6", sim=False)

    
    last_wp = 0
    new_record = False
    records: List[ControlSweep] = []

    last_flap = 0
    with vehicle.subscribe(
        vehicle.state.ids + [168, 147, 74, 36, 65], 
        10
    ):
        while True:

            if not vehicle.last_heartbeat().mode == "AUTO":
                continue

            new_flap = vehicle.last_SERVOOUTPUTRAW().servo9_raw
            if not new_flap == last_flap:
                last_flap = new_flap
                logging.info(f"new servo 9 position = {last_flap}")

            next_wp = vehicle.last_missionCurrent().seq
            if not next_wp == last_wp or last_wp == 0:
                logging.info(f"next wp {next_wp}")
                target = vehicle.next_PositionTargetGlobal(None)
                target_point = vehicle.box.gps_to_point(target.position) + PZ(target.alt) 
                last_wp = next_wp
                new_record = True

                if len(records) > 0:
                    records[-1].stop()
                    
            st = vehicle.last_state()
            path = target_point - st.pos 
            
            angle = Point.angle_between(
                path, 
                st.att.inverse().transform_point(st.vel) # world frame velocity vector
            )[0]

            if np.degrees(angle) < 8:# and abs((st.pos.z[0] - target.alt)) < 10:
                if new_record:
                    new_record = False
                    time_to_next_wp = abs(path)[0] / abs(st.vel)[0]
                    logging.info(f"next wp distance = {abs(path)[0]}, vel = {abs(st.vel)[0]}, time = {time_to_next_wp}")
                    if time_to_next_wp > 60:

                        records.append(ControlSweep(
                            command = lambda pwm: vehicle.set_flap(pwm, "inbd"),
                            positions=list(np.linspace(1200, 1800, 6).astype(int)),
                            duration=time_to_next_wp-20,
                            outdir=vehicle.conn.outdir / f"sweep_{len(records)}"
                        ))
                        
                        logging.info(f"start recording {len(records)}")
                    
                if len(records) > 0:    
                    records[-1].update(
                        st, 
                        vehicle.last_BatteryStatus(), 
                        vehicle.last_wind(),
                        vehicle.last_vfrhud(),
                        vehicle.last_SERVOOUTPUTRAW()
                    )

