import logging
logging.basicConfig(level=logging.DEBUG)

from geometry import PZ, Point
import numpy as np
import pandas as pd
from typing import List
import titan



if __name__ == "__main__":
    last_wp = 0
    records: List[titan.Recording] = []
    newrec = False

    with titan.vehicle.subscribe(
        titan.vehicle.state.ids + [168, 147, 74], 
        10
    ):
        while True:
            if not titan.vehicle.last_heartbeat().mode == "AUTO":
                continue

            next_wp = titan.vehicle.last_missionCurrent().seq
            if next_wp > last_wp or last_wp == 0:
                last_wp = next_wp
                target = titan.vehicle.next_PositionTargetGlobal(None)
                target_point = titan.vehicle.box.gps_to_point(target.position) + PZ(target.alt) 
                logging.info(f"reached wp {last_wp}")
                newrec = True           

            st = titan.vehicle.last_state()
            path = target_point - st.pos 
            
            angle = Point.angle_between(
                path, 
                st.att.inverse().transform_point(st.vel) # world frame velocity vector
            )[0]

            if np.degrees(angle) < 2:# and abs((st.pos.z[0] - target.alt)) < 10:
                if newrec:
                    newrec=False
                    time_to_next_wp = abs(path)[0] / abs(st.vel)[0]

                    if time_to_next_wp > 60:
                        if len(records) > 0:
                            logging.info(f"finished recording {len(records)}")
                            print({k: df.mean() for k, df in records[-1].dfs().items()})

                        records.append(titan.Recording(
                            command = lambda pwm: titan.set_flap(pwm, "inbd"),
                            positions=[1300, 1500, 1700],
                            duration=(time_to_next_wp-10)/3
                        ))

                        logging.info(f"start recording {len(records)}, duration={time_to_next_wp}")

                if len(records) > 0:    
                    records[-1].update(st, titan.vehicle.last_BatteryStatus())

