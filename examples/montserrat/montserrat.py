from droneinterface.vehicle import Vehicle
from droneinterface.messages import mavlink
from time import time
from flightplotting import plotsec
import logging
import sys
from geometry import GPS, PZ, Point
import sys
from pathlib import Path
import numpy as np
import pandas as pd
from threading import Thread
import os
import plotly.express as px

logging.basicConfig(level=logging.INFO)

#sim_vehicle.py -L Montserrat --console --map --out 127.0.0.1:14550
vehicle = Vehicle.connect('tcp:127.0.0.1:5763', 1, outdir = Path("log_tmp"))
    
last_wp = 0


with vehicle.subscribe(
    vehicle.state.ids + [168, 147, 74], 
    10
):
    while True:
        #check it is in Auto
        last = time()
        next_wp = vehicle.last_missionCurrent().seq
        if next_wp > last_wp or last_wp == 0:
            last_wp = next_wp
            target = vehicle.next_PositionTargetGlobal(None)
            target_point = vehicle.box.gps_to_point(target.position) + PZ(target.alt) 
            logging.info(f"reached wp {last_wp}")

        st = vehicle.last_state()
        
        angle = Point.angle_between(
            Point(1,1,0) * (target_point - st.pos), # vector pointing to the next waypoint
            Point(1,1,0) * st.att.inverse().transform_point(st.vel) # velocity vector
        )[0]

        if np.degrees(angle) < 2 and abs((st.pos.z[0] - target.alt)) < 10:
            #do the optimization
            pass


#RC_CHANNELS_OVERRIDE
#MAV_CMD_DO_CHANGE_SPEED
