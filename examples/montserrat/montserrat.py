from droneinterface.vehicle import Vehicle
from droneinterface.messages import mavlink
from time import sleep
from flightplotting import plotsec
import logging
import sys
from json import dumps
import traceback
import os
from geometry import GPS, PZ
import sys
from pathlib import Path


logging.basicConfig(level=logging.INFO)

#sim_vehicle.py -L Montserrat --console --map --out 127.0.0.1:14550
vehicle = Vehicle.connect('tcp:127.0.0.1:5763', 1, outdir = Path("log_tmp"))


#while True:
    #logging.info("waiting to reach the next waypoint")
    #last_wp = vehicle.next_MissionItemReached(None)
    #logging.info(f"passed waypoint {last_wp.seq}")
    #
    #next_wp = vehicle.get_missionCurrent()
    #logging.info(f"next waypoint = {next_wp.seq}")
    #target = vehicle.get_PositionTargetGlobal(None, 0.0)
    #logging.info(f"next target: \n {str(target.position)}")
    
last_wp = 0
with vehicle.subscribe(
    vehicle.state.ids + [168, 147, 74], 
    10
):
    while True:
        next_wp = vehicle.last_missionCurrent().seq
        if next_wp > last_wp or last_wp == 0:
            last_wp = next_wp
            target = vehicle.get_PositionTargetGlobal(None, 0.0)
            logging.info(f"reached wp {last_wp}")
        st = vehicle.get_state()



        path = st.pos - (target.position - vehicle.origin) + PZ(target.alt)
        logging.info(f"path to next wp: {path}")
        



#MAVLink_mission_current_message
#seq 0

#"1_1_46": "MAVLink_mission_item_reached_message"
# seq 2

#"1_1_168": "MAVLink_wind_message"
#direction -11.248583793640137
#speed 1.3592932224273682
#speed_z 0.0

#RC_CHANNELS_OVERRIDE
#MAV_CMD_DO_CHANGE_SPEED


#"1_1_87": "MAVLink_position_target_global_int_message"
#time_boot_ms 2570360
#coordinate_frame 0
#type_mask 65016
#lat_int 167159379
#lon_int -622289689
#alt 100.0999984741211
#vx 0.0
#vy 0.0
#vz 0.0
#afx 0.0
#afy 0.0
#afz 0.0
#yaw 0.0
#yaw_rate 0.0