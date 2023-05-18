from droneinterface.vehicle import Vehicle, Repeater
from droneinterface.messages import RCOverride
from pathlib import Path
import numpy as np
import pandas as pd
from math import floor
import plotly.express as px
from multiprocessing import Process
#sim_vehicle.py -L Montserrat --console --map

#vehicle = Vehicle.connect('udp:0.0.0.0:14550', 4, outdir = Path("log_tmp"))

def set_flap(vehicle: Vehicle, pwm, location="inbd") -> Repeater:    
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


Vehicle.set_flap = set_flap

def titan(flightname, sim=False) -> Vehicle: 
    outd = Path(f"log_tmp/{flightname}")
    outd.mkdir(exist_ok=True)
    return Vehicle.connect(
        'tcp:0.0.0.0:5763' if sim else 'udp:0.0.0.0:14550', 
        1 if sim else 4, 
        outdir = outd, 
        store_messages="all"
    )
    


