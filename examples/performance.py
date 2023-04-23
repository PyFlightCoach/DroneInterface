from droneinterface.vehicle import Vehicle
from droneinterface.messages import mavlink
from time import sleep
from flightplotting import plotsec
import logging

logging.basicConfig(level=logging.INFO)

#setup the connection
vehicle = Vehicle.connect('tcp:127.0.0.1:5760', 1, input=False)


vehicle.set_mode(mavlink.PLANE_MODE_TAKEOFF)
vehicle.arm()


with vehicle.subscribe([147]) as obs:
    while True:
        print(obs.current)
