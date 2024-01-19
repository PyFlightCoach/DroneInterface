from droneinterface.vehicle import Vehicle
from droneinterface.messages import mavlink
from time import sleep
from flightplotting import plotsec
import logging
import sys

logging.basicConfig(level=logging.INFO)

#setup the connection
vehicle = Vehicle.connect('tcp:127.0.0.1:5762', 1)


#vehicle.set_mode(mavlink.PLANE_MODE_AUTO)
#vehicle.arm()


with vehicle.subscribe([mavlink.MAVLINK_MSG_ID_BATTERY_STATUS], 10):
    while True:
        bs = vehicle.next_batterystatus()
        if not bs is None:
            sys.stdout.write(f"\rCurrent = {bs.current} A, Voltage = {bs.voltage} V, Power = {bs.watts} W")
        sys.stdout.flush()
