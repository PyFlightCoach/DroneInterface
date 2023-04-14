from droneinterface.vehicle import Vehicle
from droneinterface.messages import mavlink
from time import sleep
from flightplotting import plotsec
import logging

logging.basicConfig(level=logging.INFO)


vehicle = Vehicle.connect('tcp:127.0.0.1:5760', 1)

observer = vehicle.subscribe(
    list(mavlink.mavlink_map.keys()), 
    set_rates=False, 
    cleanup=False, 
    wait=False, 
)


while True:
    logging.info(observer.next_message)


