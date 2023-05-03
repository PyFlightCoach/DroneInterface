from droneinterface.vehicle import Vehicle
from droneinterface.messages import mavlink
from time import sleep
import logging
logging.basicConfig(level=logging.DEBUG)

#setup the connection
vehicle = Vehicle.connect('tcp:127.0.0.1:5760', 1, 1, "log_tmp")

#getting a single mavlink message, wrapped in a pyflightcoach wrapper
print(vehicle.get_HomePosition().home)

#getting a pyflightcoach state object (combines 3 mavlink messages):
print(vehicle.get_state())

