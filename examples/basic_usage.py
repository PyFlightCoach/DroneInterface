from droneinterface.vehicle import Vehicle
from droneinterface.messages import mavlink
from time import sleep, time
import logging
logging.basicConfig(level=logging.DEBUG)

#setup the connection
vehicle = Vehicle.connect('tcp:127.0.0.1:5760', 1, 1, "log_tmp")

#getting a single mavlink message, wrapped in a pyflightcoach wrapper
logging.info(vehicle.get_HomePosition().home)

#getting a pyflightcoach state object (combines 3 mavlink messages):
logging.info(vehicle.get_state())


#subscribing to a higer rate for some messages

end = time() + 10
with vehicle.subscribe(vehicle.state.ids, 10):
    while time() < end:
        pass


