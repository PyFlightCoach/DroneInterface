from droneinterface import Vehicle, enable_logging, logger
from time import sleep, time

enable_logging('DEBUG')

#setup the connection
vehicle = Vehicle.connect('tcp:127.0.0.1:5762', 1, 1, "log_tmp")

vehicle.next_heartbeat(1).__repr__()

#getting a single mavlink message, wrapped in a pyflightcoach wrapper
logger.info(vehicle.get_GlobalOrigin(5, None).position)
logger.info(vehicle.get_GlobalPositionInt(5, None).position)

#getting a pyflightcoach state object (combines 3 mavlink messages):
logger.info(vehicle.get_state(2).data)

logger.info(vehicle.state.ids)


##subscribing to a higer rate for some messages
with vehicle.subscribe(vehicle.state.ids, 10):
    print(vehicle.next_state().timestamp)



pass