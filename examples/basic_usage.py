from droneinterface.vehicle import Vehicle
from time import sleep, time
from droneinterface import enable_logging
from loguru import logger
enable_logging('DEBUG')

#setup the connection
vehicle = Vehicle.connect('tcp:127.0.0.1:5762', 1, 1, "log_tmp")


vehicle.wait_for_test(lambda : vehicle.get_SysStatus().can_arm)

#getting a single mavlink message, wrapped in a pyflightcoach wrapper
logger.info(vehicle.get_GlobalOrigin(5, None).position)
logger.info(vehicle.get_GlobalPositionInt(5, None).position)

#getting a pyflightcoach state object (combines 3 mavlink messages):
logger.info(vehicle.get_state(2).data)

logger.info(vehicle.state.ids)

def print_rates(keys, duration=5, rate=2):
    end = time() + duration
    while time() < end:
        logger.info(vehicle.conn.rates(keys))
        sleep(1/rate)


print_rates(vehicle.state.ids)

##subscribing to a higer rate for some messages
with vehicle.subscribe(vehicle.state.ids, 10):
    print_rates(vehicle.state.ids)

print_rates(vehicle.state.ids)

pass