from droneinterface.vehicle import Vehicle
from droneinterface.messages import mavlink
from time import sleep, time
import logging
from json import dumps
logging.basicConfig(level=logging.INFO)

#setup the connection
vehicle = Vehicle.connect('tcp:127.0.0.1:5760', 1, 1, "log_tmp")

#getting a single mavlink message, wrapped in a pyflightcoach wrapper
logging.info(vehicle.get_GlobalOrigin(5, None).position)

#getting a pyflightcoach state object (combines 3 mavlink messages):
logging.info(vehicle.get_state())

msgkeys = ["1_1_0", "1_1_26", "1_1_31", "1_1_32"]


def print_rates(keys, duration=5, rate=2):
    end = time() + duration
    while time() < end:
        logging.info(vehicle.conn.rates(keys))
        sleep(1/rate)


print_rates(msgkeys)

pass
#
##subscribing to a higer rate for some messages
observer =  vehicle.subscribe(vehicle.state.ids, 10)


observer.start()

logging.info("started observer")

print_rates(msgkeys)


observer.stop()

logging.info("stopped observer")
#
print_rates(msgkeys)




pass