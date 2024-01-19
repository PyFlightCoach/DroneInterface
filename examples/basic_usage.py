from droneinterface.vehicle import Vehicle
from droneinterface.messages import mavlink
from time import sleep, time
import logging
from json import dumps
from flightplotting import plotsec

logging.basicConfig(level=logging.DEBUG)

#setup the connection
vehicle = Vehicle.connect('tcp:127.0.0.1:5762', 1, 1, "log_tmp")


#getting a single mavlink message, wrapped in a pyflightcoach wrapper
logging.info(vehicle.get_GlobalOrigin(5, None).position)
logging.info(vehicle.get_GlobalPositionInt(5, None).position)

#getting a pyflightcoach state object (combines 3 mavlink messages):
logging.info(vehicle.get_state(2))

ids = [0, 26, 31, 32]


def print_rates(keys, duration=5, rate=2):
    end = time() + duration
    while time() < end:
        logging.info(vehicle.conn.rates(keys))
        sleep(1/rate)


#print_rates(ids)

##subscribing to a higer rate for some messages
st = vehicle.get_state()
with vehicle.subscribe(vehicle.state.ids, 10):
    while st.duration < 5:
        st = st.append(vehicle.last_state())
        print(st.duration)


plotsec(st).show()



pass