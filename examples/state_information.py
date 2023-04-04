
from droneinterface.vehicle import Vehicle
import logging
logging.basicConfig(level=logging.DEBUG)

try:
    veh = Vehicle.connect('tcp:127.0.0.1:5760', 1)

    logging.info(veh.get_state())
except Exception as ex:
    print(ex)




