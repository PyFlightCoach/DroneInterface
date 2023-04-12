from droneinterface.vehicle import Vehicle
from droneinterface.messages import mavlink
from time import sleep
import logging
logging.basicConfig(level=logging.INFO)

#setup the vehicleection
vehicle = Vehicle.connect('tcp:127.0.0.1:5760', 1)

#getting a single mavlink message, wrapped in a pyflightcoach wrapper
print(vehicle.HomePosition.home)

#getting a pyflightcoach state object (combines 3 mavlink messages):
print(vehicle.get_state())

#mavlink message subscription, handling the context manually
observer = vehicle.subscribe([
    mavlink.MAVLINK_MSG_ID_ATTITUDE_QUATERNION,
    mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
    mavlink.MAVLINK_MSG_ID_SCALED_IMU
])

for i in range(10):
    for id in observer.ids:
        logging.info(observer[id])
    sleep(0.2)
    
observer.stop()

#mavlink message subscription using a with statement
with vehicle.subscribe(vehicle.get_state.ids) as observer:
    for i in range(10):
        logging.info(observer.get_state())
        sleep(0.2)
