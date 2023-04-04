from droneinterface.vehicle import Vehicle
from droneinterface.messages import mavlink
from time import sleep
import logging
logging.basicConfig(level=logging.DEBUG)

#setup the connection
conn = Vehicle.connect('tcp:127.0.0.1:5760', 1)


#mavlink message subscription using a with statement
with conn.subscribe(conn.get_state.ids) as observer:
    for i in range(10):
        try:
            logging.info(observer.get_state())
        except Exception as ex:
            print(ex)
        sleep(0.2)


#mavlink message subscription, handling the context manually
msgs = [
    mavlink.MAVLINK_MSG_ID_ATTITUDE_QUATERNION,
    mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
    mavlink.MAVLINK_MSG_ID_SCALED_IMU
]

observer = conn.subscribe(msgs)

for i in range(10):
    for m in msgs:
        logging.info(observer[m])
    sleep(0.2)
    
observer.stop()

