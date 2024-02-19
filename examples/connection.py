from droneinterface import Connection
from pymavlink import mavutil
from time import sleep

conn = Connection(mavutil.mavlink_connection('tcp:127.0.0.1:5762', retries=100))
conn.start()
while conn.is_alive():
    print(conn.msgs)
    sleep(3)