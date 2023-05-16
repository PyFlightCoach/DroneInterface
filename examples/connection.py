from droneinterface import Connection
from pymavlink import mavutil
from time import sleep

conn = Connection(mavutil.mavlink_connection('udp:0.0.0.0:14550'))
conn.start()
while True:
    print(conn.msgs)
    sleep(3)