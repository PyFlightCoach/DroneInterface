from droneinterface import Connection
from pymavlink import mavutil


conn = Connection(mavutil.mavlink_connection('tcp:127.0.0.1:5760'))
conn.start()
while True:
    print(conn.msgs)