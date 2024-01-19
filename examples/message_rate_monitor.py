from droneinterface.vehicle import Vehicle
from droneinterface.messages import mavlink
from time import sleep, time
from json import dumps
import sys
import os

#setup the connection
vehicle = Vehicle.connect('tcp:127.0.0.1:5762', 1, 1, "log_tmp")



def print_rates():
    print(dumps({m.last_message.__class__.__name__: m.rate for m in vehicle.conn.msgs[1].values()}, indent=2))
    

while True:
    print_rates()
    sleep(1)
    os.system('cls||clear')
