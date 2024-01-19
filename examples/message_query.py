from droneinterface.vehicle import Vehicle
from droneinterface.messages import mavlink
from time import sleep
from flightplotting import plotsec
import logging
import sys
from json import dumps
import traceback
import os
logging.basicConfig(level=logging.INFO)



vehicle = Vehicle.connect('tcp:127.0.0.1:5762', 1)


while True:
    print(dumps({k: v.last_message.__class__.__name__ for k, v in vehicle.msgs.items()}, indent=2))
    try:
        id = int(input("enter msg id\n"))
        os.system('cls||clear')
        msg = vehicle.get_message(id, 2, 5)
        if msg is None:
            print("couldn't get message")
            answer = input("load any message?")
            if answer in ["y", "Y"]:
                msg = vehicle.get_message(id, 2, None)

        if not msg is None:
            for k, v in msg.__dict__.items():
                print(k)
                print(v)
                print("")
            
    except Exception as e:
        print(e)
    input("continue?\n")
    os.system('cls||clear')