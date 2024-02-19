from droneinterface.vehicle import Vehicle

from datetime import datetime



#setup the connection
vehicle = Vehicle.connect('tcp:127.0.0.1:5762', 1, input=False, timeout=1)



while vehicle.conn.is_alive():
    hb = vehicle.next_heartbeat(2)
    print((datetime.now() - datetime.fromtimestamp(hb.timestamp)).microseconds / 1000000)