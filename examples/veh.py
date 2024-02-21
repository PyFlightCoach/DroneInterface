from droneinterface import enable_logging
from droneinterface.vehicle import Vehicle

enable_logging('DEBUG')

vehicle = Vehicle.connect('tcp:127.0.0.1:5762', 1, input=False, timeout=1, retries=20)

while not vehicle.get_SysStatus().sensor_health.mav_sys_status_prearm_check:
    pass

print('arming')
vehicle.arm()
#while vehicle.conn.is_alive():
#    hb = vehicle.next_heartbeat(2)
#    print((datetime.now() - datetime.fromtimestamp(hb.timestamp)).microseconds / 1000000)
#    sleep(2)