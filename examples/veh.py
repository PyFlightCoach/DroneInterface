from droneinterface import enable_logging, Vehicle, mavlink, logger
import geometry as g

enable_logging('INFO')

vehicle = Vehicle.connect('tcp:127.0.0.1:5762', 1, input=False, timeout=1, retries=20)

vehicle.wait_for_test(lambda : vehicle.get_SysStatus().can_arm)

vehicle.arm()
vehicle.set_mode(mavlink.PLANE_MODE_TAKEOFF)

home: g.GPS = vehicle.get_HomePosition(None, None).home

vehicle.wait_for_test(lambda : (vehicle.next_GlobalPositionInt(None).position - home).z[0] < -49 )

logger.info('takeoff complete')

vehicle.set_mode(mavlink.PLANE_MODE_GUIDED)

ps = [
    [500, 500, -100],
    [500, -500, -150],
    [-500, -500, -200],
    [-500, 500, -250],
    [0, 0, -30],

]
for p in ps:
    vehicle.nav_waypoint(1, home.offset(g.Point(*p)))
    mi = vehicle.next_MissionItemReached(None)

