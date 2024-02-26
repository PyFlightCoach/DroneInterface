
#sh run_simulation.sh -p ./autotuner.py
from droneinterface import Vehicle, enable_logging, logger, AwaitCondition

enable_logging('DEBUG')

vehicle = Vehicle.connect('tcp:127.0.0.1:5762', 1, input=False, timeout=1, retries=20)

ac = AwaitCondition(lambda : vehicle.last_SysStatus().can_arm)
while vehicle.conn.is_alive() and not ac.result:
    pass
logger.info('can arm!')

#The above demonstrates the use of the AwaitCondition class to wait for a condition to be met,
#but to solve this problem the following is neater: 
vehicle.wait_for_test(lambda : vehicle.get_SysStatus().can_arm)

