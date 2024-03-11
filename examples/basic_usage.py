from droneinterface import Vehicle, enable_logging, Timeout, logger


enable_logging('DEBUG')

#setup the connection
vehicle = Vehicle.connect('tcp:127.0.0.1:5762')


print(vehicle.get_parameter('SCR_ENABLE'))


vehicle.send_paramrequestlist()


while True:
    try:
        msg = vehicle.next_paramvalue(2)
    except Timeout:
        break
for k, v in vehicle.parameters.items():
    logger.info(f'{k}: {v}')

vehicle.set_parameter('SCRTSPD_kD', 0.0)

print(vehicle.get_parameter('SCRTSPD_kD'))