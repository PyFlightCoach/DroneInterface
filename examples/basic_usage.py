from droneinterface import Vehicle, enable_logging, Timeout, logger
from json import dumps

enable_logging('DEBUG')

#setup the connection
ve = Vehicle.connect('tcp:127.0.0.1:5762')


print(ve.get_parameter('SCR_ENABLE'))


ve.request_parameters()


print(dumps(ve.parameters, indent=2))