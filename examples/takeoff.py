from droneinterface.vehicle import Vehicle
import logging
from pymavlink import mavutil
from time import time, sleep
from flightanalysis import State
from flightplotting import plotsec

logging.basicConfig(level=logging.DEBUG)


veh = Vehicle.connect('tcp:127.0.0.1:5760', 1)

logging.info(veh.get_state())


veh.set_mode(13)

veh.arm()




#
#start = time()
#state = veh.get_state()
#while time() < start + 20:
#    state = state.append(veh.get_state(), "t")
#    sleep(0.1)
#
#
#plotsec(state, nmodels=5).show()
pass
#with veh.subscribe(list(veh.get_state.wrappers.values())) as observer:
#    pass


