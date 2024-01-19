from droneinterface.vehicle import Vehicle
from droneinterface.messages import mavlink
from time import time
from flightplotting import plotsec
import logging
from flightdata import State
import numpy as np

logging.basicConfig(level=logging.INFO)

#setup the connection
vehicle = Vehicle.connect('tcp:127.0.0.1:5762', 1, input=False)


vehicle.set_mode(mavlink.PLANE_MODE_TAKEOFF)
vehicle.arm()
end = time() + 20
with vehicle.subscribe(vehicle.state.ids, 10) as observer:
    st = vehicle.last_state()

    while st.z[-1] < 49:
        state = vehicle.last_state()
        if state.z[0] > 2:#
            
            st = st.append(state, "now")
        #sleep(0.01)
        print(st.z[-1])
plotsec(st, nmodels=3, scale=2).show()
