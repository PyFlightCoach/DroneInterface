from droneinterface.vehicle import Vehicle
from droneinterface.messages import mavlink
from time import sleep
from flightplotting import plotsec
import logging

logging.basicConfig(level=logging.INFO)

#setup the connection
vehicle = Vehicle.connect('tcp:127.0.0.1:5760', 1, input=False)


vehicle.set_mode(mavlink.PLANE_MODE_TAKEOFF)
vehicle.arm()

with vehicle.subscribe(vehicle.get_state.ids) as observer:
    st = observer.get_state()

    while st.z[-1] < 49:
        state = observer.get_state()
        if state.z[0] > 2:
            st = st.append(state)
        sleep(0.01)

plotsec(st, nmodels=50, scale=2).show()
