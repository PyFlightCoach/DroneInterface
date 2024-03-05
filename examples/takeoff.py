from droneinterface.vehicle import Vehicle
from droneinterface.messages import mavlink
from time import time
from flightplotting import plotsec
import logger

logger.basicConfig(level=logger.INFO)

#setup the connection
vehicle = Vehicle.connect('tcp:127.0.0.1:5762')


vehicle.set_mode(mavlink.PLANE_MODE_TAKEOFF)
vehicle.arm()
end = time() + 20
with vehicle.subscribe(vehicle.state.ids, 10) as observer:
    st = vehicle.next_state(0.5)
    while st.z[-1] < 49:
        if st.z[0] > 2:#           
            st = st.append(st, "now")
        #sleep(0.01)
        print(st.z[-1])
        st = vehicle.next_state(0.5)
plotsec(st, nmodels=3, scale=2).show()
