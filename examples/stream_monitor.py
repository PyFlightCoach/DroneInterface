from droneinterface.vehicle import Vehicle
from droneinterface.messages import mavlink
from time import sleep
from flightplotting import plotsec
import logging
from time import time

logging.basicConfig(level=logging.INFO)


vehicle = Vehicle.connect('tcp:127.0.0.1:5760', 1)


vehicle.set_mode(mavlink.PLANE_MODE_TAKEOFF)
vehicle.arm()

with vehicle.subscribe(vehicle.get_state.ids) as ob:
    while True:
        st = ob.get_state()

        print(st.pos)
        if st.pos.z[0] >20:
            break
        sleep(0.5)



observer = vehicle.subscribe(
    list(mavlink.mavlink_map.keys()), 
    set_rates=False, 
    cleanup=False, 
    wait=False, 
)


end = time() + 10


msgs = {}

while time() < end:
    msg = observer.next_message
    if not msg.id in msgs:
        msgs[msg.id] = 1
    else:
        msgs[msg.id] += 1
    
print(msgs)

pass

