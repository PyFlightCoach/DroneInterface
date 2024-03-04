from droneinterface import Vehicle, enable_logging, mavlink, Watcher
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import pandas as pd

enable_logging('DEBUG')

#setup the connection
vehicle = Vehicle.connect('tcp:127.0.0.1:5762', 1, 1, "log_tmp")

vehicle.wait_for_test(lambda : vehicle.get_SysStatus().can_arm)

vehicle.arm()
vehicle.set_mode(mavlink.PLANE_MODE_AUTO)

with vehicle.subscribe(mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 5):
    watcher = Watcher(lambda : vehicle.next_custompidstate(None).data, 1000, None)


    figure= plt.figure()
    line, = plt.plot([0], [0], '-')
    plt.axis([0, 100, 15, 25])


    def animate(i):
        try:
            df = pd.DataFrame(watcher.data)
            line.set_data(list(watcher.times), df.iloc[:, 1])
        except Exception as ex:
            pass
        return line,


    ani = FuncAnimation(figure, animate, interval=10)

    plt.show()