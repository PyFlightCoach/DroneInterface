from droneinterface import Vehicle, enable_logging, mavlink, Timeout

from time import time
import numpy as np
import pandas as pd


def record_message(vehicle, id, duration):
    msgs = []
    start = time()
    while time() < start + duration:
        try:
            msgs.append(vehicle.next_message(id, timeout=duration))
        except Timeout:
            pass
    return msgs

enable_logging('DEBUG')
#setup the connection
vehicle = Vehicle.connect('tcp:127.0.0.1:5762', 1, 1, "log_tmp")


id = mavlink.MAVLINK_MSG_ID_SCALED_IMU
msgs={}


msgs['group_a'] = record_message(vehicle, id, 5)
vehicle.set_message_rate(id, 30)
msgs['group_b'] = record_message(vehicle, id, 5)
vehicle.set_message_rate(id, 10)
msgs['group_c'] = record_message(vehicle, id, 5)
vehicle.set_message_rate(id, 1)
msgs['group_d'] = record_message(vehicle, id, 5)

results = []
for k, msg in msgs.items():
    arr = np.array([m.timestamp for m in msg])
    if len(arr) >= 3:
        arr = arr - arr[0]
        period = np.diff(arr)
        results.append(dict(
            group = k,
            length = len(arr),
            mean = period.mean(),
            min = period.max(),
            max = period.min(),
            std=period.std(),
        ))

df = pd.DataFrame(results)
print(df)
print(1 / df.iloc[:,2:])
pass