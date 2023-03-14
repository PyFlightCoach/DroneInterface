from pytest import fixture
from droneinterface.connection import Connection
from droneinterface.messages import mavlink
from droneinterface.messages.wrappers import wrappers
from geometry import GPS
from geometry.testing import assert_almost_equal
from time import time, sleep

conn = Connection.connect('tcp:127.0.0.1:5760', 1)
msgs = [31, 32, 33]

with conn.subscribe(msgs) as observer:
    for i in range(10):
        for m in msgs:
            print(observer[m])
        sleep(1)



observer = conn.subscribe(msgs)

for i in range(10):
    for m in msgs:
        print(observer[m])
    sleep(1)
    
observer.stop()

#observer.stop()

