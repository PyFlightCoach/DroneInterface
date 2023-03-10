from pytest import fixture
from droneinterface.connection import Connection
from droneinterface.messages import mavlink
from droneinterface.messages.message_wrappers import wrappers
from geometry import GPS
from geometry.testing import assert_almost_equal
from time import time, sleep

conn = Connection.connect('tcp:127.0.0.1:5760', 1)
msgs = [31, 32, 33]
observer = conn.subscribe(msgs)

end = time() + 10
while time() < end:
    for m in msgs:
        try:
            print(observer.data[m].__dict__)
        except Exception as ex:
            print(ex)
    sleep(1)
    
observer.stop()

