from droneinterface.connection import Connection
from time import sleep
import logging
logging.basicConfig(level=logging.DEBUG)

#setup the connection
conn = Connection.connect('tcp:127.0.0.1:5760', 1)

#message ids to watch
msgs = [31, 32, 33]

#mavlink message subscription using a with statement
with conn.subscribe(msgs) as observer:
    for i in range(10):
        for m in msgs:
            print(observer[m])
        sleep(1)


#mavlink message subscription, handling the context manually
observer = conn.subscribe(msgs)

for i in range(10):
    for m in msgs:
        print(observer[m])
    sleep(1)
    
observer.stop()

