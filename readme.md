This project will contain an opinionated but tidy way to talk to a Drone through PyMavLink 
using the PyFlightCoach Libraries


#### Setup the connection:
```sh
from droneinterface import Vehicle
vehicle = Vehicle.connect('tcp:127.0.0.1:5762')
```

#### Get a single mavlink message wrapped in a pyflightcoach wrapper:
```sh
>>> vehicle.get_GlobalPositionInt()
GlobalPositionInt(
    position=GPS(lat_=51.42 long_=-2.67 alt_=117.33, len=1),
    agl=0.055,
    velocity=Point(x_=-0.02 y_=-0.01 z_=0.0, len=1),
    heading=13.95565269894666,
    time_boot_ms=1840783
)
```

#### wait up to 1 second for a message:
```sh
>>> vehicle.next_heartbeat(1).type
MAV_TYPE(1=['mav_type_generic', 'mav_type_fixed_wing'])
```

#### Don't wait long enough:
```sh
>>> vehicle.next_heartbeat(1e-6)
droneinterface.scheduling.exceptions.Timeout: timeout after 1e-06 seconds waiting for message 0
```

#### Getting a pyflightcoach state object (combines 3 mavlink messages):
```sh
>>> vehicle.get_state().data
            x         y     z        rw        rx        ry        rz             t        dt  ...         p         q        r        du        dv          dw   dp   dq   dr
0.0 -0.006941 -0.022264  0.01 -0.012416  0.989064  0.080289  0.123095  1.709667e+09  0.033333  ... -0.006507 -0.022492  0.00954 -0.567203 -1.919822  997.321415  0.0  0.0  0.0

[1 rows x 21 columns]
>>> 
```

#### subscribing to a higher messaging rate:
```sh
with vehicle.subscribe(vehicle.state.ids, 10):
    print(vehicle.next_state().data)
```


