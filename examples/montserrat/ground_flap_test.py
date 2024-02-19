
import logger
logger.basicConfig(level=logger.INFO)


import titan

vehicle = titan.titan("flight_12_ground_test", sim=False)

repeater = None
while True:
    
    try:
        pwm = int(input("enter flap position\n"))
        if not repeater is None:
            repeater.stop()
        repeater = vehicle.set_flap(pwm, "inbd")
        new_flap = vehicle.get_SERVOOUTPUTRAW(None, 0 )
        servos = [getattr(new_flap, f'servo{i+1}_raw') for i in range(10)]
        servos = ",".join([f"{s:04d}" for s in servos ])
        print(f"\r{servos}")
        
    except Exception as ex:

        logger.exception(ex)
