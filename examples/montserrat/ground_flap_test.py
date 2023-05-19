
import logging
logging.basicConfig(level=logging.INFO)


import titan

vehicle = titan.titan("flight_3_ground_test", sim=False)

repeater = None
while True:
    
    try:
        pwm = int(input("enter flap position\n"))
        if not repeater is None:
            repeater.stop()
        repeater = vehicle.set_flap(pwm, "inbd")
        
    except Exception as ex:

        logging.exception(ex)
