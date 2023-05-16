
import logging
logging.basicConfig(level=logging.INFO)


import titan


repeater = None
while True:
    
    try:
        pwm = int(input("enter flap position\n"))
        if not repeater is None:
            repeater.stop()
        repeater = titan.set_flap(pwm, "otbd")
        
    except Exception as ex:

        logging.exception(ex)
