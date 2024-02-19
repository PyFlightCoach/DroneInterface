import titan
import logger
import sys

if __name__ == '__main__':
    logger.basicConfig(level=logger.INFO)

    vehicle = titan.titan("flight_5_sim", sim=True)
    last_flap = 0

    with vehicle.subscribe([36], 10):
        cols=[f" {i+1:02d} " for i in range(10)]
        print(",".join(cols))
        while True:
            new_flap = vehicle.last_SERVOOUTPUTRAW()
            servos = [getattr(new_flap, f'servo{i+1}_raw') for i in range(10)]
            servos = ",".join([f"{s:04d}" for s in servos ])
            sys.stdout.write(f"\r{servos}")
            sys.stdout.flush()
