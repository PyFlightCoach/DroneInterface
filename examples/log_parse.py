from droneinterface import Connection
from pymavlink import mavutil
from pathlib import Path
import shutil
import sys
import logging
from ardupilot_log_reader import Ardupilot
from time import time
logging.basicConfig(level=logging.DEBUG)


if __name__ == '__main__':
    s1 = time()
    df = Connection.parse_bin(
        "examples/00000054.BIN",
        ['XKF1', 'XKQ1', 'NKF1', 'NKQ1', 'NKF2', 'XKF2', 'ARSP', 'GPS', 'RCIN', 'RCOU', 'IMU', 'BARO', 'MODE', 'RPM', 'MAG', 'BAT', 'BAT2']
    )
    duration_1 = time() - s1

    s2 = time()
    parser = Ardupilot("examples/00000054.BIN", types=['XKF1', 'XKQ1', 'NKF1', 'NKQ1', 'NKF2', 'XKF2', 'ARSP', 'GPS', 'RCIN', 'RCOU', 'IMU', 'BARO', 'MODE', 'RPM', 'MAG', 'BAT', 'BAT2'])
    df = parser.full_df()
    duration_2 = time() - s2

    print(f"drone_interface = {duration_1}")  # 20 seconds
    print(f"ardupilot_log_reader = {duration_2}") # 5 seconds