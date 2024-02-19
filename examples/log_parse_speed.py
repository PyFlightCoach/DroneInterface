from droneinterface import Connection
from pymavlink import mavutil
from pathlib import Path
import shutil
import sys
import logger

logger.basicConfig(level=logger.DEBUG)



conn = Connection(
        mavutil.mavlink_connection("examples/00000054.BIN"),
        store_messages=['XKF1', 'XKQ1', 'NKF1', 'NKQ1', 'NKF2', 'XKF2', 'ARSP', 'GPS', 'RCIN', 'RCOU', 'IMU', 'BARO', 'MODE', 'RPM', 'MAG', 'BAT', 'BAT2']
    )