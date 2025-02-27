import time
import sys
from pymavlink import mavutil

master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
master.wait_heartbeat()
print("HEARTBEAT_DONE")
 
master.param_fetch_all()
 
while True:
            m = master.recv_match(type = "PARAM_VALUES", blocking = True, timeout = 1)
            print(m)
            if m is None:
                        break