# Example MAVLink command to arm the drone
from pymavlink import mavutil

master = mavutil.mavlink_connection('udp:127.0.0.1:14550')  # Adjust to your connection
master.wait_heartbeat()

# Set mode to GUIDED or OFFBOARD (as needed)
mode = 'GUIDED'  # Change to 'OFFBOARD' for PX4
mode_id = master.mode_mapping()[mode]
master.set_mode(mode_id)

# Arm the vehicle
master.arducopter_arm()

print("Vehicle armed!")