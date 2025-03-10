# Example MAVLink command to arm the drone
from pymavlink import mavutil

master = mavutil.mavlink_connection(
    '/dev/serial/by-id/usb-Holybro_Pixhawk6C_1E0030001151333036383238-if00',
    baud=115200
)  # Adjust to your connection
master.wait_heartbeat()

# Set mode to GUIDED or OFFBOARD (as needed)
mode = 'GUIDED'  # Change to 'OFFBOARD' for PX4
mode_id = master.mode_mapping()[mode]
master.set_mode(mode_id)

# Arm the vehicle
master.arducopter_arm()

print("Vehicle armed!")