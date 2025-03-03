import time
from pymavlink import mavutil

# ✅ Connect to Pixhawk
master = mavutil.mavlink_connection(
    '/dev/serial/by-id/usb-Holybro_Pixhawk6C_1E0030001151333036383238-if00',
    baud=115200
)

# ✅ Wait for heartbeat
master.wait_heartbeat()
print("✅ Heartbeat received!")

# ✅ Set to GUIDED mode
master.set_mode("GUIDED")
time.sleep(2)

# ✅ Arm the drone (required for motor test)
master.arducopter_arm()
time.sleep(3)

# ✅ Perform motor test (Motor 1, 20% throttle, for 2 seconds)
print("🛠 Testing Motor #1 at 20% power for 2 seconds...")
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST, 0,
    1,  # Motor instance (1 = first motor)
    0,  # Test type (0 = PWM, 1 = Percentage)
    20,  # Throttle (20%)
    2,  # Duration in seconds
    0, 0, 0  # Unused parameters
)

# ✅ Wait for motor test to complete
time.sleep(5)

# ✅ Disarm after test
master.arducopter_disarm()
print("🛑 Drone Disarmed!")

# ✅ Close connection
master.close()
print("✅ Done!")
