import time
from pymavlink import mavutil

# Connect to Pixhawk
master = mavutil.mavlink_connection(
    '/dev/serial/by-id/usb-Holybro_Pixhawk6C_1E0030001151333036383238-if00',
    baud=115200
)

# ✅ Wait for heartbeat
master.wait_heartbeat()
print("✅ Heartbeat received!")

# ✅ Set GUIDED mode
master.set_mode("GUIDED")
time.sleep(2)

# ✅ Disable RC Pre-Arm Check
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_PARAMETER, 0,
    301, 0, 0, 0, 0, 0, 0  # 301 = RC_OPTIONS, set to 0 (disable RC checks)
)
print("🎮 RC requirement disabled!")

# ✅ Disable ARMING_CHECK (Optional)
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_PARAMETER, 0,
    160, 0, 0, 0, 0, 0, 0  # 160 = ARMING_CHECK, set to 0 (disable all checks)
)
print("✅ ARMING_CHECK disabled (RC not required)")

# ✅ Try Arming
master.arducopter_arm()
time.sleep(2)

# ✅ Check if armed
msg = master.recv_match(type='HEARTBEAT', blocking=True)
armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED

if armed:
    print("🚀 Drone Armed Successfully!")
else:
    print("❌ Still failed to arm. Check for other pre-arm issues.")

# Close connection
master.close()
print("✅ Done!")
