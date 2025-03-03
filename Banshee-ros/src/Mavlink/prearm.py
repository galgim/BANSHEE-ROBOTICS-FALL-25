import time
from pymavlink import mavutil

# Connect to Pixhawk
master = mavutil.mavlink_connection(
    '/dev/serial/by-id/usb-Holybro_Pixhawk6C_1E0030001151333036383238-if00',
    baud=115200
)

# Wait for heartbeat
master.wait_heartbeat()
print("✅ Heartbeat received!")

# Function to check pre-arm messages
def check_prearm_errors():
    print("🔍 Checking for pre-arm failures...")
    while True:
        msg = master.recv_match(type='STATUSTEXT', blocking=True)
        if msg:
            print(f"⚠ {msg.text}")
            if "PreArm" in msg.text:
                return msg.text  # Return the pre-arm error message

        time.sleep(0.5)  # Keep checking

# Try arming
master.arducopter_arm()
time.sleep(3)  # Wait for response

# Check if armed
msg = master.recv_match(type='HEARTBEAT', blocking=True)
armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED

if armed:
    print("🚀 Drone Armed Successfully!")
else:
    print("❌ Failed to arm. Checking pre-arm failures...")
    error_msg = check_prearm_errors()
    print(f"🛑 Pre-Arm Failure Reason: {error_msg}")

# Close connection
master.close()
print("✅ Done!")
