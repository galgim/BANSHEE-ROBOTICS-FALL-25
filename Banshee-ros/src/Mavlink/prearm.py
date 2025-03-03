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

# ✅ Check for Pre-Arm Errors
def check_prearm_errors():
    print("🔍 Checking for pre-arm failures...")
    while True:
        msg = master.recv_match(type='STATUSTEXT', blocking=True)
        if msg:
            print(f"⚠ {msg.text}")
            if "PreArm" in msg.text:
                return msg.text  # Return the pre-arm error message
        time.sleep(0.5)

# ✅ Set GUIDED Mode
master.set_mode("GUIDED")
time.sleep(2)

msg = master.recv_match(type='HEARTBEAT', blocking=True)
mode = mavutil.mode_string_v10(msg)
print(f"🎮 Current Flight Mode: {mode}")

if mode != "GUIDED":
    print("❌ Failed to switch to GUIDED mode! Check flight controller settings.")
    master.close()
    exit()

# ✅ Disable RC Check (Allows arming without an RC controller)
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_PARAMETER, 0,
    301, 1, 0, 0, 0, 0, 0  # 301 = RC_OPTIONS, set to 1 (Ignore RC checks)
)
print("🎮 RC requirement skipped!")

# ✅ Disable GPS Requirement (If flying without GPS)
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_PARAMETER, 0,
    511, 0, 0, 0, 0, 0, 0  # 511 = GPS_CHECK, set to 0 to disable GPS requirement
)
print("📡 GPS Check Disabled!")

# ✅ Disable Safety Switch (If Needed)
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_PARAMETER, 0,
    220, 0, 0, 0, 0, 0, 0  # 220 = BRD_SAFETYENABLE, set to 0
)
print("🔓 Safety Disabled!")

# ✅ Check Battery Voltage
msg = master.recv_match(type='SYS_STATUS', blocking=True)
voltage = msg.voltage_battery / 1000.0
print(f"🔋 Battery Voltage: {voltage}V")

if voltage < 10.5:
    print("⚠️ WARNING: Low Battery! Charge before flying.")
    master.close()
    exit()

# ✅ Attempt to Arm
print("🚀 Attempting to arm the drone...")
master.arducopter_arm()
time.sleep(2)

# ✅ Verify Arming
msg = master.recv_match(type='HEARTBEAT', blocking=True)
armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED

if armed:
    print("✅ Drone Armed Successfully!")
else:
    print("❌ Failed to arm. Checking pre-arm errors...")
    prearm_message = check_prearm_errors()
    print(f"🛑 Pre-Arm Failure Reason: {prearm_message}")

# ✅ Close connection
master.close()
print("✅ Done!")
