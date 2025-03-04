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

# 🔹 **Ensure GUIDED Mode**
def set_guided_mode():
    master.set_mode("GUIDED")
    time.sleep(2)
    msg = master.recv_match(type='HEARTBEAT', blocking=True)
    mode = mavutil.mode_string_v10(msg)
    print(f"🎮 Current Flight Mode: {mode}")
    if mode != "GUIDED":
        print("❌ Failed to set GUIDED mode! Check flight controller settings.")
    return

# 🔹 **Disable Safety Switch**
def disable_safety_switch():
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_PARAMETER, 0,
        220, 0, 0, 0, 0, 0, 0  # 220 = BRD_SAFETYENABLE, Set to 0
    )
    print("🔓 Safety Disabled!")

# 🔹 **Arm the Drone**
def arm_drone():
    master.arducopter_arm()
    time.sleep(2)
    msg = master.recv_match(type='HEARTBEAT', blocking=True)
    if not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
        print("❌ Drone failed to arm! Check pre-arm checks.")
    else:
        print("🚀 Drone Armed!")
    return

# 🔹 **Move Forward (without GPS)**
def move_forward(vx=1.0, duration=3):
    for _ in range(duration):
        master.mav.set_position_target_local_ned_send(
            0, master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            int(0b000011000000),  # Velocity only
            0, 0, 0,
            vx, 0, 0,  # Move Forward (North)
            0, 0, 0,
            0, 0
        )
        print(f"➡ Moving forward at {vx} m/s")
        time.sleep(1)
    print("✅ Forward movement complete!")

# 🔹 **Descend**
def descend(vz=-0.5, duration=5):
    for _ in range(duration):
        master.mav.set_position_target_local_ned_send(
            0, master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            int(0b000011000000),
            0, 0, 0,
            0, 0, vz,  # Descend velocity
            0, 0, 0,
            0, 0
        )
        print(f"🔽 Descending at {vz} m/s")
        time.sleep(1)
    print("✔ Descent complete!")

# --- Pre-Flight Sequence (without checks) ---
set_guided_mode()
disable_safety_switch()
arm_drone()

# ✅ **Execute Movement**
move_forward(vx=1.0, duration=3)  # Move forward
descend(vz=-0.5, duration=5)       # Then descend

# 🔹 **Disarm and Exit**
master.arducopter_disarm()
print("🛑 Drone Disarmed!")
master.close()
print("✅ Done!")
