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


# 🔹 **Check GPS Status**
def check_gps():
    msg = master.recv_match(type='GPS_RAW_INT', blocking=True)
    print(f"📡 GPS Fix Type: {msg.fix_type} (0=No Fix, 3=3D Fix)")
    if msg.fix_type < 3:
        print("❌ No GPS Fix! Ensure GPS is connected or use a non-GPS mode.")
        return False
    return True


# 🔹 **Check EKF Status**
def check_ekf():
    msg = master.recv_match(type='EKF_STATUS_REPORT', blocking=True)
    print(f"🧠 EKF Status: {msg.flags}")
    if msg.flags == 0:
        print("❌ EKF Not Ready! Wait for sensors to initialize.")
        return False
    return True


# 🔹 **Check Battery Voltage**
def check_battery():
    msg = master.recv_match(type='SYS_STATUS', blocking=True)
    voltage = msg.voltage_battery / 1000.0  # Convert mV to V
    print(f"🔋 Battery Voltage: {voltage}V")
    if voltage < 10.5:
        print("⚠️ WARNING: Low battery! Charge before flying.")
        return False
    return True


# 🔹 **Ensure GUIDED Mode**
def set_guided_mode():
    master.set_mode("GUIDED")
    time.sleep(2)
    msg = master.recv_match(type='HEARTBEAT', blocking=True)
    mode = mavutil.mode_string_v10(msg)
    print(f"🎮 Current Flight Mode: {mode}")
    if mode != "GUIDED":
        print("❌ Failed to set GUIDED mode! Check flight controller settings.")
        return False
    return True


# 🔹 **Arm the Drone**
def arm_drone():
    master.arducopter_arm()
    time.sleep(2)
    msg = master.recv_match(type='HEARTBEAT', blocking=True)
    if not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
        print("❌ Drone failed to arm! Check pre-arm checks.")
        return False
    print("🚀 Drone Armed!")
    return True


# 🔹 **Disable Safety Switch**
def disable_safety_switch():
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_PARAMETER, 0,
        220, 0, 0, 0, 0, 0, 0  # 220 = BRD_SAFETYENABLE, Set to 0
    )
    print("🔓 Safety Disabled!")


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


# ✅ **Pre-Flight Checklist**
if not check_battery():
    print("❌ Flight Aborted: Low Battery!")
    master.close()
    exit()

# if not check_ekf():
#     print("❌ Flight Aborted: EKF Not Ready!")
#     master.close()
#     exit()

# if not check_gps():
#     print("⚠️ WARNING: No GPS detected! Switching to Local NED mode.")

if not set_guided_mode():
    print("❌ Flight Aborted: Unable to switch to GUIDED mode!")
    master.close()
    exit()

disable_safety_switch()

if not arm_drone():
    print("❌ Flight Aborted: Failed to arm!")
    master.close()
    exit()

# ✅ **Execute Movement**
move_forward(vx=1.0, duration=3)  # Move forward
descend(vz=-0.5, duration=5)  # Then descend

# 🔹 **Disarm and Exit**
master.arducopter_disarm()
print("🛑 Drone Disarmed!")

master.close()
print("✅ Done!")
