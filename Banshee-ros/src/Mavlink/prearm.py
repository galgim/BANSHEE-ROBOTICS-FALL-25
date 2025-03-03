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

# ✅ Arm the drone
master.arducopter_arm()
time.sleep(3)

# ✅ Function to move the drone
def move_drone(vx=1.0, vz=0, duration=3):
    """Move the drone in the Local NED frame (without GPS)"""
    for _ in range(duration):
        master.mav.set_position_target_local_ned_send(
            0, master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  
            int(0b000011000000),  # Velocity control only
            0, 0, 0,  # Position (ignored)
            vx, 0, vz,  # Velocity (X forward, Y ignored, Z up/down)
            0, 0, 0,  # Acceleration (ignored)
            0, 0  # Yaw (ignored)
        )
        print(f"🚀 Moving: Forward {vx} m/s, Descend {vz} m/s")
        time.sleep(1)

    print("✅ Movement complete!")

# ✅ Move forward first
move_drone(vx=1.0, duration=3)

# ✅ Then descend
move_drone(vx=0, vz=-0.5, duration=5)

# ✅ Disarm after movement
master.arducopter_disarm()
print("🛑 Drone Disarmed!")

# ✅ Close connection
master.close()
print("✅ Done!")
