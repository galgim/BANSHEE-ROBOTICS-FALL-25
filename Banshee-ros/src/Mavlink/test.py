import time
from pymavlink import mavutil

# Connect to the Pixhawk via serial
master = mavutil.mavlink_connection(
    '/dev/serial/by-id/usb-Holybro_Pixhawk6C_1E0030001151333036383238-if00',
    baud=115200
)

# Wait for the heartbeat before proceeding
master.wait_heartbeat()
print("✅ Heartbeat received from system %u, component %u" %
      (master.target_system, master.target_component))

# Set mode to GUIDED (required for velocity control)
master.set_mode("GUIDED")
time.sleep(2)

# Arm the drone
master.arducopter_arm()
time.sleep(2)  # Wait for arming confirmation
print("🚀 Drone Armed!")


def move_forward(vx=1.0, duration=3):
    """Move the drone forward at a set velocity."""
    for _ in range(duration):
        master.mav.set_position_target_local_ned_send(
            0, master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # Local frame
            int(0b000011000000),  # Only velocity
            0, 0, 0,  # Position (ignored)
            vx, 0, 0,  # Velocity: X forward
            0, 0, 0,  # Acceleration (ignored)
            0, 0  # Yaw (ignored)
        )
        print(f"➡ Moving forward at {vx} m/s")
        time.sleep(1)

    print("✅ Forward movement complete!")


def descend(vz=-0.5, duration=5):
    """Command the drone to descend at a set velocity"""
    for _ in range(duration):
        master.mav.set_position_target_local_ned_send(
            0, master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # Local NED frame
            int(0b000011000000),  # Only velocity
            0, 0, 0,  # Position (ignored)
            0, 0, vz,  # Velocity: Only Z downward
            0, 0, 0,  # Acceleration (ignored)
            0, 0  # Yaw (ignored)
        )
        print(f"🔽 Descending at {vz} m/s")
        time.sleep(1)

    print("✔ Descent complete!")


# Execute movement sequence
move_forward(vx=1.0, duration=3)  # Move forward for 3 sec
descend(vz=-0.5, duration=5)  # Then descend

# Disarm the drone after landing
master.arducopter_disarm()
print("🛑 Drone Disarmed!")

# Close the connection
master.close()
print("✅ Done!")
