import time
import tkinter as tk
from pymavlink import mavutil

# ✅ Connect to Pixhawk
print("🔗 Connecting to Pixhawk...")
master = mavutil.mavlink_connection(
    '/dev/serial/by-id/usb-Holybro_Pixhawk6C_1E0030001151333036383238-if00',
    baud=115200
)

# ✅ Wait for heartbeat
master.wait_heartbeat()
print("✅ Heartbeat received!")

# ✅ Set GUIDED Mode
def set_guided_mode():
    master.set_mode("GUIDED")
    time.sleep(2)
    msg = master.recv_match(type='HEARTBEAT', blocking=True)
    mode = mavutil.mode_string_v10(msg)
    print(f"🎮 Current Flight Mode: {mode}")
    if mode != "GUIDED":
        print("❌ Failed to switch to GUIDED mode! Check flight controller settings.")
        master.close()
        exit()

# ✅ Arm and Takeoff
def arm_and_takeoff(altitude):
    print("🚀 Arming motors...")
    master.arducopter_arm()
    time.sleep(3)

    print("🔼 Taking off...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
        0, 0, 0, 0, 0, 0, altitude
    )

    # Wait until the drone reaches target altitude
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        current_alt = msg.relative_alt / 1000.0  # Convert mm to meters
        print(f"📏 Altitude: {current_alt:.1f}m")
        if current_alt >= altitude - 1.0:
            print("🎯 Target altitude reached!")
            break
        time.sleep(1)

# ✅ Set Velocity Control
def send_velocity(vx, vy, vz):
    """ Sends velocity commands to the drone using MAVLink. """
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,  
        int(0b000011000000),  # Velocity only
        0, 0, 0,  # Position (ignored)
        vx, vy, vz,  # Velocity: X (Forward), Y (Left/Right), Z (Up/Down)
        0, 0, 0,  # Acceleration (ignored)
        0, 0  # Yaw (ignored)
    )
    print(f"➡ Moving: vx={vx}, vy={vy}, vz={vz}")

# ✅ Handle Keyboard Input
def key(event):
    if event.keysym == 'Up':
        send_velocity(1.0, 0, 0)  # Move Forward
    elif event.keysym == 'Down':
        send_velocity(-1.0, 0, 0)  # Move Backward
    elif event.keysym == 'Left':
        send_velocity(0, -1.0, 0)  # Move Left
    elif event.keysym == 'Right':
        send_velocity(0, 1.0, 0)  # Move Right
    elif event.keysym == 'r':
        print("🔄 Returning to Launch (RTL)...")
        master.set_mode("RTL")  # Return to launch

# ✅ Start Mission
set_guided_mode()
arm_and_takeoff(10)

# ✅ Tkinter Window for Keyboard Control
root = tk.Tk()
print("🎮 Control the drone with the arrow keys. Press 'r' for RTL mode.")
root.bind_all('<Key>', key)
root.mainloop()

# ✅ Close Connection
master.close()
print("✅ Done!")
