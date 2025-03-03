import time
from pymavlink import mavutil

master = mavutil.mavlink_connection('/dev/serial/by-id/usb-Holybro_Pixhawk6C_1E0030001151333036383238-if00', baud=115200)

master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
      (master.target_system, master.target_component))

# while True:
#     # msg = master.recv_match(type='ATTITUDE', blocking=True) #PITCH, YAW, ROLL
#     # msg = master.recv_match(type='SYS_STATUS', blocking=True) # VOLTAGE
#     # msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True) #position and velocity
#     msg = master.recv_match(type='SERVO_OUTPUT_RAW', blocking=True)
#     if msg:
#         print(f"Motor PWM Outputs: {msg.servo1_raw}, {msg.servo2_raw}, {msg.servo3_raw}, {msg.servo4_raw}")
def descend(vz=-0.5, duration=5):
    """Command the drone to descend at a set velocity"""
    for _ in range(duration):
        master.mav.set_position_target_local_ned_send(
            0, master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # Local NED frame
            int(0b110111000111),  # Only use velocity
            0, 0, 0,  # Position (ignored)
            0, 0, vz,  # Velocity: Only Z downward
            0, 0, 0,  # Acceleration (ignored)
            0, 0  # Yaw (ignored)s
        )
        print(f"🔽 Descending at {vz} m/s")
        time.sleep(1)

    print("✔ Descent complete!")

descend()