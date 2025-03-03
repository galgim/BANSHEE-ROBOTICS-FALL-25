import time
from pymavlink import mavutil

master = mavutil.mavlink_connection('/dev/serial/by-id/usb-Holybro_Pixhawk6C_1E0030001151333036383238-if00', baud=115200)

master.arducopter_arm()

print('arm')

master.mav.command_long_send(
    1, 0,
    mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST, 0,
    1, 0, 20, 2, 0, 0, 0
)



time.sleep(5)

master.arducopter_disarm()

print('done')

master.close()