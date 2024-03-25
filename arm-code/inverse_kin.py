import math
import motorctrl_v1 as motor
import Movement_Calc_v2 as calculation
import numpy as np
import time
import cv2
import socket

BASE_ID = 1
BICEP_ID = 2
FOREARM_ID = 3
WRIST_ID = 4
CLAW_ID = 0


PORT_NUM = '/dev/ttyUSB0'  # for rpi
MOVEARM_MODE = 1
ADDR_PRESENT_POSITION = 132
ALL_IDs = [BASE_ID, BICEP_ID, FOREARM_ID, WRIST_ID, CLAW_ID]
MOVE_IDs = [BASE_ID, BICEP_ID, FOREARM_ID, WRIST_ID, CLAW_ID]

motor.portInitialization(PORT_NUM, ALL_IDs)

reset_angle = calculation.angle_Calc([10, 0, 5], 0)

#angle for the rest position of the arm
angle0 = calculation.angle_Calc([275, 0, 205], 0)

#angle for the rest position of the arm
angle01 = calculation.angle_Calc([350, 0, 75], 0)

# Test coordinate #1: first quadrant, smaller x, higher z
angle1 = calculation.angle_Calc([205, 0, 215], 0)

# Test coordinate #2: second quadrant, larger x, lower z
angle2 = calculation.angle_Calc([295, 0, 195], 0)

# Test coordinate #3: half x from rest, half z from rest, y=0
angle3 = calculation.angle_Calc([140, 0, 150], 0)

# Test coordinate #3: half x from rest, higher z from rest, y=0 ******* Check with the real arm to see if this configuration is physically posibble with the parallel claw
angle4 = calculation.angle_Calc([140, 0, 220], 0)

#"[%s, %s, %s, %s]" % (int(baseTheta), int(shoulderTheta), int(elbowTheta), int(wristTheta))

motor.dxlSetVelo([15, 15, 15, 15, 15], [0, 1, 2, 3, 4])  # ALWAYS SET SPEED BEFORE ANYTHING
time.sleep(0.5)

# print("set up move")
# motor.simMotorRun([110, 223, 270, 47, 272], [0, 1, 2, 3, 4])  # Reset claw looking up
# time.sleep(2)

print("Set up move")
motor.simMotorRun(reset_angle, [1, 2, 3, 4])
time.sleep(4)

print("move 1")
motor.simMotorRun(angle0, [1, 2, 3, 4])
time.sleep(4)

print("move 2")
motor.simMotorRun(angle01, [1, 2, 3, 4])
time.sleep(4)

print("move 3")
motor.simMotorRun(angle3, [1, 2, 3, 4])
time.sleep(4)

print("move 4")
motor.simMotorRun(angle4, [1, 2, 3, 4])
time.sleep(4)

print("Set up move")
motor.simMotorRun(reset_angle, [1, 2, 3, 4])
time.sleep(4)


