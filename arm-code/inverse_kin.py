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

#angle for the max length reaching out in the x pos
max_length_angle = calculation.angle_Calc([375, 0, 50], 0)
# angle01 = calculation.angle_Calc([50, 0, 375], 0)

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

def checkMovement(ids):
    motorStatus = [0] * len(ids)
    finished = [1] * len(ids)
    firstPosition = 0
    secondPosition = 0
    while True:
        for id in (ids):
            firstPosition = motor.ReadMotorData(id, ADDR_PRESENT_POSITION)
            time.sleep(.1)
            secondPosition = motor.ReadMotorData(id, ADDR_PRESENT_POSITION)
            if (abs(firstPosition - secondPosition) < 5):
                motorStatus[id] = 1
        if (motorStatus == finished):
            print("finished")
            break

# print("set up move")
# motor.simMotorRun([110, 223, 270, 47, 272], [0, 1, 2, 3, 4])  # Reset claw looking up
# time.sleep(2)

# while True:
#     test = [0, 0, 0]
#     for i in range(0,3):
#         if i == 1:
#             print("coord inputed")  
#         else:
#             test[i] = int(input("Enter Coord: "))

#     print(test)

#     angle01 = calculation.angle_Calc(test, 1)
#     print("move 2")
#     motor.simMotorRun(angle01, [1, 2, 3, 4])
#     time.sleep(4)


# print("Set up move")
# motor.simMotorRun(reset_angle, [1, 2, 3, 4])
# time.sleep(4)

# print("move 1")
# motor.simMotorRun(angle0, [1, 2, 3, 4])
# time.sleep(4)

# print("move 2")
# motor.simMotorRun(max_length_angle, [1, 2, 3, 4])
# time.sleep(4)

# print("move 3")
# motor.simMotorRun(angle3, [1, 2, 3, 4])
# time.sleep(4)

# print("move 4")
# motor.simMotorRun(angle4, [1, 2, 3, 4])
# time.sleep(4)

# print("Set up move")
# motor.simMotorRun(reset_angle, [1, 2, 3, 4])
# time.sleep(4)

print("set up move")
motor.simMotorRun([110, 223, 270, 47, 272], [0, 1, 2, 3, 4])  # Reset claw looking up
time.sleep(2)

print("move 1 move to chamber")
motor.simMotorRun(max_length_angle, [1, 2, 3, 4])
time.sleep(4)

print("move 2 pitch wrist")
motor.simMotorRun([190, 200], [3, 4])  # Reset claw looking up
time.sleep(2)

print("move 3 close grip")
motor.simMotorRun([30], [0])  # Reset claw looking up
time.sleep(2)

print("move 4 pull forearm back")
motor.simMotorRun([160], [3])  # Reset claw looking up
time.sleep(2)

# test_angle = calculation.angle_Calc([150,0,100], 0) might work for return to home
pull_out_angle = calculation.angle_Calc([300,0,60], 0)
print(pull_out_angle)
print("move 4 pull away slight")
motor.simMotorRun(pull_out_angle, [1, 2, 3, 4])
time.sleep(4)

test_angle = calculation.angle_Calc([250,0,60], 0)
print(test_angle)
print("move 5 pull away more")
motor.simMotorRun(test_angle, [1, 2, 3, 4])
time.sleep(4)

test_angle = calculation.angle_Calc([200,0,60], 0)
print(test_angle)
print("move 6 pull away even more")
motor.simMotorRun(test_angle, [1, 2, 3, 4])
time.sleep(4)

print("move 7 pull forearm back")
motor.simMotorRun([45], [3])  # Reset claw looking up
time.sleep(2)

print("move 8 pull forearm back")
motor.simMotorRun([200], [2])  # Reset claw looking up
time.sleep(2)

print("move 9 pull forearm back")
motor.simMotorRun([220], [4])  # Reset claw looking up
time.sleep(2)

print("move 10 pull forearm back")
motor.simMotorRun([250], [2])  # Reset claw looking up
time.sleep(2)

print("move 11 pull forearm back")
motor.simMotorRun([190], [4])  # Reset claw looking up
time.sleep(2)

print("move 10 pull forearm back")
motor.simMotorRun([290], [2])  # Reset claw looking up
time.sleep(2)

motor.simMotorRun([265, 47, 170], [2, 3, 4])
time.sleep(2)
motor.simMotorRun([275], [4])
time.sleep(7)

#Push In Battery
print("move back to chamber")
motor.simMotorRun([180], [2])
time.sleep(2)


push_in_angle = calculation.angle_Calc([310,0,65], 0)
print(push_in_angle)
print("push in to chamber")
motor.simMotorRun(push_in_angle, [1, 2, 3, 4])
time.sleep(4)

