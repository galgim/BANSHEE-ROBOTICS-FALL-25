#!/usr/bin/env python3
import math
import motorctrl_v2 as motor
import Movement_Calc_v2 as calculation
import numpy as np
import time
import cv2
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist

# Define motor ID
BASE_ID = 1
BICEP_ID = 2
FOREARM_ID = 3
WRIST_ID = 4
CLAW_ID = 0

# Define port number for Raspberry Pi
PORT_NUM = '/dev/ttyUSB0'  # for rpi

# Define move mode and address for present position
MOVEARM_MODE = 1
ADDR_PRESENT_POSITION = 132

# List of all motor IDs
ALL_IDs = [BASE_ID, BICEP_ID, FOREARM_ID, WRIST_ID, CLAW_ID]
MOVE_IDs = [BASE_ID, BICEP_ID, FOREARM_ID, WRIST_ID, CLAW_ID]

# Initialize motor port
motor.portInitialization(PORT_NUM, ALL_IDs)
# ctrl.portInitialization(PORT_NUM, ALL_IDs)

# # Define Motor Movements

# Grab high
def Grab_high():
    start_time = time.time()
    print("Grabbing high")
    motor.dxlSetVelo([30, 30, 10, 10, 10], [0, 1, 2, 3, 4])
    motor.simMotorRun([170, 330, 90],[2, 3, 4])
    time.sleep(0.1)

# Grab low
def Grab_low():
    start_time = time.time()
    print("Grabbing low")
    motor.dxlSetVelo([30, 30, 10, 10, 10], [0, 1, 2, 3, 4])
    motor.simMotorRun([132, 347, 54],[2, 3, 4])
    time.sleep(0.1)

def Drone_grab():
    start_time = time.time()
    print("Grabbing drone")
    motor.dxlSetVelo([10, 10, 10], [2, 3, 4])
    motor.simMotorRun([132, 347, 54],[2, 3, 4])

#######

def Push_low():    
    start_time = time.time()
    print("Push in low sequence")
    motor.dxlSetVelo([13, 23, 15], [2, 3, 4])
    # max_length_angle = calculation.angle_Calc([190, 0, 73], 0)
    # motor.simMotorRun(max_length_angle, [1, 2, 3, 4])
    # for i in range(270,370,10):
    #     print(i)
    #     initial_pull_out_angle = calculation.angle_Calc([i,0,100], 0)
    #     print("move 4 pull away slight")
    #     motor.simMotorRun(initial_pull_out_angle, [1, 2, 3, 4])
    #     time.sleep(3)
    print("place chamber")
    motor.simMotorRun([132, 343, 60],[2, 3, 4]) # guesstimate
    # motor.dxlSetVelo([10, 8, 5], [2, 3, 4])
    # motor.simMotorRun([110, 330, 60],[2, 3, 4])
    # time.sleep(2)
    # print("ready")
    # motor.dxlSetVelo([10, 8, 10], [2, 3, 4])
    # motor.simMotorRun([100, 320, 50],[2, 3, 4])
    # time.sleep(1)
    # # slight push
    # motor.simMotorRun([310, 65],[3, 4])
    # time.sleep(1)
    # motor.simMotorRun([290, 75],[3, 4])
    # time.sleep(1)
    # motor.simMotorRun([90, 270, 90],[2, 3, 4])
    # time.sleep(1)
    # motor.simMotorRun([80, 260, 95],[2, 3, 4])
    time.sleep(1)
    motor.simMotorRun([80, 250, 95],[2, 3, 4])

def Push_high():    
    start_time = time.time()
    print("Push in low sequence")
    motor.dxlSetVelo([30, 30, 10, 10, 10], [0, 1, 2, 3, 4])
    motor.simMotorRun([120, 250, 80],[2, 3, 4]) # guesstimate
    time.sleep(0.1)


# Close Claw
def Close():
    start_time = time.time()
    print("close claw")
    motor.dxlSetVelo([30, 30, 30, 30, 10], [0, 1, 2, 3, 4])
    motor.simMotorRun([108],[0])
    time.sleep(1)
    # motor.simMotorRun([134],[1])

#Open Claw
def Open():
    start_time = time.time()
    print("open claw")
    motor.dxlSetVelo([30, 30, 30, 30, 30], [0, 1, 2, 3, 4])
    motor.simMotorRun([45],[0])
    time.sleep(1)

# Setup initial motor positions
def startsetup():
    print("setting up")
    motor.dxlSetVelo([30, 30, 30, 20, 30], [0, 1, 2, 3, 4])
    motor.simMotorRun([224, 222, 347, 139], [1, 2, 3, 4])
    time.sleep(1)

# Dictionary mapping commands to functions
Command_dict = {
    "grab high": Grab_high,
    "grab low": Grab_low,
    "push low": Push_low,
    "drone": Drone_grab,
    "close": Close,
    "open": Open,
    "setup": startsetup,
}

def main(args=None):
    Open()
    startsetup()
    Close()
    time.sleep(1)
    Push_low()
    


if __name__ == '__main__':

    main()
    
# TOP_BVM = 0
# BOT_BVM = 1
# DRONE_BAT = 2

# def pull_out(dev):
#     if dev == 0:
#         motor.dxlSetVelo([30, 30, 30, 30, 10], [0, 1, 2, 3, 4])
#         # motor.simMotorRun([x,x,x,x,x],[0,1,2,3,4])
#         time.sleep(1)
#     elif dev == 1:
#         motor.dxlSetVelo([30, 30, 30, 30, 10], [0, 1, 2, 3, 4])
#         # motor.simMotorRun([x,x,x,x,x],[0,1,2,3,4])
#         time.sleep(1)
#     elif dev == 2:
#         motor.dxlSetVelo([30, 30, 30, 30, 10], [0, 1, 2, 3, 4])
#         # motor.simMotorRun([x,x,x,x,x],[0,1,2,3,4])
#         time.sleep(1)
#     else:
#         print("INVALID")

# def push_in(dev):
#     if dev == 0:
#         motor.dxlSetVelo([30, 30, 30, 30, 10], [0, 1, 2, 3, 4])
#         # motor.simMotorRun([x,x,x,x,x],[0,1,2,3,4])
#         time.sleep(1)
#     elif dev == 1:
#         motor.dxlSetVelo([30, 30, 30, 30, 10], [0, 1, 2, 3, 4])
#         # motor.simMotorRun([x,x,x,x,x],[0,1,2,3,4])
#         time.sleep(1)
#     elif dev == 2:
#         motor.dxlSetVelo([30, 30, 30, 30, 10], [0, 1, 2, 3, 4])
#         # motor.simMotorRun([x,x,x,x,x],[0,1,2,3,4])
#         time.sleep(1)
#     else:
#         print("INVALID")

# try:
#     mode = int(input("Enter action (0 for pull_out, 1 for push_in): "))
#     device = int(input("Enter device (0 for TOP_BVM, 1 for BOT_BVM, 2 for DRONE_BAT): "))

#     if mode == 0:
#         pull_out(device)
#     elif mode == 1:
#         push_in(device)
#     else:
#         print("Invalid action")

# except ValueError:
#     print("Invalid input. Please enter numeric values only.")