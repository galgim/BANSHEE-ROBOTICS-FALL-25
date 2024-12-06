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

# Define Motor Movements

def Drone_grab():
    start_time = time.time()
    print("Grabbing drone")
    motor.dxlSetVelo([10, 10, 10], [2, 3, 4])
    motor.simMotorRun([132, 347, 54],[2, 3, 4])

#######

def Push_low():    
    start_time = time.time()
    print("Push in low sequence")
    motor.dxlSetVelo([13, 38, 20], [2, 3, 4])   # set initial velocity
    print("place chamber")
    motor.simMotorRun([132, 343, 70],[2, 3, 4]) # move arm to first position
    motor.dxlSetVelo([15, 28, 10], [2, 3, 4])   # set final velocity
    motor.simMotorRun([80, 248, 95],[2, 3, 4])  # move arm w/ battery to chamber
    Open()                                      # let go of battery

def Pull_low():    
    start_time = time.time()
    print("Pull out low sequence")
    motor.dxlSetVelo([30, 20, 30], [2, 3, 4])   # set initial velocity
    print("remove chamber")
    motor.simMotorRun([80, 248, 95],[2, 3, 4])  # move arm to chamber position
    Close()                                     # grab battery
    motor.dxlSetVelo([12, 30, 20], [2, 3, 4])   # set pull out velocity
    motor.simMotorRun([132, 343, 40],[2, 3, 4]) # pull battery out


def Push_high():    
    start_time = time.time()
    print("Push in high sequence")
    motor.dxlSetVelo([20, 20, 20], [2, 3, 4])   # set initial velocity
    print("place chamber")
    motor.simMotorRun([130, 310, 90],[2, 3, 4]) # move arm to first position
    motor.dxlSetVelo([15, 28, 10], [2, 3, 4])   # set final velocity
    motor.simMotorRun([60, 180, 144],[2, 3, 4])  # move arm w/ battery to chamber
    Open()                                      # let go of battery

def Pull_high():    
    start_time = time.time()
    print("Pull out high sequence")
    motor.dxlSetVelo([20, 20, 30], [2, 3, 4])   # set initial velocity
    print("remove chamber")
    motor.simMotorRun([60, 180, 144],[2, 3, 4])  # move arm to chamber position
    Close()                                     # grab battery
    motor.dxlSetVelo([12, 30, 20], [2, 3, 4])   # set pull out velocity
    motor.simMotorRun([130, 310, 85],[2, 3, 4]) # pull battery out


# Close Claw
def Close():
    start_time = time.time()
    print("close claw")
    motor.dxlSetVelo([30], [0])
    motor.simMotorRun([108],[0])
    time.sleep(1)

#Open Claw
def Open():
    start_time = time.time()
    print("open claw")
    motor.dxlSetVelo([30], [0])
    motor.simMotorRun([45],[0])
    time.sleep(1)

# Setup initial motor positions
def startsetup():
    print("setting up")
    motor.dxlSetVelo([30, 30, 30, 20, 30], [0, 1, 2, 3, 4])
    motor.simMotorRun([225, 222, 347, 139], [1, 2, 3, 4])
    time.sleep(1)

# Dictionary mapping commands to functions
Command_dict = {
    "push low": Push_low,
    "drone": Drone_grab,
    "close": Close,
    "open": Open,
    "setup": startsetup,
}

def main(args=None):
    startsetup()
    Open()
    time.sleep(1)
    Pull_high()
    time.sleep(1)

    # Open()
    # startsetup()
    # time.sleep(1)
    # Pull_low()
    # startsetup()
    # Close()
    # time.sleep(1)
    # Push_low()
    # time.sleep(1)
    # startsetup()

    


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