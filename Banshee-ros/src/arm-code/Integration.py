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

# Calculate the angle for the max length reaching out in the x position
max_length_angle = calculation.angle_Calc([375, 0, 73], 0)

# Check movement of motors
def checkMovement(ids):
    motorStatus = [0] * len(ids)
    finished = [1] * len(ids)
    firstPosition = 0
    secondPosition = 0
    while True:
        for id in ids:
            firstPosition = motor.ReadMotorData(id, ADDR_PRESENT_POSITION)
            time.sleep(0.1)
            secondPosition = motor.ReadMotorData(id, ADDR_PRESENT_POSITION)
            if abs(firstPosition - secondPosition) < 5:
                motorStatus[id] = 1
        if motorStatus == finished:
            print("finished")
            break

# Define various arm movements
def Grab_high():
    start_time = time.time()
    print("Grabbing high")
    motor.dxlSetVelo([30, 30, 10, 10, 10], [0, 1, 2, 3, 4])
    motor.simMotorRun([112, 132, 250, 54],[0, 2, 3, 4])
    # 2 and 4 is at a 90 degree angle from starting
    # adjust 3 for height
    # probably have to adjust 4 to be parallel with battery

    time.sleep(0.1)

    motor.simMotorRun([45],[0])
    # grabbing battery

    time.sleep(0.1)

def Grab_low():
    start_time = time.time()
    print("Grabbing low")
    motor.dxlSetVelo([30, 30, 10, 10, 10], [0, 1, 2, 3, 4])
    motor.simMotorRun([112, 132, 300, 54],[0, 2, 3, 4])
    # 2 and 4 is at a 90 degree angle from starting
    # adjust 3 for height
    # probably have to adjust 4 to be parallel with battery

    time.sleep(0.1)

    motor.simMotorRun([45],[0])
    # grabbing battery

    time.sleep(0.1)

def Open():
    start_time = time.time()
    print("open claw")
    motor.dxlSetVelo([30, 30, 10, 10, 10], [0, 1, 2, 3, 4])
    motor.simMotorRun([45],[0])

# Setup initial motor positions
def startsetup():
    print("setting up")
    motor.dxlSetVelo([30, 30, 10, 30, 10], [0, 1, 2, 3, 4])
    motor.simMotorRun([45, 222, 222, 347, 144], [0, 1, 2, 3, 4])
    time.sleep(1)

# Dictionary mapping commands to functions
Command_dict = {
    "grab high": Grab_high,
    "grab low": Grab_low,
    "open": Open,
    "setup": startsetup,
}

def main(args=None):
    # rclpy.init(args=args)
    # node = MyNode()
    while True:
        command = input("Enter a command: ")
        if command in Command_dict:
            # startsetup()
            Command_dict[command]()
            # startsetup()
        elif command == "exit":
            print("Exiting program.")
            break
        else:
            print("Invalid command. Please try again.")
    # rclpy.shutdown()

# Run the main function
if __name__ == '__main__':
    main()