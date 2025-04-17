#!/usr/bin/env python3
import math
import motorctrl_v2 as motor
import Movement_calc_v2 as calculation
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

def Push_low():    
    start_time = time.time()
    print("Push in low sequence")
    motor.dxlSetVelo([117, 17, 94], [2, 3, 4])                    # set initial velocity (new)
    print("place chamber")
    motor.simMotorRun([105, 330, 45],[2, 3, 4])                   # move arm to first position (new)
    time.sleep(.5)
    motor.dxlSetVelo([25, 82, 50], [2, 3, 4])                     # set final velocity
    motor.simMotorRun([74, 253, 92],[2, 3, 4])                    # move arm to chamber position (new)
    Open()                                                        # let go of battery

# Pull Battery into low BVM
def Pull_low():    
    start_time = time.time()
    print("Pull out low sequence")
    motor.dxlSetVelo([60, 142, 99, 45], [1, 2, 3, 4])             # set initial speed (new)
    print("remove chamber")
    motor.simMotorRun([74, 253, 92],[2, 3, 4])                    # move arm to chamber position (new)
    Close()                                                       # grab battery
    motor.dxlSetVelo([15, 78, 54], [2, 3, 4])                     # set pull out velocity
    motor.simMotorRun([100, 328, 43],[2, 3, 4])                   # move arm to middle position
    motor.dxlSetVelo([127, 19, 109], [2, 3, 4])                   # set grab speed                      
    startsetup()
    #Droneside()

# Push Battery into top BVM
def Push_high():    
    start_time = time.time()
    print("Push in high sequence")
    motor.dxlSetVelo([92, 37, 70], [2, 3, 4])                     # set initial velocity (new)
    print("place chamber")
    motor.simMotorRun([130, 310, 90],[2, 3, 4])                   # move arm to first position
    time.sleep(0.5)
    motor.dxlSetVelo([40, 100, 50], [2, 3, 4])                    # set final velocity (new)
    motor.simMotorRun([90, 222, 123],[2, 3, 4])                   # move arm to chamber (new)
    Open()                                                        # let go

# Pull Battery from top BVM
def Pull_high():    
    start_time = time.time()
    print("Pull out high sequence")
    motor.dxlSetVelo([60, 30, 30, 5], [1, 2, 3, 4])             # set initial velocity (new)-reduced m.p 2 and 3 by 60
    motor.simMotorRun([90, 222, 123],[2, 3, 4])                   # move arm to chamber (new)
    Close()                                                       # grab battery
    print("remove chamber")
    motor.dxlSetVelo([70, 140, 74], [2, 3, 4])                    # set pull out velocity (new)
    motor.simMotorRun([130, 320, 70],[2, 3, 4])                   # middle position
    startsetup()
    #Droneside()

# Push Battery into Drone
def Drone_push():
    start_time = time.time()
    print("Drone push sequence")
    motor.dxlSetVelo([30, 50, 68, 13, 49], [0, 1, 2, 3, 4])       # set initial speed
    print("push drone bat")
    motor.simMotorRun([45], [1])                                  # turn around
    motor.simMotorRun([154, 334, 90],[2, 3, 4])                   # move arm to first position
    motor.dxlSetVelo([59, 91, 22], [2, 3, 4])                     # set push in speed (new)
    motor.simMotorRun([95, 243, 112],[2, 3, 4])                   # move arm to chamber position
    Open()                                                        # push battery

# Pull Battery from Drone
def Drone_pull():
    start_time = time.time()
    print("Drone pull sequence")
    motor.dxlSetVelo([30, 50, 68, 13, 49], [0, 1, 2, 3, 4])       # set initial velocity (new)
    print("remove drone bat")      
    motor.simMotorRun([154, 334, 90],[2, 3, 4])                   # get to middle position
    motor.dxlSetVelo([59, 91, 22], [2, 3, 4])                     # set grab speed  (new)                    
    motor.simMotorRun([95, 243, 112],[2, 3, 4])                   # move to battery position
    Close()  
    motor.simMotorRun([154, 334, 90],[2, 3, 4])                   # get to middle position
    motor.dxlSetVelo([25, 15, 25], [2, 3, 4])                     # set grab speed                      
    startsetup()
    #BVMside()                   

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
    start_time = time.time()
    print("setting up")
    motor.dxlSetVelo([60, 20, 50], [2, 3, 4])
    motor.simMotorRun([222, 347, 139], [2, 3, 4])
    time.sleep(1)

def BVMside():
    start_time = time.time()
    print("BVMside")
    motor.dxlSetVelo([80], [1])
    motor.simMotorRun([225], [1])
    time.sleep(1)

def Droneside():
    start_time = time.time()
    print("Droneside")
    motor.dxlSetVelo([80], [1])
    motor.simMotorRun([45], [1])
    time.sleep(1)  

# Dictionary mapping commands to functions
Command_dict = {
    "push low": Push_low,
    "drone push": Drone_push,
    "drone pull": Drone_pull,
    "close": Close,
    "open": Open,
    "setup": startsetup,
}

def main(args=None):    
    BVMside()
    startsetup()
    Pull_high()
    time.sleep(.5)
    Push_high()
    startsetup()
   
  

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