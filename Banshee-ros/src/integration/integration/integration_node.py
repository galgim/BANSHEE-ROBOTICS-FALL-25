#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int8  # Import message type for the 'done' signal
import math
from integration import motorctrl_v3 as motor
from integration import Movement_calc_v2 as calculation
import numpy as np
import time
import cv2

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


# Calculate the angle for the max length reaching out in the x position
max_length_angle = calculation.angle_Calc([375, 0, 73], 0)

# define
TOP_BVM = 0
BOT_BVM = 1
DRONE_BAT = 2

# Upper BVM batteries when dev = 0
# Lower BVM batteries when dev = 1
# Drone batteries when dev = 2

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

# Push Battery into low BVM
def Push_low():    
    start_time = time.time()
    print("Push in low sequence")
    motor.dxlSetVelo([20, 30, 25], [2, 3, 4])   # set initial velocity
    print("place chamber")
    motor.simMotorRun([94, 332, 38],[2, 3, 4]) # move arm to first position
    time.sleep(0.1)
    motor.dxlSetVelo([10, 40, 25], [2, 3, 4])   # set final velocity
    motor.simMotorRun([80, 248, 95],[2, 3, 4])  # move arm w/ battery to chamber
    Open()                                      # let go of battery'

# Pull Battery from low BVM
def Pull_low():    
    start_time = time.time()
    print("Pull out low sequence")
    motor.dxlSetVelo([30, 35, 25, 35], [1, 2, 3, 4])            # set initial velocity
    print("remove chamber")
    motor.simMotorRun([80, 248, 95],[2, 3, 4])                  # move arm to chamber position
    Close()                                                     # grab battery
    motor.dxlSetVelo([17, 37, 20], [2, 3, 4])                   # set pull out velocity
    motor.simMotorRun([132, 343, 40],[2, 3, 4])                 # pull battery out
    motor.dxlSetVelo([35, 20, 30], [2, 3, 4])                   # set grab speed                      
    motor.simMotorRun([222, 334, 139],[2, 3, 4])                # pull battery out
    motor.simMotorRun([45], [1])                   


# Push Battery into top BVM
def Push_high():    
    start_time = time.time()
    print("Push in high sequence")
    motor.dxlSetVelo([25, 25, 30], [2, 3, 4])   # set initial velocity
    print("place chamber")
    motor.simMotorRun([130, 310, 90],[2, 3, 4]) # move arm to first position
    motor.dxlSetVelo([17, 33, 15], [2, 3, 4])   # set final velocity
    motor.simMotorRun([60, 180, 144],[2, 3, 4])  # move arm w/ battery to chamber
    Open()                                      # let go of battery

# Pull Battery from top BVM
def Pull_high():    
    start_time = time.time()
    print("Pull out high sequence")
    motor.dxlSetVelo([30, 25, 25, 35], [1, 2, 3, 4])            # set initial velocity
    print("remove chamber")
    motor.simMotorRun([60, 180, 144],[2, 3, 4])                 # move arm to chamber position
    Close()                                                     # grab battery
    motor.dxlSetVelo([17, 33, 15], [2, 3, 4])                   # set pull out velocity
    motor.simMotorRun([130, 320, 70],[2, 3, 4])                 # middle position
    motor.dxlSetVelo([35, 20, 25], [2, 3, 4])                   # set grab speed                      
    motor.simMotorRun([222, 334, 139],[2, 3, 4])                # pull battery out
    motor.simMotorRun([45], [1])                   


# Push Battery into Drone
def Drone_push():
    start_time = time.time()
    print("Drone push sequence")
    motor.dxlSetVelo([30, 50, 30, 30, 30], [0, 1, 2, 3, 4])     # set initial speed
    print("push drone bat")
    motor.simMotorRun([45], [1])                                # turn around
    motor.simMotorRun([154, 334, 90],[2, 3, 4])                 # move arm to first position
    motor.dxlSetVelo([17, 27, 8], [2, 3, 4])                    # set push in speed
    motor.simMotorRun([95, 243, 112],[2, 3, 4])                 # move arm to chamber position
    Open()                                                      # push battery

# Pull Battery from Drone
def Drone_pull():
    start_time = time.time()
    print("Drone pull sequence")
    motor.dxlSetVelo([30, 50, 30, 30, 30], [0, 1, 2, 3, 4])     # set initial velocity
    print("remove drone bat")      
    motor.simMotorRun([154, 334, 90],[2, 3, 4])                 # get to middle position
    motor.dxlSetVelo([17, 27, 8], [2, 3, 4])                    # set grab speed                      
    motor.simMotorRun([95, 243, 112],[2, 3, 4])                 # move to battery position
    Close()  
    motor.simMotorRun([154, 334, 90],[2, 3, 4])                 # get to middle position
    motor.dxlSetVelo([25, 15, 25], [2, 3, 4])                    # set grab speed                      
    motor.simMotorRun([222, 334, 139],[2, 3, 4])                # pull battery out
    motor.simMotorRun([225], [1])                   

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
def startsetup(pid_controllers):
    start_time = time.time()
    print("setting up")
    motor.dxlSetVelo([30, 30, 30], [2, 3, 4], pid_controllers)
    motor.simMotorRun([222, 347, 139], [2, 3, 4])
    time.sleep(1)

def BVMside():
    start_time = time.time()
    print("BVMside")
    motor.dxlSetVelo([30], [1])
    motor.simMotorRun([225], [1])
    time.sleep(1)

def Droneside():
    start_time = time.time()
    print("BVMside")
    motor.dxlSetVelo([30], [1])
    motor.simMotorRun([45], [1])
    time.sleep(1)  

# Dictionary mapping commands to functions
Command_dict = {
    "grab h": Pull_high,
    "push h": Push_high,
    "pull l": Pull_low,
    "push l": Push_low,
    "pull d": Drone_pull,
    "push d": Drone_push,
    "close": Close,
    "open": Open,
    "setup": startsetup,
    "BVMside": BVMside,
    "Droneside": Droneside
}

class IntegrationNode(Node):
    def __init__(self):
        super().__init__('Integration_Node')
        self.mode = 0
        self.get_logger().info("Integration Node has started.")
        
        # Subscriber for 'ConfirmPosition' signal from Camera node
        self.subscription = self.create_subscription(
          Int8,'DestinationConfirm', self.done_callback, 10)
        
        self.armFinished = self.create_publisher(
          Bool, 'ArmDone', 10)
        
        # Flag to indicate if the node has received the signal to start
        self.start_signal_received = False
        self.batteryLevel = None

        startsetup()


    def done_callback(self, msg):
      self.start_signal_received = True
      self.batteryLevel = msg.data
      self.get_logger().info(f"Received batteryLevel {self.batteryLevel} from Camera Node. Integration Node is now ready to execute commands.")
      self.run()


    def run(self): 
      self.get_logger().info(f"Run method triggered. Start signal: {self.start_signal_received}")
      if self.start_signal_received:
        self.get_logger().info("Executing command...")

      # PULL FUNCTION
      if self.start_signal_received and self.mode == 0:
        # Proceed to command execution after receiving 'done' signal
        # pull_out(self.batteryLevel)
        if self.batteryLevel == 0:  
          Pull_high()
          startsetup()
        elif self.batteryLevel == 1:
          Pull_low()
          startsetup()
        else:
          Drone_pull()
          startsetup()

        self.mode = 1
        self.start_signal_received = False
        msg = Bool()
        msg.data = True
        self.armFinished.publish(msg)

      # PUSH FUNCTION
      elif self.start_signal_received and self.mode == 1:
        # Proceed to command execution after receiving 'done' signal
        # push_in(self.batteryLevel)if self.batteryLevel == 0:  
        if self.batteryLevel == 0:  
          Push_high()
          startsetup()
        elif self.batteryLevel == 1:
          Push_low()
          startsetup()
        else:
          Drone_push()
          startsetup()

        self.mode = 0
        self.start_signal_received = False
        msg = Bool()
        msg.data = True
        self.armFinished.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = IntegrationNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()