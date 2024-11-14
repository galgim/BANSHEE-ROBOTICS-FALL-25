#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool  # Import message type for the 'done' signal
import math
from integration import motorctrl_v2 as motor
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

def pull_out(dev):
    if dev == 0:
        motor.dxlSetVelo([30, 30, 30, 30, 10], [0, 1, 2, 3, 4])
        # motor.simMotorRun([x,x,x,x,x],[0,1,2,3,4])
        time.sleep(1)
    elif dev == 1:
        motor.dxlSetVelo([30, 30, 30, 30, 10], [0, 1, 2, 3, 4])
        # motor.simMotorRun([x,x,x,x,x],[0,1,2,3,4])
        time.sleep(1)
    elif dev == 2:
        motor.dxlSetVelo([30, 30, 30, 30, 10], [0, 1, 2, 3, 4])
        # motor.simMotorRun([x,x,x,x,x],[0,1,2,3,4])
        time.sleep(1)
    else:
        print("INVALID")

def push_in(dev):
    if dev == 0:
        motor.dxlSetVelo([30, 30, 30, 30, 10], [0, 1, 2, 3, 4])
        # motor.simMotorRun([x,x,x,x,x],[0,1,2,3,4])
        time.sleep(1)
    elif dev == 1:
        motor.dxlSetVelo([30, 30, 30, 30, 10], [0, 1, 2, 3, 4])
        # motor.simMotorRun([x,x,x,x,x],[0,1,2,3,4])
        time.sleep(1)
    elif dev == 2:
        motor.dxlSetVelo([30, 30, 30, 30, 10], [0, 1, 2, 3, 4])
        # motor.simMotorRun([x,x,x,x,x],[0,1,2,3,4])
        time.sleep(1)
    else:
        print("INVALID")


class IntegrationNode(Node):
    def __init__(self):
        super().__init__('Integration_Node')
        self.mode = 0
        self.get_logger().info("Integration Node has started.")
        
        # Subscriber for 'ConfirmPosition' signal from Camera node
        self.subscription = self.create_subscription(
            Bool,'DestinationConfirm', self.done_callback, 10)
        
        self.armFinished = self.create_publisher(
           Bool, 'ArmDone', 10)
        
        # Flag to indicate if the node has received the signal to start
        self.start_signal_received = False

        self.run()

    def done_callback(self, msg):
      self.get_logger().info("Callback triggered, message received.")
      if msg.data:
        self.get_logger().info("Received 'done' signal from Stepper Node. Integration Node is now ready to execute commands.")
        self.start_signal_received = True

    def run(self):
      self.get_logger().info("Waiting for 'done' signal from Stepper Node...")  

      if self.start_signal_received and self.mode == 0:
        # Proceed to command execution after receiving 'done' signal
        pull_out(self.mode)
        self.mode = 1
        self.start_signal_received = False
        self.armFinished.publish(True)

      elif self.start_signal_received and self.mode == 1:
        # Proceed to command execution after receiving 'done' signal
        push_in(self.mode)
        self.mode = 0
        self.start_signal_received = False
        self.armFinished.publish(True)
        

def main(args=None):
    rclpy.init(args=args)
    node = IntegrationNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()