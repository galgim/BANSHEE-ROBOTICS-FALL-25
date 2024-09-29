#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Importing necessary libraries
from dynamixel_sdk import *  # Uses Dynamixel SDK library
import os

# System-specific keyboard input management
if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

# Define motor settings and control table addresses for XM series motors
MY_DXL = 'X_SERIES'
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
ADDR_PROFILE_VELOCITY = 112
DXL_MINIMUM_POSITION_VALUE = 0         # Minimum position (0 degrees)
DXL_MAXIMUM_POSITION_VALUE = 4095      # Maximum position (360 degrees)
DXL_MOVING_STATUS_THRESHOLD = 10       # Threshold for considering movement as complete
BAUDRATE = 57600

PROTOCOL_VERSION = 2.0  # Protocol version for XM series motors
DEVICENAME = '/dev/ttyUSB0'  # Port device name
TORQUE_ENABLE = 1  # Enable torque
TORQUE_DISABLE = 0  # Disable torque

# Initialize global variables for motor IDs and their positions
DXL_IDs = [1, 2, 3, 4]  # Example motor IDs, you can add more motors here

# Initialize PortHandler and PacketHandler
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Helper function to convert angles to Dynamixel units
def angle_to_position(angle):
    return int((angle / 360.0) * 4095)

# Helper function to convert Dynamixel units to angles
def position_to_angle(position):
    return (position / 4095.0) * 360

# Initialize and open the port
def initialize_port():
    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        getch()
        quit()

    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        getch()
        quit()

# Enable torque for all motors
def enable_torque():
    for DXL_ID in DXL_IDs:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print(f"Dynamixel ID:{DXL_ID} torque enabled")

# Disable torque for all motors
def disable_torque():
    for DXL_ID in DXL_IDs:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

# Function to move motors to specified angles
def move_to_angles(angle_targets):
    for i, DXL_ID in enumerate(DXL_IDs):
        goal_position = angle_to_position(angle_targets[i])
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, goal_position)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

# Function to get current motor angles
def get_current_angles():
    angles = []
    for DXL_ID in DXL_IDs:
        dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            current_angle = position_to_angle(dxl_present_position)
            angles.append(current_angle)
            print(f"[ID:{DXL_ID}] Current angle: {current_angle:.2f} degrees")
    return angles

# Close the port
def close_port():
    portHandler.closePort()

# Main function
def main():
    # Initialize port and enable torque
    initialize_port()
    enable_torque()

    while True:
        target_angles=[90,200,100,200]
        print("Moving motors to target angles...")
        move_to_angles(target_angles)
        get_current_angles()
        time.sleep(1)
        target_angles=[180,90,270,350]
        print("Moving motors to target angles...")
        move_to_angles(target_angles)
        get_current_angles()
        if getch() == chr(0x1b):  # ESC to quit
            break

    # Disable torque and close port before exiting
    disable_torque()
    close_port()

if __name__ == "__main__":
    main()
