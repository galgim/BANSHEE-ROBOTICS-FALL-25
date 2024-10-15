#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Importing necessary libraries
from dynamixel_sdk import * # Uses Dynamixel SDK library
import os
import time

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
LEN_GOAL_POSITION = 4 #Byte Length of goal position
LEN_PRESENT_POSITION = 4 #Byte length of present positiond
DXL_MAXIMUM_POSITION_VALUE = 4095      # Maximum position (360 degrees)
DXL_MOVING_STATUS_THRESHOLD = 10       # Threshold for considering movement as complete
BAUDRATE = 57600

PROTOCOL_VERSION = 2.0  # Protocol version for XM series motors
DEVICENAME = '/dev/ttyUSB0'  # Port device name
TORQUE_ENABLE = 1  # Enable torque
TORQUE_DISABLE = 0  # Disable torque

# Initialize global variables for motor IDs and their positions
DXL_IDs = [0, 1, 2, 3, 4]  # Example motor IDs, you can add more motors here

# -----------------------------------------------------------------------------

# Define COMM_SUCCESS if not already defined
COMM_SUCCESS = 0
COMM_TX_FAIL = -1001

def portInitialization(portname, dxlIDs):
    global portHandler, packetHandler, DXL_ID, motorNum, DEVICENAME
    
    DEVICENAME = portname  # Portname should be passed in as an argument
    DXL_ID = dxlIDs  # List of motor IDs
    motorNum = len(DXL_ID)
    
    # Initialize PortHandler instance and PacketHandler instance
    portHandler = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    # Open port
    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        getch()  # Wait for user input if failed
        quit()

    # Set port baudrate
    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        getch()  # Wait for user input if failed
        quit()

    # Enable Dynamixel Torque for each motor
    for motorID in DXL_ID:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, motorID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))  # Handle communication failure
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))  # Handle errors from the motor
        else:
            print(f"Dynamixel ID:{motorID} torque enabled")
    #     if dxl_comm_result != COMM_SUCCESS:
    #         print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    #     elif dxl_error != 0:
    #         print("%s" % packetHandler.getRxPacketError(dxl_error))
    #     else:
    #         print("Dynamixel", motorID,
    #               "has been successfully connected")
    # print("-------------------------------------")

# -----------------------------------------------------------------------------------------

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

#The functions take in an array of X angle inputs and an array of X dynamixel IDs. Running the functions will drive the X dynamixels to the desired angle inputs simultaneously
def simWrite(dxl_goal_inputs, dxlIDs):
    idNum = len(dxlIDs)
    #Intializate simultaneous motor movement
    global motor_sync_write
    motor_sync_write = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)
    global motor_sync_read
    motor_sync_read = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
    #print("Motor_Sync_Write is ", motor_sync_write)
    #print("Motor_Sync_Read is ", motor_sync_read)

    #Create parameter storage for present positions
    for id in range(idNum):
        dxl_addparam_result = motor_sync_read.addParam(dxlIDs[id])
        # if dxl_addparam_result != True:
        #     print("[ID:%03d] groupSyncRead addparam failed" % dxlIDs[id])
        #print("DXL_ADDPARAM_RESULT is " ,dxl_addparam_result)

    param_goal_position = [0] * idNum
    #Allocate goal position values into 4-byte array for bicep and forearm motors. Dynamixel motors use either 2-bytes or 4-bytes
    for id in range(idNum): 
        param_goal_position[id] = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_inputs[id])), DXL_HIBYTE(DXL_LOWORD(dxl_goal_inputs[id])),DXL_LOBYTE(DXL_HIWORD(dxl_goal_inputs[id])), DXL_HIBYTE(DXL_HIWORD(dxl_goal_inputs[id]))]
    #print("param_goal_bicep_position is ", param_goal_position[1])
    #print("param_goal_bicep_position is ", param_goal_position[2])

    for id in range(idNum): 
    #Add goal position input values of bicep and forearm motors to Syncwrite parameter storage
        dxl_addparam_result = motor_sync_write.addParam(dxlIDs[id], param_goal_position[id])
        # if dxl_addparam_result != True:
        #     print("[ID:%03d] groupSyncWrite addparam failed" % dxlIDs[id])
        #print("groupSyncWrite for [ID:%03d] works" % (DXL_ID[device_index]))
   
    #Syncwrite goal position to bicep and forearm motors
    dxl_comm_result = motor_sync_write.txPacket()
    # if dxl_comm_result != COMM_SUCCESS:
    #     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    #Clear syncwrite parameter storage
    motor_sync_write.clearParam()

#The functions take in an array of X angle inputs and an array of X dynamixel IDs. Running the functions will drive the X dynamixels to the desired angle inputs simultaneously

def write(dxl_goal_inputs, dxlIDs):
    idNum = len(dxlIDs)
    #Intializate simultaneous motor movement
    global motor_sync_write
    motor_sync_write = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)
    global motor_sync_read
    motor_sync_read = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

    #Create parameter storage for present positions
    for id in range(idNum):
        dxl_addparam_result = motor_sync_read.addParam(dxlIDs[id])

    param_goal_position = [0] * idNum
    #Allocate goal position values into 4-byte array for bicep and forearm motors. Dynamixel motors use either 2-bytes or 4-bytes
    for id in range(idNum): 
        param_goal_position[id] = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_inputs[id])), DXL_HIBYTE(DXL_LOWORD(dxl_goal_inputs[id])),DXL_LOBYTE(DXL_HIWORD(dxl_goal_inputs[id])), DXL_HIBYTE(DXL_HIWORD(dxl_goal_inputs[id]))]

    for id in range(idNum): 
    #Add goal position input values of bicep and forearm motors to Syncwrite parameter storage
        dxl_addparam_result = motor_sync_write.addParam(dxlIDs[id], param_goal_position[id])
   
    #Syncwrite goal position to bicep and forearm motors
    dxl_comm_result = motor_sync_write.txPacket()

    #Clear syncwrite parameter storage
    motor_sync_write.clearParam()

def simPosCheck(dxl_goal_inputs, dxlIDs):
    idNum = len(dxlIDs)
    def simReadData():
        dxl_present_position = [0] * idNum
        # Syncread present position
        dxl_comm_result = motor_sync_read.txRxPacket()
        # if dxl_comm_result != COMM_SUCCESS:
            # print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        # Check if groupsyncread data of Dynamixel is available
        for motorID in dxlIDs:
            dxl_getdata_result = motor_sync_read.isAvailable(motorID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            # if dxl_getdata_result != True:
            #     print("[ID:%03d] groupSyncRead getdata failed" % motorID)

        # Get Dynamixel present position value
        for id in range(idNum):
            dxl_present_position[id] = motor_sync_read.getData(dxlIDs[id], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            #print("ID:%03d, position = %03d" % (dxlIDs[motorIndex], dxl_present_position[motorIndex]))

        return dxl_present_position
    
    print("Simultaneous position checking. DXL IDs being read: ", dxlIDs)
    repetition_status = [0] * idNum
    movement_status = [0] * idNum

    present_position = simReadData()
    kicker = 0
    while (kicker == 0):
        new_position = simReadData()
        #Kicker method to prevent the infinite looping of the motor
        for id in range(idNum):
            if (abs(new_position[id] - present_position[id]) < 2) and movement_status[id] == 0:
                repetition_status[id] += 1
            else:
                repetition_status[id] = 0
            if repetition_status[id] >= 10:
                kicker = 1
        
        present_position = new_position

        movement_complete_count = 0
        for id in range(idNum):
            if (abs(dxl_goal_inputs[id]- present_position[id]) < DXL_MOVING_STATUS_THRESHOLD):
                movement_complete_count += 1
                movement_status[id] = 1
        
        if (movement_complete_count == idNum):
            kicker = 1
    
    return present_position, movement_status

def motor_check(motorIndex, goal_position):
    motor_repetition_status = 0
    motor_status = 0
    
    motor_present_position = ReadMotorData(motorIndex, ADDR_PRESENT_POSITION)
    while 1:
        #Read moving status of motor. If status = 1, motor is still moving. If status = 0, motor stopped moving
        motor_new_position = ReadMotorData(motorIndex, ADDR_PRESENT_POSITION)

        #print("[ID:%03d] PresPos:%03d  NewPos:%03d" %
              #(DXL_ID[device_index], motor_present_position, motor_new_position))

        if (abs(motor_new_position - motor_present_position) < 2):
            motor_repetition_status += 1
        else:
            motor_repetition_status = 0
        if motor_repetition_status >= 10:
            break

        motor_present_position = motor_new_position
        #print("ID:%03d, motor_repetition_status: %03d" % (DXL_ID[device_index], motor_repetition_status))


        #Checks the present position of the motor and compares it to the goal position
        motor_check_value = abs(goal_position - motor_present_position)
        if (motor_check_value > DXL_MOVING_STATUS_THRESHOLD):
            motor_status = 0
            #print("ID:%03d, motor_check_value:%03d and motor status:%03d " % (DXL_ID[device_index],motor_check_value, motor_status))
        else:
            motor_status = 1
            #print("ID:%03d, motor_check_value:%03d and motor status:%03d " % (DXL_ID[device_index],motor_check_value, motor_status))
            break   

    return (motor_present_position, motor_status)

#Equation used to convert from angle degrees to positional units and vice versa
#To go from angles to step positions, order of values is 0, 360, 0, 4095
#To go from step positions to degrees, order of values is 0, 4095, 0, 360
#Inputs are angles or units you want to convert.
#Outputs are the converted values of angles or units
def _map(x, in_min, in_max, out_min, out_max):
    return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

#Reads the data of one motor in a given address. Dynamixel XM430-W350-R has most data in 4 bytes
#Input is (ID of motor to be read, address where data resides)
#Output is the data value that was read
def ReadMotorData(motorID, data_address):
    data_value, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(
        portHandler, motorID, data_address)
    # if dxl_comm_result != COMM_SUCCESS:
    #     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    # elif dxl_error != 0:
    #     print("%s" % packetHandler.getRxPacketError(dxl_error))
    return data_value

#Writes data to one motor at a given address. Dynamixel XM430-W350-R has most data in 4 bytes
#Input is (ID of motor to be read, address where data resides, data you want to write)
#There is no output
def WriteMotorData(motorID, data_address, data_inputs):
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(
    portHandler, motorID, data_address, data_inputs)
    # if dxl_comm_result != COMM_SUCCESS:
    #     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    # elif dxl_error != 0:
    #     print("%s" % packetHandler.getRxPacketError(dxl_error))

def dxlPresPos(dxlIDs: list[int])->list[int]:
    idNum = len(dxlIDs)
    dxl_present_position = [0] * idNum
    print("DXL IDs being read: ", dxlIDs)
    for id in range(idNum): #Reads the current position of the motor
        dxl_present_position[id] = ReadMotorData(dxlIDs[id], ADDR_PRESENT_POSITION)
    print("Present positions are: ", dxl_present_position)
    return (dxl_present_position)

def dxlPresAngle(dxlIDs):
    idNum = len(dxlIDs)
    dxl_present_position = [0] * idNum
    dxl_present_angle = [0] * idNum

    print("DXL IDs being read: ", dxlIDs)
    for id in range(idNum): #Reads the current position of the motor
        dxl_present_position[id] = ReadMotorData(dxlIDs[id], ADDR_PRESENT_POSITION)
    print("Present positions are: ", dxl_present_position)

    for id in range(idNum): #Converts the position into angles
        dxl_present_angle[id] = _map(dxl_present_position[id], 0, 4095, 0, 360)
    print("Present angles are: ", dxl_present_angle)
    print("-------------------------------------")
    return (dxl_present_angle)


def dxlSetVelo(vel_array, dxlIDs):
    if (len(vel_array) == len(dxlIDs)):
        idNum = len(dxlIDs)
        for id in range(idNum):
                    WriteMotorData(dxlIDs[id], ADDR_PROFILE_VELOCITY, vel_array[id])
    else:
        print("ERROR: Number of velocity inputs not matching with number of DXL ID inputs!")
    print("-------------------------------------")
    dxlGetVelo(dxlIDs)


def dxlGetVelo(dxlIDs):
    idNum = len(dxlIDs)
    dxl_present_velocity = [0] * idNum

    print("DXL IDs being read: ", dxlIDs)
    for id in range(idNum):
        dxl_present_velocity[id] = ReadMotorData(dxlIDs[id], ADDR_PROFILE_VELOCITY)
    print("Velocities are ", dxl_present_velocity)
    print("-------------------------------------")
    return (dxl_present_velocity)


def motorRunWithInputs(angle_inputs, dxlIDs):
    idNum = len(dxlIDs)

    #Format is [base, bicep, forearm, wrist, claw]
    if (len(angle_inputs) == idNum):
        dxl_goal_angle = angle_inputs
        dxl_goal_inputs = [0] * idNum
        dxl_end_position = [0] * idNum
        dxl_end_angle = [0] * idNum
        movementStatus = [0] * idNum

        print("Motors are rotating. DXL ID: ", dxlIDs)
        # ------------------Start to execute motor rotation------------------------
        while 1:
            #Convert angle inputs into step units for movement
            for id in range(idNum):
                dxl_goal_inputs[id] = _map(dxl_goal_angle[id], 0, 360, 0, 4095)
            print("Goal angles are ", dxl_goal_angle)

            #Write goal position for all motors (base, bicep, forearm, wrist, claw)
            for id in range(idNum):
                WriteMotorData(dxlIDs[id], ADDR_GOAL_POSITION, dxl_goal_inputs[id])

                #Read position for each motor and set status of motor
                dxl_end_position[id], movementStatus[id] = motor_check(dxlIDs[id], dxl_goal_inputs[id]) #Read position for the motor
                dxl_end_angle[id] = _map(dxl_end_position[id], 0, 4095, 0, 360)
            #print("Angle for Dynamixel:%03d is %03d" % (DXL_ID[device_index], dxl_end_angle[device_index]))

            for id in range(idNum):
                print("Angle for Dynamixels %03d after execution is %03d ----------------------------" % (dxlIDs[id], dxl_end_angle[id]))
            # ------------------------------------------------------------------------------------------------------------------------------------------------------

            #Motor movement completes and motor movement status to be sent out
            # ------------------------------------------------------------------------------------------------------------------------------------------------------
            print("-------------------------------------")
            return movementStatus
    else:
        print("ERROR: Number of angle inputs not matching with number of DXL ID inputs")


def simMotorRun(angle_inputs, dxlIDs):
    idNum = len(dxlIDs)
    movementStatus = [1] * idNum#Format is [base, bicep, forearm, wrist, claw]
    if (len(angle_inputs) == idNum):
        dxl_goal_angle = angle_inputs
        dxl_goal_inputs = [0] * idNum
        dxl_end_position = [0] * idNum
        dxl_end_angle = [0] * idNum
        movementStatus = [0] * idNum

        print("Motors are simultaneously rotating. DXL ID: ", dxlIDs)
        # ------------------Start to execute motor rotation------------------------
        while 1:
            #Convert angle inputs into step units for movement
            for id in range(idNum):
                dxl_goal_inputs[id] = _map(dxl_goal_angle[id], 0, 360, 0, 4095)
            print("Goal angles are ", dxl_goal_angle)

            simWrite(dxl_goal_inputs, dxlIDs)
            dxl_end_position, movementStatus = simPosCheck(dxl_goal_inputs, dxlIDs)
            for id in range(idNum):
                dxl_end_angle[id] = _map(dxl_end_position[id], 0, 4095, 0, 360)
            
            # for id in range(idNum):
            #     print("Angle for Dynamixel:%03d is %03d ----------------------------" % (dxlIDs[id], dxl_end_angle[id]))
            # ------------------------------------------------------------------------------------------------------------------------------------------------------
            print("-------------------------------------")
            return movementStatus
    else:
        print("ERROR: Number of angle inputs not matching with number of DXL ID inputs")
        return movementStatus

    #Format is [base, bicep, forearm, wrist, claw]
    if (len(angle_inputs) == idNum):
        dxl_goal_angle = angle_inputs
        dxl_goal_inputs = [0] * idNum
        dxl_end_position = [0] * idNum
        dxl_end_angle = [0] * idNum
        movementStatus = [0] * idNum

        print("Motors are simultaneously rotating. DXL ID: ", dxlIDs)
        # ------------------Start to execute motor rotation------------------------
        while 1:
            #Convert angle inputs into step units for movement
            for id in range(idNum):
                dxl_goal_inputs[id] = _map(dxl_goal_angle[id], 0, 360, 0, 4095)
            print("Goal angles are ", dxl_goal_angle)

            simWrite(dxl_goal_inputs, dxlIDs)
            dxl_end_position, movementStatus = simPosCheck(dxl_goal_inputs, dxlIDs)
            for id in range(idNum):
                dxl_end_angle[id] = _map(dxl_end_position[id], 0, 4095, 0, 360)
            
            # for id in range(idNum):
            #     print("Angle for Dynamixel:%03d is %03d ----------------------------" % (dxlIDs[id], dxl_end_angle[id]))
            # ------------------------------------------------------------------------------------------------------------------------------------------------------
            print("-------------------------------------")
            return movementStatus
    else:
        print("ERROR: Number of angle inputs not matching with number of DXL ID inputs")
        return movementStatus
    
# Close the port
def close_port():
    portHandler.closePort()

# Main function
def main():

    # Define motor ID
    BASE_ID = 1
    BICEP_ID = 2
    FOREARM_ID = 3
    WRIST_ID = 4
    CLAW_ID = 0

    # Define move mode and address for present position
    MOVEARM_MODE = 1
    ADDR_PRESENT_POSITION = 132

    # List of all motor IDs
    ALL_IDs = [BASE_ID, BICEP_ID, FOREARM_ID, WRIST_ID, CLAW_ID]
    MOVE_IDs = [BASE_ID, BICEP_ID, FOREARM_ID, WRIST_ID, CLAW_ID]
    
    # Define port number for Raspberry Pi
    PORT_NUM = '/dev/ttyUSB0'  # for rpi

    portInitialization(PORT_NUM,ALL_IDs)

    # Initialize port and enable torque
    initialize_port()
    enable_torque()

    while True:
        dxlSetVelo([10,40,80,100],[1,2,3,4])
        simMotorRun([0,10,100,1000],[1,2,3,4])
        print("Moving motors to target angles...")
        time.sleep(1)
        dxlSetVelo([100,80,40,10],[1,2,3,4])
        simMotorRun([200,100,10,0],[1,2,3,4])
        print("Moving motors to target angles...")
        time.sleep(1)
        if getch() == chr(0x1b):  # ESC to quit
            break

    # Disable torque and close port before exiting
    disable_torque()
    close_port()

if __name__ == "__main__":
    main()
