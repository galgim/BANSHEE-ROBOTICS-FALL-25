#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
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

from dynamixel_sdk import *  # Uses Dynamixel SDK library

# ********* DYNAMIXEL Model definition *********
MY_DXL = 'X_SERIES'  # X330 (5.0 V recommended), X430, X540, 2X430

# Control table address
if MY_DXL == 'X_SERIES' or MY_DXL == 'MX_SERIES':
    ADDR_TORQUE_ENABLE = 64
    ADDR_PROFILE_VELOCITY = 112 #Address of the velocity of the motors
    ADDR_GOAL_POSITION = 116
    ADDR_PRESENT_POSITION = 132
    DXL_MINIMUM_POSITION_VALUE = 0  # Refer to the Minimum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE = 2000  # Refer to the Maximum Position Limit of product eManual
    BAUDRATE = 57600

# DYNAMIXEL Protocol Version (1.0 / 2.0)
PROTOCOL_VERSION = 2.0

# Factory default IDs of the DYNAMIXEL motors
DXL_IDs = [1, 2, 3,4]  # Add as many motor IDs as required

# Use the actual port assigned to the U2D2
DEVICENAME = '/dev/ttyUSB0'

TORQUE_ENABLE = 1  # Value for enabling the torque
TORQUE_DISABLE = 0  # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20  # Dynamixel moving status threshold

index = 0
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]  # Goal position

# Initialize PortHandler instance
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
packetHandler = PacketHandler(PROTOCOL_VERSION)
def ReadMotorData(motorID, data_address):
    data_value, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(
        portHandler, motorID, data_address)
    # if dxl_comm_result != COMM_SUCCESS:
    #     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    # elif dxl_error != 0:
    #     print("%s" % packetHandler.getRxPacketError(dxl_error))
    return data_value
def WriteMotorData(motorID, data_address, data_inputs):
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(
    portHandler, motorID, data_address, data_inputs)
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
def checkMovement(ids):
    motorStatus = [0] * len(ids)
    finished = [1] * len(ids)
    firstPosition = 0
    secondPosition = 0
    while True:
        for id in ids:
            firstPosition = ReadMotorData(id, ADDR_PRESENT_POSITION)
            time.sleep(0.1)
            secondPosition = ReadMotorData(id, ADDR_PRESENT_POSITION)
            if abs(firstPosition - secondPosition) < 5:
                motorStatus[id] = 1
        if motorStatus == finished:
            print("finished")
            break
def _map(x, in_min, in_max, out_min, out_max):
    return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)
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

def simMotorRun(angle_inputs, dxlIDs):
    idNum = len(dxlIDs)
    movementStatus = [1] * idNum
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

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

# Enable Dynamixel Torque for each motor
for DXL_ID in DXL_IDs:
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print(f"Dynamixel {DXL_ID} has been successfully connected")

while 1:
    print("Press any key to continue! (or press ESC to quit!)")
    if getch() == chr(0x1b):
        break
    checkMovement(DXL_ID)
    dxlSetVelo([30,30,30,30],[1,2,3,4])
    # Write goal position for each motor
    for DXL_ID in DXL_IDs:
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, dxl_goal_position[index])
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

    while 1:
        all_motors_reached_goal = True
        for DXL_ID in DXL_IDs:
            # Read present position for each motor
            dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))

            print(f"[ID:{DXL_ID:03d}] GoalPos:{dxl_goal_position[index]:03d}  PresPos:{dxl_present_position:03d}")

            if abs(dxl_goal_position[index] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD:
                all_motors_reached_goal = False

        if all_motors_reached_goal:
            break

    # Change goal position
    if index == 0:
        index = 1
    else:
        index = 0

# Disable Dynamixel Torque for each motor
for DXL_ID in DXL_IDs:
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

# Close port
portHandler.closePort()
