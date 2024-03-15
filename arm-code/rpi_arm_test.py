# Raspberry Pi Arm Test
# Distance between GCS and BTP is 10 inches
# Distance between BVM and BTP is 12.5 inches
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
SERVER_HOST = '192.168.1.61'
SERVER_PORT = 3300

def checkMovement(ids):
    time.sleep(0.5)
    motorStatus = [0] * len(ids)
    finished = [1] * len(ids)
    firstPosition = 0
    secondPosition = 0
    while True:
        for id in (ids):
            firstPosition = motor.ReadMotorData(id, ADDR_PRESENT_POSITION)
            time.sleep(.1)
            secondPosition = motor.ReadMotorData(id, ADDR_PRESENT_POSITION)
            if (abs(firstPosition - secondPosition) < 2):
                motorStatus[id] = 1
        if (motorStatus == finished):
            print("finished")
            break
        
    
def pullout():
    print("pull out start")
    motor.dxlSetVelo([30, 30, 30, 30, 30], [0, 1, 2, 3, 4])  # ALWAYS SET SPEED BEFORE ANYTHING
    time.sleep(0.5)

    print("move 1")
    motor.simMotorRun([110, 223, 270, 47, 272], [0, 1, 2, 3, 4])  # Reset claw looking up
    checkMovement(MOVE_IDs)
    print("move fin")
    time.sleep(2)

    # move arm down
    print("move 2")
    motor.simMotorRun([168], [2])  # back to pull down more
    checkMovement(MOVE_IDs)
    print("move fin")
    time.sleep(2)

    #reach out
    print("move 3")
    motor.simMotorRun([150, 84, 269], [2, 3, 4])
    checkMovement(MOVE_IDs)
    print("move fin")
    time.sleep(2)

    #reach out further
    print("move 4")
    motor.simMotorRun([145, 122, 233], [2, 3, 4])
    checkMovement(MOVE_IDs)
    print("move fin")
    time.sleep(2)

    #close claw
    print("move 5")
    motor.simMotorRun([30], [0])
    checkMovement(MOVE_IDs)
    print("move fin")
    time.sleep(2)

    #pull down and back
    print("move 6")
    motor.simMotorRun([138, 100, 285], [2, 3, 4])
    checkMovement(MOVE_IDs)
    print("move fin")
    time.sleep(2)

    #pull down and back
    print("move 6.5")
    motor.simMotorRun([180, 100, 285], [2, 3, 4])
    checkMovement(MOVE_IDs)
    print("move fin")
    time.sleep(2)

    #pull up
    print("move 7")
    motor.simMotorRun([180, 50, 285], [2, 3, 4])
    checkMovement(MOVE_IDs)
    print("move fin")
    time.sleep(2)

    #pull up
    print("move 7")
    motor.simMotorRun([280, 50, 285], [2, 3, 4])
    checkMovement(MOVE_IDs)
    print("move fin")
    time.sleep(2)

    #pull all the way back
    print("move 8")
    motor.simMotorRun([280, 50, 170], [2, 3, 4])
    checkMovement(MOVE_IDs)
    print("move fin")
    time.sleep(2)

    # Look up
    print("move 9")
    motor.simMotorRun([275], [4])
    checkMovement(MOVE_IDs)
    print("move fin")
    time.sleep(2)

def pushin():
    print("Push In Start")
    motor.dxlSetVelo([30, 30, 30, 30, 30], [0, 1, 2, 3, 4])  # ALWAYS SET SPEED BEFORE ANYTHING
    time.sleep(0.5)

    print("move 1")
    motor.simMotorRun([170], [4])
    checkMovement(MOVE_IDs)
    print("move fin")
    time.sleep(2)

    print("move 2")
    motor.simMotorRun([153, 50, 285], [2, 3, 4])
    checkMovement(MOVE_IDs)
    print("move fin")
    time.sleep(2)

    print("move 3")
    motor.simMotorRun([138, 75, 285], [2, 3, 4])
    checkMovement(MOVE_IDs)
    print("move fin")
    time.sleep(2)

    print("move 4")
    motor.simMotorRun([145, 122, 233], [2, 3, 4])
    checkMovement(MOVE_IDs)
    print("move fin")
    time.sleep(2)

    print("move 5")
    motor.simMotorRun([110], [0])
    checkMovement(MOVE_IDs)
    print("move fin")
    time.sleep(2)

    print("move 6")
    motor.simMotorRun([145, 122, 233], [2, 3, 4])
    checkMovement(MOVE_IDs)
    print("move fin")
    time.sleep(2)

    print("move 7")
    motor.simMotorRun([150, 84, 269], [2, 3, 4])
    checkMovement(MOVE_IDs)
    print("move fin")
    time.sleep(2)

while True:
    uin = int(input("Push(0) or Pull (1)"))
    if uin == 0: pushin()
    elif uin == 1: pullout()
    else: print("Invalid Input")