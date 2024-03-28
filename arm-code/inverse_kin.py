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

#angle for the max length reaching out in the x pos
max_length_angle = calculation.angle_Calc([375, 0, 50], 0)

#"[%s, %s, %s, %s]" % (int(baseTheta), int(shoulderTheta), int(elbowTheta), int(wristTheta))

def checkMovement(ids):
    motorStatus = [0] * len(ids)
    finished = [1] * len(ids)
    firstPosition = 0
    secondPosition = 0
    while True:
        for id in (ids):
            firstPosition = motor.ReadMotorData(id, ADDR_PRESENT_POSITION)
            time.sleep(.1)
            secondPosition = motor.ReadMotorData(id, ADDR_PRESENT_POSITION)
            if (abs(firstPosition - secondPosition) < 5):
                motorStatus[id] = 1
        if (motorStatus == finished):
            print("finished")
            break

def time_between_moves(velocity: list[int], ids: list[int], startAngles: list[int], endAngles: list[int]) -> None:
    motor.dxlSetVelo(velocity, ids)  # ALWAYS SET SPEED BEFORE ANYTHING
    time.sleep(0.1)
    print("Move 1")
    motor.simMotorRun(startAngles, ids)  # Reset claw looking up
    start_time = time.time()
    checkMovement(ids)
    check_time = time.time()
    print("Move 2")
    motor.simMotorRun(startAngles, ids)
    end_time = time.time()
    print("total time")
    print(end_time-start_time)
    print("check time")
    print(check_time-start_time)

def gcs_pullout():
    start_time = time.time()
    motor.dxlSetVelo([25, 25, 25, 25, 25], [0, 1, 2, 3, 4])  # ALWAYS SET SPEED BEFORE ANYTHING
    time.sleep(0.1)
    print("set up move")
    motor.simMotorRun([110, 223, 270, 47, 272], [0, 1, 2, 3, 4])  # Reset claw looking up
    time.sleep(2)

    print("move 1 move to chamber")
    motor.simMotorRun(max_length_angle, [1, 2, 3, 4])
    time.sleep(0.4)

    print("move 2 pitch wrist")
    motor.simMotorRun([200], [4])  # Reset claw looking up
    time.sleep(0.5)

    print("move 3 close grip")
    motor.simMotorRun([30], [0])  # Reset claw looking up
    time.sleep(0.1)

    print("move 4 pull forearm back")
    motor.simMotorRun([160], [3])  # Reset claw looking up
    time.sleep(0.15)

    inital_pull_out_angle = calculation.angle_Calc([300,0,60], 0)
    print("move 4 pull away slight")
    motor.simMotorRun(inital_pull_out_angle, [1, 2, 3, 4])
    time.sleep(0.15)

    second_pull_out_angle = calculation.angle_Calc([250,0,60], 0)
    print("move 5 pull away more")
    motor.simMotorRun(second_pull_out_angle, [1, 2, 3, 4])
    time.sleep(0.3)

    final_pull_out_angle = calculation.angle_Calc([200,0,60], 0)
    print("move 6 pull away even more")
    motor.simMotorRun(final_pull_out_angle, [1, 2, 3, 4])
    time.sleep(0.6)

    print("move 7 pull forearm back")
    motor.simMotorRun([45], [3])  # Reset claw looking up
    time.sleep(0.8)

    print("move 8 pull forearm back")
    motor.simMotorRun([200], [2])  # Reset claw looking up
    time.sleep(0.25)

    print("move 9 pull forearm back")
    motor.simMotorRun([220], [4])  # Reset claw looking up
    time.sleep(0.4)

    motor.dxlSetVelo([15, 15, 15, 15, 15], [0, 1, 2, 3, 4])  # ALWAYS SET SPEED BEFORE ANYTHING
    time.sleep(0.1)

    print("move 13 pull forearm back")
    motor.simMotorRun([265, 47, 170], [2, 3, 4])
    time.sleep(0.8)

    print("move 14 pull forearm back")
    motor.simMotorRun([270], [4])
    end_time = time.time()
    print(end_time-start_time)

def gcs_push_in():
    #Push In Battery
    start_time = time.time()
    motor.dxlSetVelo([15, 15, 15, 15, 15], [0, 1, 2, 3, 4])  # ALWAYS SET SPEED BEFORE ANYTHING
    time.sleep(0.1)

    print("move back to chamber")
    motor.simMotorRun([180, 62], [2, 3])
    time.sleep(2.5)

    motor.dxlSetVelo([50, 50, 50, 50, 50], [0, 1, 2, 3, 4])  # ALWAYS SET SPEED BEFORE ANYTHING
    time.sleep(0.1)

    gcs_push_in_angle = calculation.angle_Calc([310,0,65], 0)
    print(gcs_push_in_angle)
    print("push in to chamber")
    motor.simMotorRun(gcs_push_in_angle, [1, 2, 3, 4])
    time.sleep(0.15)

    print("push all the way in to chamber")
    motor.simMotorRun(max_length_angle, [1, 2, 3, 4])
    time.sleep(0.15)

    print("open claw")
    motor.simMotorRun([110], [0])  # Reset claw looking up
    time.sleep(0.15)

    gcs_pull_out_angle = calculation.angle_Calc([300,0,60], 0)
    print("move 4 pull away slight")
    motor.simMotorRun(gcs_pull_out_angle, [1, 2, 3, 4])
    time.sleep(1.5)

    print("set up move")
    motor.simMotorRun([110, 223, 270, 47, 272], [0, 1, 2, 3, 4])  # Reset claw looking up
    end_time = time.time()
    print(end_time-start_time)

if __name__ == "__main__":
    velocity = [25, 25, 25, 25, 25]
    ids = ALL_IDs
    startAngles = [110, 223, 270, 47, 272]
    endAngles = [40, 223, 270, 70, 180]
    time_between_moves(velocity=velocity, ids=ids, startAngles=startAngles,endAngles=endAngles)