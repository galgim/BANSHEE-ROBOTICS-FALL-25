import math
import motorctrl_v1 as motor
import Movement_Calc_v2 as calculation
import numpy as np
import time
import serial
import cv2
import socket
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)
GPIO.setup(22, GPIO.OUT)
GPIO.output(22, GPIO.LOW)

BASE_ID = 1
BICEP_ID = 2
FOREARM_ID = 3
WRIST_ID = 4
CLAW_ID = 0


# PORT_NUM = '/dev/cu.usbserial-FT5NY9DI'  #for mac
PORT_NUM = '/dev/ttyUSB0'  # for rpi
ser = serial.Serial('/dev/ttyUSB1', 9600, timeout=1)  # for rpi

BAUDRATE = 1000000
MOVEARM_MODE = 1
ADDR_PRESENT_POSITION = 132
ALL_IDs = [BASE_ID, BICEP_ID, FOREARM_ID, WRIST_ID, CLAW_ID]
MOVE_IDs = [BASE_ID, BICEP_ID, FOREARM_ID, WRIST_ID, CLAW_ID]

motor.portInitialization(PORT_NUM, ALL_IDs)
max_length_angle = calculation.angle_Calc([375, 0, 50], 0)

def debug_gcs_pull_out():
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
    motor.simMotorRun([49], [3])  # Reset claw looking up
    time.sleep(0.8)

    print("move 8 pull forearm back")
    motor.simMotorRun([200], [2])  # Reset claw looking up
    time.sleep(0.25)

    print("move 9 pull forearm back")
    motor.simMotorRun([220], [4])  # Reset claw looking up
    time.sleep(0.4)

    motor.dxlSetVelo([15, 15, 30, 15, 15], [0, 1, 2, 3, 4])  # ALWAYS SET SPEED BEFORE ANYTHING
    time.sleep(0.1)

    print("move 13 pull forearm back")
    motor.simMotorRun([265, 47, 170], [2, 3, 4])
    time.sleep(0.8)

    print("move 14 pull forearm back")
    motor.simMotorRun([270], [4])
    end_time = time.time()
    print(end_time-start_time)

def debug_gcs_push_in():
    #Push In Battery
    start_time = time.time()
    motor.dxlSetVelo([15, 15, 15, 15, 15], [0, 1, 2, 3, 4])  # ALWAYS SET SPEED BEFORE ANYTHING
    time.sleep(0.1)
    #265, 47, 170
    print("move back to chamber1")
    motor.simMotorRun([270, 54], [2, 3])
    time.sleep(2.5)

    print("move back to chamber2")
    motor.simMotorRun([180, 56], [2, 3])
    time.sleep(3)

    print("move back to chamber2")
    motor.simMotorRun([180, 62], [2, 3])
    time.sleep(2.5)
    motor.dxlSetVelo([40, 40, 40, 40, 40], [0, 1, 2, 3, 4])  # ALWAYS SET SPEED BEFORE ANYTHING
    time.sleep(0.1)

    gcs_push_in_angle = calculation.angle_Calc([310,0,70], 0)
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

bvm_max_length_angle = calculation.angle_Calc([370, 0, 70], 0)

def debug_bvm_pull_out():
    start_time = time.time()
    motor.dxlSetVelo([30, 30, 30, 30, 30], [0, 1, 2, 3, 4])  # ALWAYS SET SPEED BEFORE ANYTHING
    time.sleep(0.1)
    print("set up move")
    motor.simMotorRun([110, 225, 270, 50, 278], [0, 1, 2, 3, 4])  # Reset claw looking up
    time.sleep(2)

    print("move 1 move to chamber")
    motor.simMotorRun(bvm_max_length_angle, [1, 2, 3, 4])
    time.sleep(0.4)

    print("move 2 pitch wrist")
    motor.simMotorRun([200], [4])  # Reset claw looking up
    time.sleep(0.5)

    print("move 3 close grip")
    motor.simMotorRun([30], [0])  # Reset claw looking up
    time.sleep(0.5)

    for i in range(280,190,-10):
        print(i)
        initial_pull_out_angle = calculation.angle_Calc([i,0,66], 0)
        print("move 4 pull away slight")
        motor.simMotorRun(initial_pull_out_angle, [1, 2, 3, 4])
        time.sleep(1.5)
        motor.dxlPresPos([0, 1, 2, 3, 4])

    print("move 5 pull away more")
    motor.simMotorRun([225, 162, 72, 269], [1, 2, 3, 4])
    time.sleep(0.3)
    motor.dxlPresPos([0, 1, 2, 3, 4])

    print("move 6 pull away more")
    motor.simMotorRun([225, 175, 72, 269], [1, 2, 3, 4])
    time.sleep(0.3)
    motor.dxlPresPos([0, 1, 2, 3, 4])

    print("move 7 pull away more")
    motor.simMotorRun([225, 179, 67, 269], [1, 2, 3, 4])
    time.sleep(1)
    motor.dxlPresPos([0, 1, 2, 3, 4])
    
    print("move 8 pull away more")
    for i in range(180,220,10):
        print(i)
        motor.simMotorRun([225, i, 60, 269], [1, 2, 3, 4])
        time.sleep(0.025)
        motor.dxlPresPos([0, 1, 2, 3, 4])
    
    print("move 9 pull away more")
    motor.simMotorRun([225, 265, 60, 269], [1, 2, 3, 4])
    time.sleep(0.2)
    motor.dxlPresPos([0, 1, 2, 3, 4])

    print("move 10 pull away more")
    motor.simMotorRun([225, 265, 45, 269], [1, 2, 3, 4])
    time.sleep(0.2)
    motor.dxlPresPos([0, 1, 2, 3, 4])

    print("set up move")
    motor.simMotorRun([30, 223, 270, 47, 272], [0, 1, 2, 3, 4])  # Reset claw looking up
    time.sleep(2)

def debug_bvm_push_in():
    #Push In Battery
    start_time = time.time()
    motor.dxlSetVelo([15, 1, 15, 15, 15], [0, 1, 2, 3, 4])  # ALWAYS SET SPEED BEFORE ANYTHING
    time.sleep(0.1)
    motor.simMotorRun([225],[1])
    time.sleep(.1)
    print("move back to chamber")
    motor.simMotorRun([220, 62], [2, 3])
    time.sleep(2)

    motor.simMotorRun([200, 62], [2, 3])
    time.sleep(1)

    motor.simMotorRun([190, 62], [2, 3])
    time.sleep(1)

    print("move back to chamber")
    motor.simMotorRun([180, 62], [2, 3])
    time.sleep(0.5)

    print("move back to chamber")
    motor.simMotorRun([170, 62], [2, 3])
    time.sleep(0.5)

    print("move back to chamber")
    motor.simMotorRun([160, 62], [2, 3])
    time.sleep(0.5)


    print("move back to chamber")
    motor.simMotorRun([150, 62], [2, 3])
    time.sleep(0.5)

    print("move back to chamber")
    motor.simMotorRun([140, 62], [2, 3])
    time.sleep(0.5)

    gcs_push_in_angle = calculation.angle_Calc([200,0,65], 0)
    print(gcs_push_in_angle)
    print("push in to chamber")
    motor.simMotorRun(gcs_push_in_angle, [1, 2, 3, 4])
    time.sleep(0.5)
    #[225, 160, 78, 266]
    gcs_push_in_angle = calculation.angle_Calc([250,0,70], 0)
    print(gcs_push_in_angle)
    print("push in to chamber")
    motor.simMotorRun(gcs_push_in_angle, [1, 2, 3, 4])
    time.sleep(0.5)

    #[225, 116, 191, 196]
    gcs_push_in_angle = calculation.angle_Calc([370,0,80], 0)
    print(gcs_push_in_angle)
    print("push in to chamber")
    motor.simMotorRun(gcs_push_in_angle, [1, 2, 3, 4])
    time.sleep(0.5)

    gcs_push_in_angle = calculation.angle_Calc([375,0,80], 0)
    print(gcs_push_in_angle)
    print("push in to chamber")
    motor.simMotorRun(gcs_push_in_angle, [1, 2, 3, 4])
    time.sleep(0.5)

    motor.simMotorRun([30, 225, 81, 236, 186], [0, 1, 2, 3, 4])
    time.sleep(0.5)

    motor.simMotorRun([100],[0])
    time.sleep(0.025)
    motor.simMotorRun([90],[4])
    time.sleep(0.025)
    motor.simMotorRun([110,270],[2,3])
    time.sleep(0.5)

    print("set up move")
    motor.simMotorRun([110, 225, 270, 47, 272], [0, 1, 2, 3, 4])  # Reset claw looking up
    time.sleep(2)

arduinoinput = ''

# Take Battery from GCS
debug_gcs_pull_out()

print("Start Arduino Code")
GPIO.output(22, GPIO.HIGH)
time.sleep(8)
ser.write(b'b')  # Tell Arduino it's good to go

# Wait for arduino to send s, means it has arrived at BVM
while True:
    print("waiting for s")
    response = ser.readline().strip()
    print(arduinoinput)
    arduinoinput = response.decode()
    if arduinoinput == 's':
        print("push battery into BVM!")
        break


# Push battery into BVM
ser.flush()
time.sleep(3)
debug_bvm_push_in()
print("BVM Push in")

time.sleep(5)  # let BVM cycle battery

# Take battery out of BVM
debug_bvm_pullout()
print("sending b to arduino")
ser.write(b'g')  # Tell Arduino it's good to go
time.sleep(0.5)
response = ser.readline().strip().decode()
print(response)
# Wait for arduino to send s, means it has arrived at GCS
while True:
    response = ser.readline().strip()
    arduinoinput = response.decode()
    if arduinoinput == 's':
        print("push battery into GCS!")
        break


# Push battery into GCS

debug_gcs_push_in()

GPIO.cleanup()
