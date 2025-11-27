#!/usr/bin/env python3
import os
import sys
import time

from dynamixel_sdk import (
    PortHandler,
    PacketHandler,
    GroupSyncWrite,
    DXL_LOBYTE,
    DXL_LOWORD,
    DXL_HIBYTE,
    DXL_HIWORD
)

# ---------------------- CONSTANTS ----------------------
PROTOCOL_VERSION = 2         # Dynamixel protocol version (1 or 2)
BAUDRATE = 1_000_000         # Port baudrate

# Control table addresses
ADDR_TORQUE_ENABLE         = 64
ADDR_PROFILE_ACCELERATION  = 108  # <— new
ADDR_PROFILE_VELOCITY      = 112
ADDR_GOAL_POSITION         = 116
ADDR_MOVING                = 122  # Moving flag (1: moving, 0: stopped)
ADDR_PRESENT_POSITION      = 132

# Data lengths and thresholds
LEN_PROFILE_ACCELERATION   = 4
LEN_GOAL_POSITION          = 4
LEN_PRESENT_POSITION       = 4
DXL_MOVING_STATUS_THRESHOLD = 60  # ticks (unused in sim now but kept)

# Torque values
TORQUE_ENABLE  = 1
TORQUE_DISABLE = 0

# ---------------------- INPUT ----------------------
def getch():
    """Read a single character from stdin without echo."""
    if os.name == 'nt':
        import msvcrt
        return msvcrt.getch().decode()
    import tty, termios
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

# ---------------------- PORT MANAGEMENT ----------------------
def portInitialization(portname: str, dxlIDs: list[int]) -> None:
    """
    Open serial port, enable torque, and set accel on all specified motors.
    """
    global portHandler, packetHandler, DXL_ID
    DXL_ID = dxlIDs

    portHandler = PortHandler(portname)
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    if not portHandler.openPort():
        print("Failed to open port"); getch(); sys.exit(1)
    if not portHandler.setBaudRate(BAUDRATE):
        print("Failed to set baudrate"); getch(); sys.exit(1)

    # enable torque and set acceleration
    for mid in DXL_ID:
        packetHandler.write1ByteTxRx(portHandler, mid, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        packetHandler.write4ByteTxRx(portHandler, mid,
                                     ADDR_PROFILE_ACCELERATION,
                                     50)  # tune this value as needed

def portTermination() -> None:
    """Disable torque on all motors and close the port."""
    for mid in DXL_ID:
        packetHandler.write1ByteTxRx(portHandler, mid, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    portHandler.closePort()

# ---------------------- UTILITY ----------------------
def _map(x: float, in_min: float, in_max: float, out_min: float, out_max: float) -> int:
    """Map a value from one range to another."""
    return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

# ---------------------- RAW READ/WRITE ----------------------
def ReadMotorData(motorID: int, address: int) -> int:
    """Read a 4-byte value from the motor's control table."""
    data, _, _ = packetHandler.read4ByteTxRx(portHandler, motorID, address)
    print(f"Data: {data}")
    return data

def WriteMotorData(motorID: int, address: int, value: int) -> None:
    """Write a 4-byte value to the motor's control table."""
    packetHandler.write4ByteTxRx(portHandler, motorID, address, value)

# ---------------------- GETTERS ----------------------
def dxlPresPos(dxlIDs: list[int]) -> list[int]:
    return [ReadMotorData(mid, ADDR_PRESENT_POSITION) for mid in dxlIDs]

def dxlPresAngle(dxlIDs: list[int]) -> list[int]:
    return [_map(pos, 0, 4095, 0, 360) for pos in dxlPresPos(dxlIDs)]

def dxlSetVelo(vel_array, dxlIDs):
    if (len(vel_array) == len(dxlIDs)):
        idNum = len(dxlIDs)
        for id in range(idNum):
                    WriteMotorData(dxlIDs[id], ADDR_PROFILE_VELOCITY, vel_array[id])
    else:
        print("ERROR: Number of velocity inputs not matching with number of DXL ID inputs!")
    print("-------------------------------------")

# ---------------------- POLLING ----------------------
def motor_check(motorID: int, goal_pos: int) -> tuple[int,int]:
    """
    Wait until a single motor's Moving flag == 0, then return (pos,status).
    """
    while True:
        moving, _, _ = packetHandler.read1ByteTxRx(portHandler, motorID, ADDR_MOVING)
        if moving == 0:
            break
        time.sleep(0.01)

    final = ReadMotorData(motorID, ADDR_PRESENT_POSITION)
    status = 1 if abs(goal_pos - final) <= DXL_MOVING_STATUS_THRESHOLD else 0
    return final, status

def wait_until_stop(dxlIDs: list[int], poll: float = 0.05) -> None:
    """Block until all motors report Moving==0."""
    while True:
        if all(packetHandler.read1ByteTxRx(portHandler, mid, ADDR_MOVING)[0] == 0
               for mid in dxlIDs):
            return
        time.sleep(poll)

# ---------------------- MOVES ----------------------
def motorRun(angle_inputs: list[float], dxlIDs: list[int]) -> list[int]:
    """
    Sequential moves with moving-flag polling.
    """
    if len(angle_inputs) != len(dxlIDs):
        raise ValueError("Length mismatch between angles and DXL IDs")

    statuses = []
    for mid, angle in zip(dxlIDs, angle_inputs):
        goal = _map(angle, 0, 360, 0, 4095)
        WriteMotorData(mid, ADDR_GOAL_POSITION, goal)
        _, stat = motor_check(mid, goal)
        statuses.append(stat)

    print(f"Target angles : {angle_inputs}")
    print(f"Current angles: {dxlPresAngle(dxlIDs)}")
    return statuses


def simMotorRun(angle_inputs: list[float], dxlIDs: list[int]) -> list[int]:
    """
    Simultaneous move: write all goals, then wait on Moving flag.
    """
    if len(angle_inputs) != len(dxlIDs):
        raise ValueError("Length mismatch between angles and DXL IDs")

    # build goals
    goals = [_map(a, 0, 360, 0, 4095) for a in angle_inputs]

    # sync write
    sync = GroupSyncWrite(portHandler, packetHandler,
                          ADDR_GOAL_POSITION, LEN_GOAL_POSITION)
    co = int(dxlPresAngle(dxlIDs)[0] / 360)
    for mid, goal in zip(dxlIDs, goals):
        if mid == 0:
            goal = goal + 4096 * co
        # print(mid)
        # print(goal)
        params = [
            DXL_LOBYTE(DXL_LOWORD(goal)), DXL_HIBYTE(DXL_LOWORD(goal)),
            DXL_LOBYTE(DXL_HIWORD(goal)), DXL_HIBYTE(DXL_HIWORD(goal))
        ]
        sync.addParam(mid, params)
    sync.txPacket()
    sync.clearParam()

    # wait for all to stop
    wait_until_stop(dxlIDs)

    print(f"Target angles : {angle_inputs}")
    # print(f"Current angles: {dxlPresAngle(dxlIDs)}")
    return [1]*len(dxlIDs)

# ---------------------- EXAMPLE USAGE ----------------------
if __name__ == '__main__':
    port_name = '/dev/ttyUSB0'
    motor_ids = [1, 2, 3, 4]  # your IDs

    portInitialization(port_name, motor_ids)
    try:
        print("Sequential move statuses:", motorRun([45, 90, 135, 180], motor_ids))
        print("Simultaneous move statuses:", simMotorRun([10, 20, 30, 40], motor_ids))
    finally:
        portTermination()
