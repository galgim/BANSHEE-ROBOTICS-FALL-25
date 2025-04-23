import os
import sys
import time

from dynamixel_sdk import (
    PortHandler,
    PacketHandler,
    GroupSyncWrite,
    GroupSyncRead,
    DXL_LOBYTE,
    DXL_LOWORD,
    DXL_HIBYTE,
    DXL_HIWORD
)

# ---------------------- CONSTANTS ----------------------
PROTOCOL_VERSION = 2      # Dynamixel protocol version (1 or 2)
BAUDRATE = 1000000        # Port baudrate

# Control table addresses
ADDR_PRESENT_POSITION = 132
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_MOVING = 122         # Moving flag (1: moving, 0: stopped)
ADDR_MOVING_STATUS = 123  # Moving status (1: goal reached, 0: not yet)
ADDR_TORQUE_ENABLE = 64

# Data lengths and thresholds
LEN_GOAL_POSITION = 4
LEN_PRESENT_POSITION = 4
DXL_MOVING_STATUS_THRESHOLD = 60

# Torque values
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

# ---------------------- INPUT ----------------------
def getch():
    """
    Read a single character from stdin without echo.
    """
    if os.name == 'nt':
        import msvcrt
        return msvcrt.getch().decode()
    import tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

# ---------------------- PORT MANAGEMENT ----------------------
def portInitialization(portname: str, dxlIDs: list[int]) -> None:
    """
    Open serial port and enable torque on all specified motors.
    """
    global portHandler, packetHandler, DXL_ID
    DXL_ID = dxlIDs

    portHandler = PortHandler(portname)
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    if not portHandler.openPort():
        print("Failed to open port")
        getch(); sys.exit(1)
    if not portHandler.setBaudRate(BAUDRATE):
        print("Failed to set baudrate")
        getch(); sys.exit(1)

    for mid in DXL_ID:
        packetHandler.write1ByteTxRx(portHandler, mid, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)


def portTermination() -> None:
    """
    Disable torque on all motors and close the port.
    """
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
    return data


def WriteMotorData(motorID: int, address: int, value: int) -> None:
    """Write a 4-byte value to the motor's control table."""
    packetHandler.write4ByteTxRx(portHandler, motorID, address, value)

# ---------------------- GETTERS/SETTERS ----------------------
def dxlPresPos(dxlIDs: list[int]) -> list[int]:
    """Return current positions for each motor."""
    return [ReadMotorData(mid, ADDR_PRESENT_POSITION) for mid in dxlIDs]


def dxlPresAngle(dxlIDs: list[int]) -> list[int]:
    """Return current angles (in degrees) for each motor."""
    return [_map(pos, 0, 4095, 0, 360) for pos in dxlPresPos(dxlIDs)]


def dxlGetVelo(dxlIDs: list[int]) -> list[int]:
    """Return current profile velocities for each motor."""
    return [ReadMotorData(mid, ADDR_PROFILE_VELOCITY) for mid in dxlIDs]


def dxlSetVelo(vel_array: list[int], dxlIDs: list[int]) -> None:
    """Set profile velocity for each motor."""
    if len(vel_array) != len(dxlIDs):
        raise ValueError("Length mismatch between velocities and DXL IDs")
    for mid, vel in zip(dxlIDs, vel_array):
        WriteMotorData(mid, ADDR_PROFILE_VELOCITY, vel)

# ---------------------- MOTOR POLLING ----------------------
def motor_check(motorIndex: int, goal_position: int) -> tuple[int, int]:
    """
    Poll a single motor until it stops moving using the Moving flag.

    Returns (final_position, status) where status=1 means within goal threshold.
    """
    # Wait until moving flag goes low (0)
    while True:
        moving, _, _ = packetHandler.read1ByteTxRx(portHandler, motorIndex, ADDR_MOVING)
        if moving == 0:
            break
        time.sleep(0.01)
    final_pos = ReadMotorData(motorIndex, ADDR_PRESENT_POSITION)
    # Determine if final position is within threshold of goal
    status = 1 if abs(goal_position - final_pos) <= DXL_MOVING_STATUS_THRESHOLD else 0
    return final_pos, status

# ---------------------- SEQUENTIAL MOVE ----------------------
def motorRun(angle_inputs: list[float], dxlIDs: list[int]) -> list[int]:
    """
    Move each motor sequentially to its target angle.

    Returns list of statuses for each motor.
    """
    if len(angle_inputs) != len(dxlIDs):
        raise ValueError("Length mismatch between angles and DXL IDs")

    statuses: list[int] = []
    for mid, angle in zip(dxlIDs, angle_inputs):
        goal = _map(angle, 0, 360, 0, 4095)
        WriteMotorData(mid, ADDR_GOAL_POSITION, goal)
        _, stat = motor_check(mid, goal)
        statuses.append(stat)
    return statuses

# ---------------------- SIMULTANEOUS MOVE ----------------------
def simWrite(goals: list[int], dxlIDs: list[int]) -> None:
    """SyncWrite goal positions to all motors."""
    sync = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)
    for mid, goal in zip(dxlIDs, goals):
        params = [
            DXL_LOBYTE(DXL_LOWORD(goal)), DXL_HIBYTE(DXL_LOWORD(goal)),
            DXL_LOBYTE(DXL_HIWORD(goal)), DXL_HIBYTE(DXL_HIWORD(goal))
        ]
        sync.addParam(mid, params)
    sync.txPacket()
    sync.clearParam()


def simPosCheck(dxl_goal_inputs: list[int], dxlIDs: list[int]) -> tuple[list[int], list[int]]:
    """
    Check positions of all motors until movement completes.

    Uses GroupSyncRead and stability count based on position deltas.
    Returns (final_positions, statuses).
    """
    # Initialize sync reader for present positions
    reader = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
    for mid in dxlIDs:
        reader.addParam(mid)

    # Initial read
    reader.txRxPacket()
    prev_positions = [reader.getData(mid, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION) for mid in dxlIDs]
    stable_counts = [0]*len(dxlIDs)
    statuses = [0]*len(dxlIDs)

    while True:
        reader.txRxPacket()
        positions = [reader.getData(mid, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION) for mid in dxlIDs]
        # Update stability counts and statuses
        for i, (pos, prev, goal) in enumerate(zip(positions, prev_positions, dxl_goal_inputs)):
            if abs(pos - prev) < 2:
                stable_counts[i] += 1
            else:
                stable_counts[i] = 0
            if stable_counts[i] >= 10 and abs(pos - goal) <= DXL_MOVING_STATUS_THRESHOLD:
                statuses[i] = 1
        # If all motors stable at goal, break
        if all(statuses):
            break
        prev_positions = positions
        time.sleep(0.05)

    return positions, statuses


def simMotorRun(angle_inputs: list[float], dxlIDs: list[int]) -> list[int]:
    """
    Simultaneously move motors and wait until all movements complete.

    Returns list of statuses for each motor.
    """
    goals = [_map(a, 0, 360, 0, 4095) for a in angle_inputs]
    simWrite(goals, dxlIDs)
    final_positions, statuses = simPosCheck(goals, dxlIDs)
    return statuses
