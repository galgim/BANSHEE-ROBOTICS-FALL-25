from dynamixel_sdk import *  # Dynamixel SDK library for motor control
import time
import math

# Control table addresses for Dynamixel XM series (4-byte values)
ADDR_TORQUE_ENABLE             = 64   # Torque ON/OFF address
ADDR_OPERATING_MODE            = 11   # Operating mode address
ADDR_PRESENT_POSITION          = 132  # Present position address (reads current position)
ADDR_GOAL_POSITION             = 116  # Goal position address (writes target position)
ADDR_PROFILE_VELOCITY          = 112  # Profile velocity address (sets motion speed)
LEN_GOAL_POSITION              = 4    # Byte length of goal position data
LEN_PRESENT_POSITION           = 4    # Byte length of present position data

# Protocol and mode constants
PROTOCOL_VERSION                       = 2  # Use protocol 2.0
TORQUE_ENABLE                          = 1  # Value to enable torque
OPERATING_MODE_CURRENT_BASED_POSITION = 5  # Current-based position control mode
DXL_MOVING_STATUS_THRESHOLD           = 50 # Threshold to consider movement complete (in ticks)

# Triangular profile parameters
MIN_SPEED_RATIO        = 0.1  # Start/end speed as fraction of cruise speed
RAMP_UP_FRACTION       = 0.1  # Fraction of distance spent ramping up
RAMP_DOWN_FRACTION     = 0.4  # Fraction of distance spent ramping down

# Global handles (initialized in portInitialization)
portHandler      = None
packetHandler    = None
motor_sync_write = None
motor_sync_read  = None
initialized_motor_ids = []


def _map(x, in_min, in_max, out_min, out_max):
    """
    Map value x from one range ([in_min, in_max]) to another ([out_min, out_max]).
    """
    return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)


def portInitialization(portname, dxlIDs):
    """
    Configure serial port and initialize each motor:
    - Disable torque
    - Set to current-based position mode
    - Enable torque
    """
    global portHandler, packetHandler, initialized_motor_ids
    initialized_motor_ids = dxlIDs
    portHandler = PortHandler(portname)
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    if not portHandler.openPort():
        raise RuntimeError("Failed to open port")
    if not portHandler.setBaudRate(1000000):
        raise RuntimeError("Failed to set baudrate")

    for motor_id in dxlIDs:
        packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR_TORQUE_ENABLE, 0)
        packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR_OPERATING_MODE, OPERATING_MODE_CURRENT_BASED_POSITION)
        packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)


def WriteMotorData(motor_id, address, value):
    """Write a 4-byte value to a motor."""
    packetHandler.write4ByteTxRx(portHandler, motor_id, address, value)


def ReadMotorData(motor_id, address):
    """Read a 4-byte value from a motor."""
    data_value, _, _ = packetHandler.read4ByteTxRx(portHandler, motor_id, address)
    return data_value


def dxlSetVelo(velocity_list, dxlIDs):
    """Set profile velocity for each motor."""
    for velocity, motor_id in zip(velocity_list, dxlIDs):
        WriteMotorData(motor_id, ADDR_PROFILE_VELOCITY, velocity)


def dxlGetVelo(dxlIDs):
    """Return current profile velocities."""
    return [ReadMotorData(motor_id, ADDR_PROFILE_VELOCITY) for motor_id in dxlIDs]


def simWrite(goal_positions, dxlIDs):
    """Send goal positions using GroupSyncWrite."""
    global motor_sync_write, motor_sync_read
    motor_sync_write = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)
    motor_sync_read  = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

    for motor_id in dxlIDs:
        motor_sync_read.addParam(motor_id)
    for position, motor_id in zip(goal_positions, dxlIDs):
        position_bytes = [
            DXL_LOBYTE(DXL_LOWORD(position)),
            DXL_HIBYTE(DXL_LOWORD(position)),
            DXL_LOBYTE(DXL_HIWORD(position)),
            DXL_HIBYTE(DXL_HIWORD(position))
        ]
        motor_sync_write.addParam(motor_id, position_bytes)
    motor_sync_write.txPacket()
    motor_sync_write.clearParam()


def simPosCheck(target_positions, dxlIDs, timeout=2.0):
    """Wait until motors reach goal positions or timeout, with debug output."""
    start_time = time.time()
    while True:
        motor_sync_read.txRxPacket()
        current_positions = [
            motor_sync_read.getData(motor_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            for motor_id in dxlIDs
        ]
        # Debug print of targets vs actuals
        print(f"[simPosCheck] Targets: {target_positions}")
        print(f"[simPosCheck] Currents: {current_positions}")
        if all(abs(curr - target) < DXL_MOVING_STATUS_THRESHOLD for curr, target in zip(current_positions, target_positions)):
            print("[simPosCheck] All within threshold, exiting check.")
            return
        if time.time() - start_time > timeout:
            print("[simPosCheck] Timeout reached, exiting check.")
            return
        time.sleep(0.1)


def simMotorRun(angle_inputs, dxlIDs, update_interval=0.05):
    """
    Move motors with a sinusoidal ramp-up and ramp-down velocity profile,
    priming with min_vel to avoid an initial jolt, and return as soon as
    all are within DXL_MOVING_STATUS_THRESHOLD, then verify and correct.
    """
    # 1) Compute goal positions and distances
    goal_positions  = [_map(angle, 0, 360, 0, 4095) for angle in angle_inputs]
    start_positions = [ReadMotorData(mid, ADDR_PRESENT_POSITION) for mid in dxlIDs]
    total_distances = [abs(goal - start) for goal, start in zip(goal_positions, start_positions)]

    # 2) Estimate cruise and min velocities
    current_vels = dxlGetVelo(dxlIDs)
    travel_times = [(dist / v if v > 0 else 0) for dist, v in zip(total_distances, current_vels)]
    common_time  = max(max(travel_times), 1e-3)
    cruise_vels  = [max(int(dist / common_time), 1) for dist in total_distances]
    min_vels     = [max(int(v * MIN_SPEED_RATIO), 1) for v in cruise_vels]

    # —— prime to slow start velocity to prevent initial jolt ——
    print(f"[simMotorRun] Priming velocities: {min_vels} for IDs {dxlIDs}")
    dxlSetVelo(min_vels, dxlIDs)

    # 3) Send the first goal
    print(f"[simMotorRun] Sending initial goal positions: {goal_positions} for IDs {dxlIDs}")
    simWrite(goal_positions, dxlIDs)

    # 4) Ramp loop with sinusoidal easing
    while True:
        motor_sync_read.txRxPacket()
        curr_positions = [
            motor_sync_read.getData(mid, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            for mid in dxlIDs
        ]

        new_vels = []
        for i, mid in enumerate(dxlIDs):
            dist    = total_distances[i]
            rem     = abs(goal_positions[i] - curr_positions[i])
            covered = dist - rem
            prog    = covered / dist if dist else 1.0

            if prog < RAMP_UP_FRACTION:
                ratio = prog / RAMP_UP_FRACTION
                s     = math.sin(ratio * math.pi / 2)
                vel   = min_vels[i] + int((cruise_vels[i] - min_vels[i]) * s)

            elif prog > 1 - RAMP_DOWN_FRACTION:
                ratio = (1 - prog) / RAMP_DOWN_FRACTION
                s     = math.sin(ratio * math.pi / 2)
                vel   = min_vels[i] + int((cruise_vels[i] - min_vels[i]) * s)

            else:
                vel = cruise_vels[i]

            new_vels.append(max(min(vel, cruise_vels[i]), min_vels[i]))

        # print(f"[simMotorRun] Updating velocities: {new_vels} for IDs {dxlIDs}")
        dxlSetVelo(new_vels, dxlIDs)
        simWrite(goal_positions, dxlIDs)

        # 5) Exit when within threshold
        if all(abs(curr - goal) <= DXL_MOVING_STATUS_THRESHOLD
               for curr, goal in zip(curr_positions, goal_positions)):
            print("[simMotorRun] Within threshold, breaking ramp loop.")
            time.sleep(0.2)
            break

        time.sleep(update_interval)

    # 6) Final verification and adjustment
    print("[simMotorRun] Entering final position check")
    simPosCheck(goal_positions, dxlIDs)


def portTermination():
    """Disable torque on all motors and close the port."""
    for motor_id in initialized_motor_ids:
        packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR_TORQUE_ENABLE, 0)
    portHandler.closePort()