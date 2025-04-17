from dynamixel_sdk import *  # Dynamixel SDK library for motor control
import time

# Control table addresses for Dynamixel XM series (4-byte values)
ADDR_TORQUE_ENABLE        = 64   # Torque ON/OFF address
ADDR_OPERATING_MODE       = 11   # Operating mode address
ADDR_PRESENT_POSITION     = 132  # Present position address (reads current position)
ADDR_GOAL_POSITION        = 116  # Goal position address (writes target position)
ADDR_PROFILE_VELOCITY     = 112  # Profile velocity address (sets motion speed)
LEN_GOAL_POSITION         = 4    # Byte length of goal position data
LEN_PRESENT_POSITION      = 4    # Byte length of present position data

# Protocol and mode constants
PROTOCOL_VERSION                      = 2  # Use protocol 2.0
TORQUE_ENABLE                         = 1  # Value to enable torque
OPERATING_MODE_CURRENT_BASED_POSITION = 5  # Current-based position control mode
DXL_MOVING_STATUS_THRESHOLD           = 10 # Threshold to consider movement complete (in ticks)

# Triangular profile parameters
MIN_SPEED_RATIO       = 0.1  # Start/end speed as fraction of cruise speed
RAMP_UP_FRACTION      = 0.2  # Fraction of distance spent ramping up
RAMP_DOWN_FRACTION    = 0.2  # Fraction of distance spent ramping down
SMALL_MOVE_THRESHOLD  = 20   # Maximum step change (in raw encoder ticks) for "small move"
                         # If all motors’ distances <= this, run at constant cruise velocity only

# Global handles (initialized in portInitialization)
portHandler      = None
packetHandler    = None
motor_sync_write = None
motor_sync_read  = None
DXL_ID           = []


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
    global portHandler, packetHandler, DXL_ID
    DXL_ID = dxlIDs
    portHandler = PortHandler(portname)
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    if not portHandler.openPort():
        raise RuntimeError("Failed to open port")
    if not portHandler.setBaudRate(1000000):
        raise RuntimeError("Failed to set baudrate")

    for mid in DXL_ID:
        packetHandler.write1ByteTxRx(portHandler, mid, ADDR_TORQUE_ENABLE, 0)
        packetHandler.write1ByteTxRx(portHandler, mid, ADDR_OPERATING_MODE, OPERATING_MODE_CURRENT_BASED_POSITION)
        packetHandler.write1ByteTxRx(portHandler, mid, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)


def WriteMotorData(mid, addr, val):
    """Write a 4-byte value to a motor."""
    packetHandler.write4ByteTxRx(portHandler, mid, addr, val)


def ReadMotorData(mid, addr):
    """Read a 4-byte value from a motor."""
    val, _, _ = packetHandler.read4ByteTxRx(portHandler, mid, addr)
    return val


def dxlSetVelo(vels, dxlIDs):
    """Set profile velocity for each motor."""
    for v, mid in zip(vels, dxlIDs):
        WriteMotorData(mid, ADDR_PROFILE_VELOCITY, v)


def dxlGetVelo(dxlIDs):
    """Return current profile velocities."""
    return [ReadMotorData(mid, ADDR_PROFILE_VELOCITY) for mid in dxlIDs]


def simWrite(positions, dxlIDs):
    """Send goal positions using GroupSyncWrite."""
    global motor_sync_write, motor_sync_read
    motor_sync_write = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)
    motor_sync_read  = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

    for mid in dxlIDs:
        motor_sync_read.addParam(mid)
    for pos, mid in zip(positions, dxlIDs):
        param = [
            DXL_LOBYTE(DXL_LOWORD(pos)),
            DXL_HIBYTE(DXL_LOWORD(pos)),
            DXL_LOBYTE(DXL_HIWORD(pos)),
            DXL_HIBYTE(DXL_HIWORD(pos))
        ]
        motor_sync_write.addParam(mid, param)
    motor_sync_write.txPacket()
    motor_sync_write.clearParam()


def simPosCheck(positions, dxlIDs, timeout=2.0):
    """Wait until motors reach goal positions or timeout."""
    t0 = time.time()
    while True:
        motor_sync_read.txRxPacket()
        current = [motor_sync_read.getData(mid, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION) for mid in dxlIDs]
        if all(abs(c - p) < DXL_MOVING_STATUS_THRESHOLD for c, p in zip(current, positions)):
            return
        if time.time() - t0 > timeout:
            return
        time.sleep(0.01)


def simMotorRun(angle_inputs, dxlIDs, update_interval=0.05, timeout=2.0):
    """
    Move motors with triangular velocity profile while maintaining synchronized end times,
    always performing a ramp-up and ramp-down.
    """
    # Convert angles to raw positions
    goal   = [_map(a, 0, 360, 0, 4095) for a in angle_inputs]
    start  = [ReadMotorData(mid, ADDR_PRESENT_POSITION) for mid in dxlIDs]
    total_dist = [abs(g - s) for g, s in zip(goal, start)]

    # Determine a common travel time T based on cruise velocities
    raw_cruise = dxlGetVelo(dxlIDs)
    times      = [(d / v if v > 0 else 0) for d, v in zip(total_dist, raw_cruise)]
    T = max(max(times), 1e-3)
    cruise_vel = [max(int(d / T), 1) for d in total_dist]

    # Minimum speeds at the start/end of the ramp
    min_vels = [max(int(v * MIN_SPEED_RATIO), 1) for v in cruise_vel]

    # Send the first goal packet
    simWrite(goal, dxlIDs)
    t0 = time.time()

    # Ramp loop
    while True:
        motor_sync_read.txRxPacket()
        curr = [motor_sync_read.getData(mid, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
                for mid in dxlIDs]

        new_vels = []
        for i, mid in enumerate(dxlIDs):
            d     = total_dist[i]
            rem   = abs(goal[i] - curr[i])
            covered = d - rem
            prog  = covered / d if d else 1

            # Triangular profile calculation
            if prog < RAMP_UP_FRACTION:
                # ramp up
                v = min_vels[i] + int((cruise_vel[i] - min_vels[i]) * (prog / RAMP_UP_FRACTION))
            elif prog > 1 - RAMP_DOWN_FRACTION:
                # ramp down
                v = min_vels[i] + int((cruise_vel[i] - min_vels[i]) * ((1 - prog) / RAMP_DOWN_FRACTION))
            else:
                # cruise
                v = cruise_vel[i]

            # clamp between min and cruise
            v = max(min(v, cruise_vel[i]), min_vels[i])
            new_vels.append(v)

        # update velocities and positions
        dxlSetVelo(new_vels, dxlIDs)
        simWrite(goal, dxlIDs)

        # exit when all motors near target or timed out
        if all(abs(c - g) < DXL_MOVING_STATUS_THRESHOLD for c, g in zip(curr, goal)) \
           or (time.time() - t0) > timeout:
            break

        time.sleep(update_interval)


def portTermination():
    """Disable torque on all motors and close the port."""
    for mid in DXL_ID:
        packetHandler.write1ByteTxRx(portHandler, mid, ADDR_TORQUE_ENABLE, 0)
    portHandler.closePort()
