import os
import glob
from pymavlink import mavutil

def find_pixhawk_port():
    """Finds the correct Pixhawk serial port (if00)."""
    serial_ports = glob.glob('/dev/serial/by-id/*')

    for port in serial_ports:
        if "Pixhawk" in port or "Holybro" in port:
            if port.endswith("-if00"):  # Select only the main MAVLink interface
                print(f"Found Pixhawk MAVLink port: {port}")
                return port

    print("No valid Pixhawk MAVLink port found!")
    return None

def connect_to_pixhawk():
    """Connects to Pixhawk via MAVLink."""
    port = find_pixhawk_port()
    if not port:
        return None

    try:
        master = mavutil.mavlink_connection(port, baud=115200)
        master.wait_heartbeat(timeout=5)
        print("Connected to Pixhawk!")
        return master
    except Exception as e:
        print(f"Failed to connect: {e}")
        return None

# Run the connection test
if __name__ == "__main__":
    master = connect_to_pixhawk()
    if master:
        print("Listening for MAVLink messages...")
        while True:
            msg = master.recv_match(blocking=True)
            if msg:
                print(msg)