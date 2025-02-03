import serial

# Change '/dev/ttyUSB0' to the correct port (COMx on Windows)
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

while True:
    ser.write(b'Hello ESP32\n')  # Send data
    response = ser.readline().decode('utf-8').strip()  # Read response
    if response:
        print("Received from ESP32:", response)
