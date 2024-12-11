
from time import sleep
from pinpong.board import Board, Pin

# Pin Definitions
DIR = Pin.D10  # Direction Pin (Dir+)
STEP = Pin.D8  # Step Pin (Pul+)

CW = 1   # Clockwise Rotation
CCW = 0  # Counter Clockwise Rotation
# Board("leonardo").begin()  # Initialization, select board type (uno, microbit, RPi, handpy) and port number. If no port number is entered, automatic detection will be performed
#Board("uno", "COM36").begin()  # Initialization with specified port on Windows
Board("leonardo", "/dev/ttyACM0").begin()  # Initialization with specified port on Linux
#Board("uno", "/dev/cu.usbmodem14101").begin()  # Initialization with specified port on Mac
mov = Pin(STEP, Pin.OUT)  # Initialize the pin for digital output'\
dir = Pin(DIR, Pin.OUT)  # Initialize the pin for digital output'\

while True:
  dir.write_digital(0)# led.value(1)  # Output high level Method 1
  mov.write_digital(1)
  sleep(0.002)  # Output high level Method 2
  mov.write_digital(0)
  sleep(0.002)
