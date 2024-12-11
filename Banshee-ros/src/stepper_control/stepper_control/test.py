
import time
from pinpong.board import Board, Pin
 
Board("uno").begin()  # Initialization, select board type (uno, microbit, RPi, handpy) and port number. If no port number is entered, automatic detection will be performed
#Board("uno", "COM36").begin()  # Initialization with specified port on Windows
Board("uno", "/dev/ttyACM0").begin()  # Initialization with specified port on Linux
#Board("uno", "/dev/cu.usbmodem14101").begin()  # Initialization with specified port on Mac
led = Pin(Pin.D13, Pin.OUT)  # Initialize the pin for digital output
 
while True:
  # led.value(1)  # Output high level Method 1
  led.write_digital(1)  # Output high level Method 2
  print("1")  # Print information in the terminal
  time.sleep(1)  # Wait for 1 second to maintain the state
 
  # led.value(0)  # Output low level Method 1
  led.write_digital(0)  # Output low level Method 2
  print("0")  # Print information in the terminal
  time.sleep(1)  # Wait for 1 second to maintain the state