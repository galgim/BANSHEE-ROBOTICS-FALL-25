import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)

# Send a HIGH signal for 1 second
def main (true):
    while(true):
        GPIO.output(17, GPIO.HIGH)
        time.sleep(2)
        GPIO.output(17, GPIO.HIGH)
        time.sleep(2)
        GPIO.cleanup()
