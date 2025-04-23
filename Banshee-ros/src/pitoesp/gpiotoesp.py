import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(26, GPIO.OUT)

# Send a HIGH signal for 1 second
GPIO.output(26, GPIO.HIGH)
time.sleep(5)
GPIO.output(26,GPIO.LOW)