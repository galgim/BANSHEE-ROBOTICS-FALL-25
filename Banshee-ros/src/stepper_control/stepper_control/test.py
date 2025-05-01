from pinpong.board import Board, Pin
from time import sleep
from serial.tools import list_ports

def find_arduino_port():
    """
    Scan all /dev/ttyACM* ports and return the one
    whose USB VID/PID (and/or description) matches
    an Arduino Leonardo.
    """
    for port in list_ports.comports():
        # Arduino SA Leonardo is VID=0x2341, PID=0x8036
        if port.vid == 0x2341 and port.pid == 0x8036:
            print(f"Found Leonardo on {port.device} ({port.description})")
            return port.device
    raise IOError("Arduino Leonardo not found")

port = find_arduino_port()
Board("leonardo", port).begin()
# Initialize connection with Arduino Leonardo (Ensure correct port!)
# Board("leonardo", "/dev/ttyACM0").begin()

# Define Stepper Motor Pins
DIR = Pin.D10  # Direction Pin
STEP = Pin.D8  # Step Pin

# Define Rotation Directions
CW = 1   # Clockwise
CCW = 0  # Counter Clockwise

# Stepper Motor Control Class
class StepperMotor:
    def __init__(self):
        self.rotation = Pin(DIR, Pin.OUT)
        self.movement = Pin(STEP, Pin.OUT)
        self.position = 0  # Current position

    def move_to_position(self, target_position):
        """Move stepper motor with smooth acceleration/deceleration for long distances, slow constant speed for short."""
        steps = target_position - self.position
        step_direction = CW if steps > 0 else CCW
        self.rotation.write_digital(step_direction)
        
        steps = abs(steps)
        print(f"Moving {steps} steps {'CW' if step_direction == CW else 'CCW'}")

        # Define delays (min_delay = fast, max_delay = slow)
        min_delay = 0.0003  # Fastest speed
        max_delay = 0.003   # Slowest speed

        accel_percent = 0.2  # Portion of steps for acceleration/deceleration

        # Threshold to decide whether to use acceleration or just slow speed
        if steps <= 200:
            delay = max_delay  # Move slow for small movements
            for step in range(steps):
                self.movement.write_digital(1)
                sleep(delay)
                self.movement.write_digital(0)
                sleep(delay)
                self.position += 1 if step_direction == CW else -1
        else:
            # Long movement with smooth acceleration and deceleration
            accel_steps = int(steps * accel_percent)
            decel_steps = int(steps * accel_percent)
            cruise_steps = steps - accel_steps - decel_steps

            for step in range(steps):
                if step < accel_steps:
                    # Acceleration: from slow to fast
                    delay = max_delay - (max_delay - min_delay) * (step / accel_steps)
                elif step >= accel_steps + cruise_steps:
                    # Deceleration: from fast back to slow
                    decel_step = step - (accel_steps + cruise_steps)
                    delay = min_delay + (max_delay - min_delay) * (decel_step / decel_steps)
                else:
                    # Cruise at fast speed
                    delay = min_delay

                # Step pulse
                self.movement.write_digital(1)
                sleep(delay)
                self.movement.write_digital(0)
                sleep(delay)

                # Update current position
                self.position += 1 if step_direction == CW else -1

        print(f"Arrived at position {self.position}.")

# Run Test
if __name__ == '__main__':
    stepper = StepperMotor()
    
    # Move small distance (slow constant speed)
    stepper.move_to_position(150)
    sleep(2)
    
    # Return to 0 (slow constant speed)
    stepper.move_to_position(0)
    sleep(2)

    # Move large distance (acceleration and deceleration to/from slow speed)
    stepper.move_to_position(500)
    sleep(2)
    
    # # Return to 0 (with smooth acceleration/deceleration)
    stepper.move_to_position(0)
