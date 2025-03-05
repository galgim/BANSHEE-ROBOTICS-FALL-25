from pinpong.board import Board, Pin
from time import sleep
from simple_pid import PID

# Initialize connection with Arduino Leonardo (Ensure correct port!)
Board("leonardo", "/dev/ttyACM0").begin()

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

        # PID Controller (TUNE THESE VALUES)
        self.pid = PID(1.0, 0.1, 0.05, setpoint=0)
        self.pid.output_limits = (0.005, 0.05)  # Step delay limits (5ms - 50ms)

    def move_to_position(self, target_position):
        """Moves stepper motor to the target position using PID speed control."""
        steps = target_position - self.position
        step_direction = CW if steps > 0 else CCW
        self.rotation.write_digital(step_direction)

        # Convert steps to absolute count
        steps = abs(steps)
        print(f"Moving {steps} steps {'CW' if step_direction == CW else 'CCW'}")

        # PID-controlled movement loop
        for _ in range(steps):
            error = target_position - self.position
            delay = self.pid(error)

            self.movement.write_digital(1)
            sleep(.0003)
            self.movement.write_digital(0)
            sleep(.0003)

            # Update current position
            self.position += 1 if step_direction == CW else -1

        print("Movement complete.")

# Run Test
if __name__ == '__main__':
    stepper = StepperMotor()
    
    # Move to 500 steps position (Change this for testing)
    stepper.move_to_position(500)
    
    # # Wait, then move back to 0
    # sleep(2)
    # stepper.move_to_position(0)
