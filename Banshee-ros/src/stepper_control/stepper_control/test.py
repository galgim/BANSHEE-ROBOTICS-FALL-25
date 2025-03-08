from pinpong.board import Board, Pin
from time import sleep

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

    def move_to_position(self, target_position):
        """Moves stepper motor to the target position with acceleration and deceleration for steps > 200."""
        steps = target_position - self.position
        step_direction = CW if steps > 0 else CCW
        self.rotation.write_digital(step_direction)
        
        steps = abs(steps)
        print(f"Moving {steps} steps {'CW' if step_direction == CW else 'CCW'}")

        if steps > 200:
            min_delay = 0.0003  # Fastest speed
            max_delay = 0.003   # Slowest speed
        else:
            min_delay = 0.003  # Fixed slow speed for small movements
            max_delay = 0.003

        for step in range(steps):
            if steps > 200:
                # Apply acceleration and deceleration only for steps > 200
                if step < steps * 0.07:  # First 20% acceleration
                    delay = max_delay - (max_delay - min_delay) * (step / (steps * 0.2))
                elif step > steps * 0.93:  # Last 20% deceleration
                    delay = min_delay + (max_delay - min_delay) * ((step - steps * 0.8) / (steps * 0.2))
                else:  # Constant speed
                    delay = min_delay
            else:
                delay = max_delay  # Fixed slow speed for small movements
            
            self.movement.write_digital(1)
            sleep(delay)
            self.movement.write_digital(0)
            sleep(delay)
            
            self.position += 1 if step_direction == CW else -1
        
        print("Movement complete.")

# Run Test
if __name__ == '__main__':
    stepper = StepperMotor()
    
    # Move to 1000 steps position
    stepper.move_to_position(2000)
    
    # Wait, then move back to 0
    sleep(2)
    stepper.move_to_position(0)
    sleep(1)
    stepper.move_to_position(100)
    sleep(1)
    stepper.move_to_position(0)