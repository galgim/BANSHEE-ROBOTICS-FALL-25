import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int8
from time import sleep 
from pinpong.board import Board, Pin

# Initialize connection with Arduino Leonardo (Ensure correct port!)
Board("leonardo", "/dev/ttyACM0").begin()

# Pin Definitions
DIR = Pin.D10  # Direction Pin (Dir+)
STEP = Pin.D8  # Step Pin (Pul+)

# Rotation Directions
CW = 1   # Clockwise
CCW = 0  # Counter Clockwise

# Set Positions in BTP
DRONE = 1481
RESET = 0
COLUMN1 = 500
COLUMN2 = 1481
COLUMN3 = 2492
COLUMN4 = 3510


class StepperMotorNode(Node):
    def __init__(self):
        super().__init__('stepper_motor_node')
        self.rotation = Pin(DIR, Pin.OUT)  # Direction control
        self.movement = Pin(STEP, Pin.OUT)  # Step control

        self.stepCoefficient = 500 / 159.5  # Conversion for distance to steps
        self.position = 0  # Current position tracking

        # ROS2 Subscribers
        self.initialSubscription = self.create_subscription(
            Int8, 'arucoID', self.initialSubscriber, 10)

        self.distanceSubscription = self.create_subscription(
            Float32, 'DestinationFalse', self.distanceSubscriber, 10)
        
        self.resetstepperSubscription = self.create_subscription(
            Int8, 'reset', self.resetstepperSubscriber, 10)

        # ROS2 Publisher to signal completion
        self.done_publisher = self.create_publisher(Bool, 'stepperDone', 10)

        self.get_logger().info('Stepper Motor Node has been started and is ready for commands.')

    def initialSubscriber(self, msg):
        """Handle Aruco ID for pre-set column positions"""
        batterychamber = msg.data
        self.get_logger().info(f"Received battery chamber: {batterychamber}")

        aruco_to_column = {
            0: COLUMN1, 4: COLUMN1,
            1: COLUMN2, 5: COLUMN2,
            2: COLUMN3, 6: COLUMN3,
            3: COLUMN4, 7: COLUMN4,
            8: DRONE
        }
        target_position = aruco_to_column.get(batterychamber, None)
        self.run_motor_cycle(target_position)

    def distanceSubscriber(self, msg):
        """Handle dynamic distance input to move additional steps"""
        distance = msg.data
        self.get_logger().info(f"Received distance: {distance}")
        addedSteps = distance * self.stepCoefficient
        self.run_motor_cycle(self.position + addedSteps)

    def resetstepperSubscriber(self, msg):
        """Reset stepper motor position to 0"""
        self.get_logger().info("Resetting stepper motor position to 0.")
        self.run_motor_cycle(RESET)
        

    def run_motor_cycle(self, newPosition):
        """Main function to execute stepper motor movements"""
        try:
            if newPosition is None:
                self.get_logger().warn("Received None as target, skipping movement.")
                return

            self.get_logger().info(f"New target position: {newPosition}")

            steps = round(newPosition - self.position)
            if steps == 0:
                self.get_logger().info("Already at target position. No movement required.")
                return

            # Check movement boundaries
            if not (0 <= self.position + steps <= 4050):
                self.get_logger().warn("Target position out of range. Skipping movement.")
                return

            # Set rotation direction
            self.rotation.write_digital(CW if steps > 0 else CCW)

            abs_steps = abs(steps)
            self.smooth_move(abs_steps)  # Move with smooth acceleration or slow constant speed

            # Update final position
            self.position += steps
            self.get_logger().info(f"Arrived at position {self.position}")

            # Publish completion signal
            sleep(1)  # Allow stabilization
            cycle_complete_msg = Bool()
            cycle_complete_msg.data = True
            self.done_publisher.publish(cycle_complete_msg)
            self.get_logger().info('Cycle complete, published signal to camera.')

        except KeyboardInterrupt:
            self.cleanup()

    def smooth_move(self, steps):
        """Perform smooth acceleration, cruising, and deceleration or constant slow speed for small steps"""
        min_delay = 0.0003  # Fastest delay (highest speed)
        max_delay = 0.002   # Slowest delay (initial and final speed)
        accel_percent = 0.3  # % of steps for acceleration/deceleration

        # If steps are small, move at constant slow speed
        if steps < 500:
            delay = max_delay
            for _ in range(steps):
                self.movement.write_digital(1)
                sleep(delay)
                self.movement.write_digital(0)
                sleep(delay)
            return  # Exit after slow constant-speed move

        # Otherwise, perform acceleration/deceleration
        accel_steps = int(steps * accel_percent)
        decel_steps = int(steps * accel_percent)
        cruise_steps = steps - accel_steps - decel_steps

        for step in range(steps):
            if step < accel_steps:
                # Acceleration phase
                delay = max_delay - (max_delay - min_delay) * (step / accel_steps)
            elif step >= accel_steps + cruise_steps:
                # Deceleration phase
                decel_step = step - (accel_steps + cruise_steps)
                delay = min_delay + (max_delay - min_delay) * (decel_step / decel_steps)
            else:
                # Constant speed (cruise)
                delay = min_delay

            # Step pulse
            self.movement.write_digital(1)
            sleep(delay)
            self.movement.write_digital(0)
            sleep(delay)

    def cleanup(self):
        self.get_logger().info('Cleaning up...')


def main(args=None):
    rclpy.init(args=args)
    stepper_motor_node = StepperMotorNode()
    try:
        rclpy.spin(stepper_motor_node)
    except KeyboardInterrupt:
        stepper_motor_node.cleanup()
    finally:
        stepper_motor_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
