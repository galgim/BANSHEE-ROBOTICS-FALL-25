# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Bool, Int32, Int8
# import RPi.GPIO as GPIO
# from time import sleep

# # Pin Definitions
# DIR = 10  # Direction Pin (Dir+)
# STEP = 8  # Step Pin (Pul+)

# CW = 1   # Clockwise Rotation
# CCW = 0  # Counter Clockwise Rotation

# # Set Positions in BTP
# COLUMN1 = 463
# COLUMN2 = 1481
# COLUMN3 = 2492
# COLUMN4 = 3510

# DELAY = 0.003

# class StepperMotorNode(Node):
#     def __init__(self):
#         super().__init__('stepper_motor_node')
        
#         # GPIO Setup
#         GPIO.setmode(GPIO.BOARD)
#         GPIO.setup(DIR, GPIO.OUT)
#         GPIO.setup(STEP, GPIO.OUT)
        
#         # Attributes
#         self.arucoID = None
#         self.distance = None
#         self.stepCoefficient = 1000 / 296  # Conversion factor for distance to steps
#         self.position = 0
        
#         # ROS2 Publisher and Subscribers
#         self.initialSubscription = self.create_subscription(
#             Int8, 'arucoID', self.initialSubscriber, 10)

#         self.distanceSubscription = self.create_subscription(
#             Int32, 'DestinationFalse', self.distanceSubscriber, 10)

#         self.done_publisher = self.create_publisher(Bool, 'stepperDone', 10)

#         self.get_logger().info('Stepper Motor Node has been started and is ready for commands.')

#     def initialSubscriber(self, msg):
#         """Callback for receiving Aruco IDs."""
#         self.arucoID = msg.data
#         self.get_logger().info(f"Received Aruco ID: {self.arucoID}")
#         self.initial_movement()

#     def distanceSubscriber(self, msg):
#         """Callback for receiving distance to move the motor."""
#         self.distance = msg.data
#         self.get_logger().info(f"Received distance: {self.distance}")
#         self.run_motor_cycle()

#     def initial_movement(self):
#         """Move the stepper motor to the column associated with the received Aruco ID."""
#         try:
#             if self.arucoID is None:
#                 self.get_logger().warn("Aruco ID not set. Skipping initial movement.")
#                 return

#             # Mapping Aruco IDs to columns
#             aruco_to_column = {
#                 0: COLUMN4, 1: COLUMN4,
#                 2: COLUMN3, 3: COLUMN3,
#                 4: COLUMN2, 5: COLUMN2,
#                 6: COLUMN1, 7: COLUMN1
#             }

#             # Determine the target column based on the received Aruco ID
#             target_steps = aruco_to_column.get(self.arucoID, None)

#             if target_steps is None:
#                 self.get_logger().warn(f"Invalid Aruco ID: {self.arucoID}. Skipping movement.")
#                 return

#             self.get_logger().info(f"Moving to column associated with Aruco ID {self.arucoID}.")

#             # Move to the target column
#             GPIO.output(DIR, CW)
#             for _ in range(target_steps):
#                 GPIO.output(STEP, GPIO.HIGH)
#                 sleep(DELAY)
#                 GPIO.output(STEP, GPIO.LOW)
#                 sleep(DELAY)

#             # Return to origin
#             GPIO.output(DIR, CCW)
#             for _ in range(target_steps):
#                 GPIO.output(STEP, GPIO.HIGH)
#                 sleep(DELAY)
#                 GPIO.output(STEP, GPIO.LOW)
#                 sleep(DELAY)

#             sleep(1)
#             self.get_logger().info('Cycle complete, publishing signal to camera.')
#             cycle_complete_msg = Bool()
#             cycle_complete_msg.data = True
#             self.done_publisher.publish(cycle_complete_msg)

#         except KeyboardInterrupt:
#             self.cleanup()

#     def run_motor_cycle(self):
#         """Move the motor based on the received distance."""
#         try:
#             if self.distance is not None:
#                 if self.distance > 4050 or self.distance < -4050:
#                     self.get_logger().warn("Distance out of range. Movement skipped.")
#                     return

#                 GPIO.output(DIR, CW if self.distance > 0 else CCW)

#                 steps = round(abs(self.stepCoefficient * self.distance))
#                 self.get_logger().info(f"Running motor cycle for {steps} steps.")

#                 for _ in range(steps):
#                     GPIO.output(STEP, GPIO.HIGH)
#                     sleep(DELAY)
#                     GPIO.output(STEP, GPIO.LOW)
#                     sleep(DELAY)

#                 sleep(1)
#                 # Publish cycle complete signal
#                 self.get_logger().info('Cycle complete, publishing signal to camera.')
#                 cycle_complete_msg = Bool()
#                 cycle_complete_msg.data = True
#                 self.done_publisher.publish(cycle_complete_msg)
#                 self.distance = None
#         except KeyboardInterrupt:
#             self.cleanup()

#     def cleanup(self):
#         """Cleanup GPIO on shutdown."""
#         self.get_logger().info('Cleaning up GPIO...')
#         GPIO.cleanup()


# def main(args=None):
#     rclpy.init(args=args)
    
#     stepper_motor_node = StepperMotorNode()
    
#     try:
#         rclpy.spin(stepper_motor_node)
#     except KeyboardInterrupt:
#         stepper_motor_node.cleanup()
#     finally:
#         stepper_motor_node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()
    
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, Int8
import RPi.GPIO as GPIO
from time import sleep

# Pin Definitions
DIR = 10  # Direction Pin (Dir+)
STEP = 8  # Step Pin (Pul+)

CW = 1   # Clockwise Rotation
CCW = 0  # Counter Clockwise Rotation

# Set Positions in BTP
COLUMN1 = 463
COLUMN2 = 1481
COLUMN3 = 2492
COLUMN4 = 3510


class StepperMotorNode(Node):
    def __init__(self):
        super().__init__('stepper_motor_node')
        
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(DIR, GPIO.OUT)
        GPIO.setup(STEP, GPIO.OUT)
        
        self.stepCoefficient = 1000/296
        self.position = 0
        
        # ROS2 Publisher and Subscribers
        self.initialSubscription = self.create_subscription(
        Int8, 'arucoID', self.initialSubscriber, 10)

        self.distanceSubscription = self.create_subscription(
        Int32, 'DestinationFalse', self.distanceSubscriber, 10)

        self.done_publisher = self.create_publisher(Bool, 'stepperDone', 10)

        self.get_logger().info('Stepper Motor Node has been started and is ready for commands.')

    def initialSubscriber(self, msg):
        arucoID = msg.data
        self.get_logger().info(f"Received Aruco ID: {arucoID}")

        aruco_to_column = {
                0: COLUMN1, 4: COLUMN1,
                1: COLUMN2, 5: COLUMN2,
                2: COLUMN3, 6: COLUMN3,
                3: COLUMN4, 7: COLUMN4
            }

        self.run_motor_cycle(aruco_to_column.get(self.arucoID, None) / self.stepCoefficient)

    def distanceSubscriber(self, msg):
        distance = msg.data
        self.get_logger().info(f"Received distance: {distance}")
        self.run_motor_cycle(distance)

    def run_motor_cycle(self, distance):
        try:
            if distance != None:
                if distance > 0:
                    GPIO.output(DIR, CW)
                else:
                    GPIO.output(DIR, CCW)
                
                # Max steps in CW 4050
                steps = round(abs(self.stepCoefficient * distance))
                for _ in range(steps):                   
                    GPIO.output(STEP, GPIO.HIGH)
                    sleep(0.002) 
                    GPIO.output(STEP, GPIO.LOW)
                    sleep(0.002)

                sleep(1)
                # Publish cycle complete signal
                self.get_logger().info('Cycle complete, publishing signal to camera')
                cycle_complete_msg = Bool()
                cycle_complete_msg.data = True
                self.done_publisher.publish(cycle_complete_msg)
        except KeyboardInterrupt:
            self.cleanup()

    def cleanup(self):
        self.get_logger().info('Cleaning up...')
        GPIO.cleanup()


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