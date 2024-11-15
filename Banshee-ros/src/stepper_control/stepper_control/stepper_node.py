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



class StepperMotorNode(Node):
    def __init__(self):
        super().__init__('stepper_motor_node')
        
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(DIR, GPIO.OUT)
        GPIO.setup(STEP, GPIO.OUT)
        
        self.arucoID = None
        self.distance = None
        self.steps = 0
        self.position = 0
        
        # ROS2 Publisher and Subscribers
        self.initialSubscription = self.create_subscription(
        Int8, 'arucoID', self.initialSubscriber, 10)

        self.distanceSubscription = self.create_subscription(
        Int32, 'DestinationFalse', self.distanceSubscriber, 10)

        self.done_publisher = self.create_publisher(Bool, 'stepperDone', 10)

        self.get_logger().info('Stepper Motor Node has been started and is ready for commands.')

        self.initial_movement()

    def initialSubscriber(self, msg):
        self.arucoID = msg.data
        self.get_logger().info(f"Received Aruco ID: {self.arucoID}")

        pass # Move to initial position

    def distanceSubscriber(self, msg):
        self.distance = msg.data
        self.get_logger().info(f"Received distance: {self.distance}")
        self.steps = abs(round(1000/296 * self.distance))
        self.run_motor_cycle()

    def initial_movement(self):
        try:
            GPIO.output(DIR, CW)
            for _ in range(500):                   
                    GPIO.output(STEP, GPIO.HIGH)
                    sleep(0.01) 
                    GPIO.output(STEP, GPIO.LOW)
                    sleep(0.01)

            sleep(1)
            self.get_logger().info('Cycle complete, publishing signal to camera')
            cycle_complete_msg = Bool()
            cycle_complete_msg.data = True
            self.done_publisher.publish(cycle_complete_msg)

        except KeyboardInterrupt:
            self.cleanup()

    def run_motor_cycle(self):
        try:
            if self.distance != None:
                if self.distance > 0:
                    GPIO.output(DIR, CW)
                else:
                    GPIO.output(DIR, CCW)
                
                # Max steps in CW 4050
                for _ in range(self.steps):                   
                    GPIO.output(STEP, GPIO.HIGH)
                    sleep(0.01) 
                    GPIO.output(STEP, GPIO.LOW)
                    sleep(0.01)
                

                sleep(1)
                # Publish cycle complete signal
                self.get_logger().info('Cycle complete, publishing signal to camera')
                cycle_complete_msg = Bool()
                cycle_complete_msg.data = True
                self.done_publisher.publish(cycle_complete_msg)
                self.distance = None
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
# import RPi.GPIO as GPIO
# from time import sleep

# # Pin Definitions
# DIR = 10  # Direction Pin (Dir+)
# STEP = 8  # Step Pin (Pul+)

# CW = 1   # Clockwise Rotation
# CCW = 0  # Counter Clockwise Rotation

# def main():
#     # Setup GPIO
#     GPIO.setmode(GPIO.BOARD)
#     GPIO.setup(DIR, GPIO.OUT)
#     GPIO.setup(STEP, GPIO.OUT)
    
#     try:
#         # Set direction to clockwise
#         GPIO.output(DIR, CW)
        
#         print("Stepper motor moving clockwise...")
#         sleep(5)
#         # Move the motor 200 steps clockwise, a little faster
#         for _ in range(500):
#             GPIO.output(STEP, GPIO.HIGH)
#             sleep(0.0005)  # Faster speed with reduced sleep time
#             GPIO.output(STEP, GPIO.LOW)
#             sleep(0.0005)
        
#         # Change direction to counterclockwise
#         GPIO.output(DIR, CCW)
#         print("Stepper motor moving counterclockwise...")
#         sleep(1)
#         # Move the motor 200 steps counterclockwise, same speed
#         for _ in range(500):
#             GPIO.output(STEP, GPIO.HIGH)
#             sleep(0.0005)  # Same speed as before
#             GPIO.output(STEP, GPIO.LOW)
#             sleep(0.0005)
        
#         print("Motion complete.")
        
#     except KeyboardInterrupt:
#         print("Interrupted!")
        
#     finally:
#         print("Cleaning up...")
#         GPIO.cleanup()

# if __name__ == '__main__':
#     main()
