import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int8
# import RPi.GPIO as GPIO
from time import sleep 
from pinpong.board import Board, Pin
import math
Board("leonardo", "/dev/ttyACM0").begin()  # Initialization with specified port on Linux
# Pin Definitions
DIR = Pin.D10  # Direction Pin (Dir+)
STEP = Pin.D8  # Step Pin (Pul+)

CW = 1   # Clockwise Rotation
CCW = 0  # Counter Clockwise Rotation

# Set Positions in BTP
DRONE = 1481
RESET = 100
COLUMN1 = 500
COLUMN2 = 1481
COLUMN3 = 2492
COLUMN4 = 3510


class StepperMotorNode(Node):
    def __init__(self):
        super().__init__('stepper_motor_node')
        self.rotation = Pin(DIR, Pin.OUT)  # Initialize the pin for digital output
        self.movement = Pin(STEP, Pin.OUT)

        self.stepCoefficient = 500/159.5
        self.position = 0
        
        # ROS2 Publisher and Subscribers
        self.initialSubscription = self.create_subscription(
        Int8, 'arucoID', self.initialSubscriber, 10)

        self.distanceSubscription = self.create_subscription(
        Float32, 'DestinationFalse', self.distanceSubscriber, 10)

        self.done_publisher = self.create_publisher(Bool, 'stepperDone', 10)

        self.get_logger().info('Stepper Motor Node has been started and is ready for commands.')

    def initialSubscriber(self, msg):
        arucoID = msg.data
        self.get_logger().info(f"Received Aruco ID: {arucoID}")

        aruco_to_column = {
                0: COLUMN1, 4: COLUMN1,
                1: COLUMN2, 5: COLUMN2,
                2: COLUMN3, 6: COLUMN3,
                3: COLUMN4, 7: COLUMN4,
                8: DRONE
            }
        self.run_motor_cycle(aruco_to_column.get(arucoID, None))

    def distanceSubscriber(self, msg):
        distance = msg.data
        self.get_logger().info(f"Received distance: {distance}")
        addedSteps = distance * self.stepCoefficient
        self.run_motor_cycle(self.position + addedSteps)

    def speedCoefficient(self, steps, i):
        if steps < 300:
            return -0.5 * pow(math.e, -pow((5(i / steps) - 2), 2)) + 1
        
    def run_motor_cycle(self, newPosition):
        try:
            if newPosition is not None:
                self.get_logger().info(str(newPosition) + " is new position")

                steps = newPosition - self.position
                abs_rounded_steps = abs(round(steps))

                # Max steps in CW 4050
                if self.position + steps > 4050 or self.position + steps < 0:
                    self.get_logger().warn("Distance out of range. Movement skipped.")
                    return

                if steps > 0:
                    self.rotation.write_digital(CW)
                    # GPIO.output(DIR, CW)
                else:
                    self.rotation.write_digital(CCW)
                    # GPIO.output(DIR, CCW)
                for i in range(abs_rounded_steps):   
                    speed = self.speedCoefficient(abs_rounded_steps, i)
                    self.movement.write_digital(1)         
                    # GPIO.output(STEP, GPIO.HIGH)
                    sleep(0.002 * speed) 
                    self.movement.write_digital(0)
                    # GPIO.output(STEP, GPIO.LOW)
                    sleep(0.002 * speed)
                
                self.position = self.position + steps

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
        # GPIO.cleanup()


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