# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Bool
# import RPi.GPIO as GPIO
# from time import sleep

# # Pin Definitions
# DIR = 10  # Direction Pin (Dir+)
# STEP = 8  # Step Pin (Pul+)

# CW = 1   # Clockwise Rotation
# CCW = 0  # Counter Clockwise Rotation

# class StepperMotorNode(Node):
#     def __init__(self):
#         super().__init__('stepper_motor_node')
        
#         GPIO.setmode(GPIO.BOARD)
#         GPIO.setup(DIR, GPIO.OUT)
#         GPIO.setup(STEP, GPIO.OUT)
#         GPIO.output(DIR, CW)
        
#         # ROS2 Publisher and Subscriber
#         self.done_publisher = self.create_publisher(Bool, '/stepper/done', 10)
#         self.command_subscriber = self.create_subscription(Bool, '/stepper/command', self.run_motor_cycle, 10)
        
#         self.get_logger().info('Stepper Motor Node has been started and is ready for commands.')

#     def run_motor_cycle(self, msg):
#         if not msg.data:
#             return 
#         try:
#             GPIO.output(DIR, CW)
            
#             # Rotate Motor for 2000 steps in CW
#             for _ in range(200):
#                 GPIO.output(STEP, GPIO.HIGH)
#                 sleep(0.01) 
#                 GPIO.output(STEP, GPIO.LOW)
#                 sleep(0.01)
            
#             # Change direction to CCW
#             GPIO.output(DIR, CCW)
            
#             # Rotate Motor for 200 steps in CCW
#             for _ in range(200):
#                 GPIO.output(STEP, GPIO.HIGH)
#                 sleep(0.01)
#                 GPIO.output(STEP, GPIO.LOW)
#                 sleep(0.01)
            
#             # Publish cycle complete signal
#             self.get_logger().info('Cycle complete, publishing signal to begin integration')
#             cycle_complete_msg = Bool()
#             cycle_complete_msg.data = True
#             self.done_publisher.publish(cycle_complete_msg)
#         except KeyboardInterrupt:
#             self.cleanup()

#     def cleanup(self):
#         self.get_logger().info('Cleaning up...')
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
import RPi.GPIO as GPIO
from time import sleep

# Pin Definitions
DIR = 10  # Direction Pin (Dir+)
STEP = 8  # Step Pin (Pul+)

CW = 1   # Clockwise Rotation
CCW = 0  # Counter Clockwise Rotation

def main():
    # Setup GPIO
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(DIR, GPIO.OUT)
    GPIO.setup(STEP, GPIO.OUT)
    
    try:
        # Set direction to clockwise
        GPIO.output(DIR, CW)
        
        print("Stepper motor moving clockwise...")
        
        # Move the motor 200 steps clockwise, a little faster
        for _ in range(200):
            GPIO.output(STEP, GPIO.HIGH)
            sleep(0.005)  # Faster speed with reduced sleep time
            GPIO.output(STEP, GPIO.LOW)
            sleep(0.005)
        
        # Change direction to counterclockwise
        GPIO.output(DIR, CCW)
        print("Stepper motor moving counterclockwise...")
        
        # Move the motor 200 steps counterclockwise, same speed
        for _ in range(200):
            GPIO.output(STEP, GPIO.HIGH)
            sleep(0.005)  # Same speed as before
            GPIO.output(STEP, GPIO.LOW)
            sleep(0.005)
        
        print("Motion complete.")
        
    except KeyboardInterrupt:
        print("Interrupted!")
        
    finally:
        print("Cleaning up...")
        GPIO.cleanup()

if __name__ == '__main__':
    main()
