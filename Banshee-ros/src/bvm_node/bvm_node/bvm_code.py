import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int8

class BVMNode(Node):
    def __init__(self):
        super().__init__("BVM_Node")

        # Mode 0 = standby
        # Mode 1 = Begin battery transfer from drone to BVM
        # Mode 2 = Begin battery transfer from BVM to drone (Will go back to mode 1 if multiple batteries are detected)
        self.mode = 0
        self.done = 0 # flag for each mode
        self.checkModeComplete = False
        self.batteries = 0
        self.batteriesDone = 0


        self.arucoPublisher = self.create_publisher(
            Int8,
            'arucoID',
            10
        )
        self.get_logger().info("ArucoID Publisher started")

        self.armSubscriber = self.create_subscription(
            Bool,
            'modeComplete',
            self.modeComplete,
            10
        )

        self.cameraSubscriber= self.create_subscription(
            Int8,
            'batteryAmount',
            self.modeComplete,
            10
        )

        self.arucoID()

        # Uncomment line and delete arucoID() once finished with GCS node
        # self.bvmLogic()
    
    def arucoID(self):
        msg = Int8()
        msg.data = int(input("Input Aruco Marker: "))
        self.arucoPublisher.publish(msg)
        self.get_logger().info('Sent marker: "%s"' % msg.data)
    
    def modeComplete(self, msg):
        if msg.data:
            self.mode += 1
    
    def batteryAmount(self, msg):
        self.batteries = msg.data
    
    def bvmLogic(self):
        if self.batteries > self.batteriesDone:
            if self.mode == 0:
                pass
            elif self.mode == 1 and self.done == 0:
                # Get BVM aruco ID needed
                arucoID = Int8() 
                arucoID.data = None # Figure out how to get it from website
                self.arucoPublisher.publish(arucoID)
                self.get_logger().info('Sent marker: "%s"' % arucoID.data)
                self.done = 1
            elif self.mode == 2 and self.done == 1:
                
                if self.batteriesDone == 0 and self.batteries > 1:
                    self.mode = 1
                else:
                    self.mode = 0

    
def main(args=None):
    rclpy.init(args=args)
    node = BVMNode()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
    main()