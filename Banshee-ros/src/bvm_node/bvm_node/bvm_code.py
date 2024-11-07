import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class BVMNode(Node):
    def __init__(self):
        super().__init__("BVM_Node")

        # Mode 0 = standby
        # Mode 1 = Begin battery transfer from drone to BVM
        # Mode 2 = Begin battery transfer from BVM to drone (Will go back to mode 1 if multiple batteries are detected)
        self.mode = 0


        self.arucoPublisher = self.create_publisher(
            String,
            'arucoID',
            10
        )
        self.get_logger().info("ArucoID Publisher started")

        self.armSubscriber = self.create_subscription(
            String,
            'modeComplete',
            self.subscriber_node,
            10
        )

        self.arucoPublisher()
    
    def arucoPublisher(self):
        msg = String()
        msg.data = input("Input Aruco Marker: ")
        self.arucoPublisher.publish(msg)
        self.get_logger().info('Sent marker: "%s"' % msg.data)
    
    def modeComplete(self, msg):
        pass

    
def main(args=None):
    rclpy.init(args=args)
    node = BVMNode()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
    main()