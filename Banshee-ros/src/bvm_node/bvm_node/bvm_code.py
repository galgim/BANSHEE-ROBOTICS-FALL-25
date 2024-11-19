import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int8
#drone battery 3-9
#upper battery 1-3
#lower battery 4-7
#we get drone landed done from gcs node (mode 1)
#voltage from esp and find lowest aruco with no voltage
#find max voltage battery lowest aruco
#back to drone aruco we previously
class BVMNode(Node):
    def __init__(self):
        super().__init__("BVM_Node")

        # Mode 0 = standby
        # Mode 1 = Begin battery transfer from drone to BVM
        # Mode 2 = Begin battery transfer from BVM to drone
        # Mode 3 = Done with Battery, set every value back to default standby value (If another battery, go back to mode 1)
        self.mode = 0
        self.done = 0 # flag for each mode
        self.checkModeComplete = False
        self.DroneMarkers = [7, 8]
        self.arucoID = None



        self.arucoPublisher = self.create_publisher(
        Int8, 'arucoID', 10)
        self.get_logger().info("ArucoID Publisher started")

        self.armSubscriber = self.create_subscription(
        Bool, 'modeComplete', self.modeComplete, 10)

        self.arucoIDPublisher()

        # Uncomment line and delete arucoID() once finished with GCS node
        # self.bvmLogic()
    
    def arucoIDPublisher(self):
        msg = Int8()
        msg.data = int(input("Input Aruco Marker: "))
        self.arucoPublisher.publish(msg)
        self.get_logger().info('Sent marker: "%s"' % msg.data)

        self.arucoIDPublisher()
    
    def modeComplete(self, msg):
        if msg.data:
            self.mode += 1
            self.done = 0
    
    def batteryAmount(self, msg):
        self.batteries = msg.data
    
    def bvmLogic(self):
        if len(self.DroneMarkers) > 0:
            if self.mode == 0:
                pass
            elif self.mode == 1 and self.done == 0:
                



                # Get BVM aruco ID needed
                arucoID = Int8() 
                arucoID.data = None # Figure out how to get it from esp
                self.arucoPublisher.publish(arucoID)
                self.get_logger().info('Sent marker: "%s"' % arucoID.data)
                self.done = 1
            elif self.mode == 2 and self.done == 0:
                



                self.done = 1
            elif self.mode == 3:
                self.previousID = arucoID


    
def main(args=None):
    rclpy.init(args=args)
    node = BVMNode()
    try:
        rclpy.spin(node)
    except:
        KeyboardInterrupt
    finally:
        rclpy.shutdown()
if __name__ == '__main__':
    main()