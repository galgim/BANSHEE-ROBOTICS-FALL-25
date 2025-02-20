import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int8
import serial
import time
import struct
#drone battery 4-5
#battery columns 0-3
#we get drone landed done from gcs node (mode 1)
#voltage from esp and find lowest aruco with no voltage
#find max voltage battery lowest aruco
#back to drone aruco we previously
class BVMNode(Node):
    def __init__(self):
        super().__init__("BVM_Node")

        # Mode 0 = standby
        # Mode 1 = Pull Drone/BVM
        # Mode 2 = Push BVM/Drone
        # Mode 3 = Check if cycle done or continue cycle (mode 0 or mode 1)
        self.mode = 0
        self.done = 0 # flag for each mode
        self.checkModeComplete = False
        self.DroneMarkers = [4, 5]
        self.batteryChamber = None
        self.ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)


        self.arucoPublisher = self.create_publisher(
        Int8, 'arucoID', 10)
        self.get_logger().info("ArucoID Publisher started")

        self.armSubscriber = self.create_subscription(
        Bool, 'ArmDone', self.modeComplete, 10)

        # self.arucoIDPublisher()

        # Uncomment line and delete arucoID() once finished with GCS node
        self.run_timer = self.create_timer(0.1, self.bvmLogic)
    
    def arucoIDPublisher(self, aruco_ID):
        msg = Int8()
        # msg.data = int(input("Input Battery Chamber: "))
        msg.data = aruco_ID
        self.arucoPublisher.publish(msg)
        self.get_logger().info('Sent marker: "%s"' % msg.data)

        # self.arucoIDPublisher()
    
    def modeComplete(self, msg):
        if msg.data:
            self.done = 0
            self.mode += 1
    
    def batteryAmount(self, msg):
        self.batteries = msg.data

    def structUnpack(self, type, data):
        try:
            return [round(v,2) for v in struct.unpack(type, data)]
        except:
            self.get_logger().info("Incomplete data received")

    def espRead(self):
        if self.ser.in_waiting > 0:
            tag = self.ser.readline().decode('utf-8').strip()
            # Examples of reading from ESP
            if tag == "Voltage":
                raw_data = self.ser.read(32)
                values = self.structUnpack('8f', raw_data)
                self.batteryChamber = values.index(max(values))
                self.get_logger().warn(f"Choosing chamber {self.batteryChamber}.")
                self.mode = 1
            elif tag == "Chamber":
                raw_data = self.ser.read(4)
                values = self.structUnpack('1i', raw_data)
            else:
                return
            self.get_logger().info("Tag: " + tag)
            self.get_logger().info(f"{values}")

    def espSend(self, tag, data):
        tag = str(tag)
        self.ser.write((tag + '\n').encode('utf-8'))
        data = str(data)
        self.ser.write((data + '\n').encode('utf-8'))
        
    def bvmLogic(self):
        if len(self.DroneMarkers) > 0:
            if self.mode == 0:
                self.espRead()                                              # Find highest and lowest voltage in BVM, esp switches to mode 1
            
            # Mode 1: Pull Drone, Push Empty
            elif self.mode == 1 and self.done == 0:
                self.espSend("Chamber", self.batteryChamber)                # unlock battery chamber
                # pull drone
                self.done = 1


                # aruco_ID = 4                                                # drone aruco_ID
                # self.arucoPublisher.publish(aruco_ID)                       # publish aruco_ID
                # self.get_logger().info('Sent marker: "%s"' % aruco_ID.data)

            # Mode 2: Pull Full, Push Drone
            elif self.mode == 2 and self.done == 0:
                self.espSend("Chamber", self.batteryChamber)                # unlock battery chamber
                # push bvm
                self.done = 1


                # aruco_ID = None # Whichever chamber is full                                              
                # self.arucoPublisher.publish(aruco_ID)                       # publish aruco_ID
                # self.get_logger().info('Sent marker: "%s"' % aruco_ID.data)   

            elif self.mode == 3 and self.done == 0:
                self.previousID = self.arucoID
                # determine whether to go to mode 1 or 0, based on drone array 
                # if drone array has number go to mode 1 and pull that number if drone array empty go to mode 0
                #drone array get rid of one index


    
def main(args=None):
    rclpy.init(args=args)
    node = BVMNode()
    try:
        rclpy.spin(node)
    except:
        KeyboardInterrupt
if __name__ == '__main__':
    main()