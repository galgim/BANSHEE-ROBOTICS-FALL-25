import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int8
import serial
import time
import struct


class BVMNode(Node):
    def __init__(self):
        super().__init__("BVM_Node")

        # Mode 0 = standby
        # Mode 1 = Pull Drone/BVM
        # Mode 2 = Push BVM/Drone
        # Mode 3 = Check if cycle done or continue cycle (mode 0 or mode 1)
        self.mode = 0  # decides what bvmlogic function will do
        self.done = 0  # flag for each mode
        self.halfCycleComplete = 0  # flag to choose whether drone battery or bvm battery chamber
        self.DroneMarkers = [8]
        self.batteryChamber = None  # Full battery chamber
        self.emptyChamber = None  # Empty battery chamber

        # Serial setup
        self.ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
        self.ser.reset_input_buffer()  # Clear garbage from ESP startup
        self.ser.reset_output_buffer()
        self.get_logger().info("Serial port connected and buffers cleared.")

        # ROS2 publishers and subscribers
        self.arucoPublisher = self.create_publisher(Int8, 'arucoID', 10)
        self.get_logger().info("ArucoID Publisher started")

        self.armSubscriber = self.create_subscription(Bool, 'ArmDone', self.modeComplete, 10)

        self.run_timer = self.create_timer(0.1, self.bvmLogic)

    VALID_TAGS = ["Voltage", "Unlock", "Lock", "CycleComplete", "DroneComplete"]

    # ----------------------- Serial Communication -----------------------

    def espRead(self):
        """ Read and process incoming data from ESP. """
        if self.ser.in_waiting > 0:
            tag = self.ser.readline().decode('utf-8', errors='ignore').strip()
            self.get_logger().info(f"Tag: {tag}")

            if tag not in self.VALID_TAGS:
                self.get_logger().warn(f"Ignored unknown tag: {tag}")
                return  # Skip unknown data

            if tag == "Voltage":
                raw_data = self.ser.read(32)
                values = self.structUnpack('8f', raw_data)
                if values:
                    self.batteryChamber = values.index(max(values))
                    self.emptyChamber = values.index(min(values))
                    self.mode = 1
                    self.get_logger().info(f"Voltages: {values}")

    def espSend(self, tag, data=None):
        """ Send command to ESP, with optional data. """
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.get_logger().info(f"Sending command: {tag} {data if data else ''}")
        self.ser.write((tag + '\n').encode('utf-8'))
        if data is not None:
            self.ser.write((str(data) + '\n').encode('utf-8'))

    # ----------------------- ROS Callbacks -----------------------

    def modeComplete(self, msg):
        """ Callback when Arm node signals completion. """
        if msg.data:
            self.done = 0
            self.mode += 1
            self.get_logger().info(f"Mode incremented to {self.mode}")

    # ----------------------- Data Handling -----------------------

    def structUnpack(self, type, data):
        """ Unpack binary float array from ESP. """
        try:
            return [round(v, 2) for v in struct.unpack(type, data)]
        except struct.error:
            self.get_logger().warn("Incomplete or malformed data received.")
            return None

    # ----------------------- Main Logic -----------------------

    def bvmLogic(self):
        """ Main state machine logic. """
        if self.mode == 0:
            self.espRead()

        elif self.mode == 1 and self.done == 0:
            # Pull Drone Battery (first half) or BVM Full Battery (second half)
            if self.halfCycleComplete == 0:
                aruco_ID = self.DroneMarkers[0]
            else:
                aruco_ID = self.batteryChamber
                self.espSend("Unlock", aruco_ID)

            self.publishAruco(aruco_ID)
            self.done = 1

        elif self.mode == 2 and self.done == 0:
            # Push Empty Battery (first half) or Return to Drone (second half)
            if self.halfCycleComplete == 0:
                aruco_ID = self.emptyChamber
                self.espSend("Unlock", aruco_ID)
            else:
                aruco_ID = self.DroneMarkers[0]
                self.espSend("Lock", self.batteryChamber)

            self.publishAruco(aruco_ID)
            self.done = 1

        elif self.mode == 3:
            # Cycle management and check for completion
            if self.halfCycleComplete == 0:
                self.mode = 1
                self.halfCycleComplete = 1
                self.espSend("Lock", self.emptyChamber)
                self.get_logger().info("Locking empty chamber. Continuing cycle.")
            else:
                self.DroneMarkers.pop(0)
                if len(self.DroneMarkers) > 0:
                    self.espSend("CycleComplete")
                    self.get_logger().info("Cycle complete. Proceeding to next.")
                else:
                    self.espSend("DroneComplete")
                    self.DroneMarkers = [8]  # Reset for next operation
                    self.get_logger().info("Drone operation complete. Resetting markers.")
                self.mode = 0
                self.halfCycleComplete = 0

    # ----------------------- Utilities -----------------------

    def publishAruco(self, aruco_ID):
        """ Publish Aruco ID for robot arm to act on. """
        msg = Int8()
        msg.data = aruco_ID
        self.arucoPublisher.publish(msg)
        self.get_logger().info(f"Published Aruco ID: {aruco_ID}")


# ----------------------- Main -----------------------

def main(args=None):
    rclpy.init(args=args)
    node = BVMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Exception occurred: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
