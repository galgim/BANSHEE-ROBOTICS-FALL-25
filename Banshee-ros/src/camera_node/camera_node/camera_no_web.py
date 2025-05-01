import glob
import serial
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import threading
from std_msgs.msg import Int8, Float32, Bool

def find_ftdi_port():
    """Finds the FTDI USB-Serial Converter port under /dev/serial/by-id."""
    ports = glob.glob('/dev/serial/by-id/*FTDI_USB-Serial_Converter*')
    if ports:
        print(f"Found FTDI port: {ports[0]}")
        return ports[0]
    else:
        print("No FTDI port found")
        return None

class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")
        # --- open FTDI serial port ---
        port = find_ftdi_port()
        self.serial_conn = None
        if port:
            try:
                self.serial_conn = serial.Serial(port, 115200, timeout=1)
                self.get_logger().info(f"Opened serial on {port}")
            except serial.SerialException as e:
                self.get_logger().error(f"Failed to open serial {port}: {e}")
        else:
            self.get_logger().error("FTDI serial port not found; continuing without serial")

        # --- ROS subscriptions & publishers ---
        self.arucoID = None
        self.batteryChamber = None
        self.sendFrame = False

        self.create_subscription(Int8, 'arucoID', self.arucoSubscriber, 10)
        self.create_subscription(Bool, 'stepperDone', self.stepperSubscriber, 10)
        self.destinationTrue = self.create_publisher(Int8, 'DestinationConfirm', 10)
        self.destinationFalse = self.create_publisher(Float32, 'DestinationFalse', 10)

        # start camera thread
        self.camera_thread = threading.Thread(target=self.cameraRun, daemon=True)
        self.camera_thread.start()
        self.get_logger().info("Camera node initialized")

    def arucoSubscriber(self, msg: Int8):
        self.sendFrame = False
        self.batteryChamber = int(msg.data)
        # map batteryChamber to arucoID
        if self.batteryChamber < 8:
            self.arucoID = self.batteryChamber % 4
        else:
            self.arucoID = self.batteryChamber - 4
        self.get_logger().info(f"Received Aruco ID: {self.arucoID}")

    def stepperSubscriber(self, msg: Bool):
        if msg.data:
            self.sendFrame = True
            self.get_logger().info("Stepper finished moving")

    def cameraRun(self):
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
        detector_params = cv2.aruco.DetectorParameters()
        refine_params = cv2.aruco.RefineParameters()
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            self.get_logger().error("Cannot open camera")
            return

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                self.get_logger().error("Can't receive frame. Exiting ...")
                break

            h, w, _ = frame.shape
            box_size = 50
            box_x = (w - box_size) // 2
            box_y = (h - box_size) // 2

            detector = cv2.aruco.ArucoDetector(aruco_dict, detector_params, refine_params)
            corners_list, ids, _ = detector.detectMarkers(frame)

            if ids is not None and self.arucoID is not None:
                direction = 1 if self.arucoID < 4 else -1
                for mid, corners in zip(ids, corners_list):
                    if mid == self.arucoID:
                        c1x = corners[0][0][0]
                        c2x = corners[0][2][0]
                        middle_x = (c1x + c2x) / 2
                        distance = (w/2 - middle_x) * direction

                        if self.sendFrame:
                            if abs(distance) <= 1.5:
                                out = Int8()
                                if self.batteryChamber < 4:
                                    out.data = 0
                                elif self.batteryChamber < 8:
                                    out.data = 1
                                else:
                                    out.data = 2
                                self.destinationTrue.publish(out)
                                self.get_logger().info(f"Published DestinationConfirm: {out.data}")
                                # notify Arduino via serial
                                if self.serial_conn:
                                    self.serial_conn.write(b'C')
                            else:
                                outf = Float32()
                                outf.data = distance
                                self.destinationFalse.publish(outf)
                                if self.serial_conn:
                                    self.serial_conn.write(f"D{distance:.2f}\n".encode())
                            self.sendFrame = False
                            self.get_logger().warn("sendFrame reset to False")

                        cv2.putText(frame, f"dist: {distance:.1f}",
                                    (10,30), cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),2)

                        # optional overlap ratio display omitted for brevity

            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners_list, ids)
            cv2.rectangle(frame, (box_x,box_y),(box_x+box_size,box_y+box_size),(0,255,0),2)
            cv2.imshow("Camera live stream", frame)

            if cv2.waitKey(1) == ord('q'):
                self.sendFrame = True

        cap.release()
        cv2.destroyAllWindows()

    def destroy_node(self):
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    finally:
        node.camera_thread.join()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
