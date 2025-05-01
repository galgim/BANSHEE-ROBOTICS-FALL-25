import glob
import serial
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import threading
from std_msgs.msg import Int8, Float32, Bool

def find_camera_device():
    """Return the first /dev/video* device that OpenCV can open, or None."""
    for dev in sorted(glob.glob('/dev/video*')):
        cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if cap.isOpened():
            cap.release()
            print(f"Using camera device: {dev}")
            return dev
        cap.release()
    print("No camera device found")
    return None

def find_ftdi_port():
    ports = glob.glob('/dev/serial/by-id/*FTDI_USB-Serial_Converter*')
    return ports[0] if ports else None

class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")

        # -- find & open camera --
        cam_dev = find_camera_device()
        if not cam_dev:
            self.get_logger().error("No /dev/video* device available; exiting")
            return
        self.cap = cv2.VideoCapture(cam_dev, cv2.CAP_V4L2)
        self.get_logger().info(f"Opened camera at {cam_dev}")

        # -- find & open serial --
        port = find_ftdi_port()
        self.serial_conn = None
        if port:
            try:
                self.serial_conn = serial.Serial(port, 115200, timeout=1)
                self.get_logger().info(f"Opened serial on {port}")
            except serial.SerialException as e:
                self.get_logger().error(f"Serial error on {port}: {e}")
        else:
            self.get_logger().warn("FTDI serial port not found")

        # -- ROS pubs/subs --
        self.arucoID = None
        self.batteryChamber = None
        self.sendFrame = False

        self.create_subscription(Int8, 'arucoID', self.arucoSubscriber, 10)
        self.create_subscription(Bool, 'stepperDone', self.stepperSubscriber, 10)
        self.destinationTrue  = self.create_publisher(Int8,   'DestinationConfirm', 10)
        self.destinationFalse = self.create_publisher(Float32,'DestinationFalse',   10)

        # -- start camera loop thread --
        self.camera_thread = threading.Thread(target=self.cameraRun, daemon=True)
        self.camera_thread.start()
        self.get_logger().info("Camera node initialized")

    def arucoSubscriber(self, msg):
        self.sendFrame = False
        self.batteryChamber = int(msg.data)
        self.arucoID = (self.batteryChamber % 4 if self.batteryChamber < 8
                        else self.batteryChamber - 4)
        self.get_logger().info(f"Got Aruco ID → {self.arucoID}")

    def stepperSubscriber(self, msg):
        if msg.data:
            self.sendFrame = True
            self.get_logger().info("Stepper done → sendFrame=True")

    def cameraRun(self):
        aruco_dict   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
        det_params   = cv2.aruco.DetectorParameters()
        refine_params= cv2.aruco.RefineParameters()

        while self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().error("Frame grab failed, exiting loop")
                break

            h, w, _ = frame.shape
            box_size = 50
            bx, by = (w - box_size)//2, (h - box_size)//2

            detector = cv2.aruco.ArucoDetector(aruco_dict, det_params, refine_params)
            corners_list, ids, _ = detector.detectMarkers(frame)

            if ids is not None and self.arucoID is not None:
                dir_sign = 1 if self.arucoID < 4 else -1
                for mid, corners in zip(ids, corners_list):
                    if mid == self.arucoID:
                        # compute distance…
                        c1x, c2x = corners[0][0][0], corners[0][2][0]
                        dist = (w/2 - (c1x+c2x)/2) * dir_sign

                        if self.sendFrame:
                            if abs(dist) <= 1.5:
                                out = Int8()
                                out.data = (0 if self.batteryChamber<4
                                            else 1 if self.batteryChamber<8
                                            else 2)
                                self.destinationTrue.publish(out)
                                if self.serial_conn:
                                    self.serial_conn.write(b'C')
                            else:
                                outf = Float32()
                                outf.data = dist
                                self.destinationFalse.publish(outf)
                                if self.serial_conn:
                                    self.serial_conn.write(f"D{dist:.2f}\n".encode())
                            self.sendFrame = False
                            self.get_logger().warn("sendFrame=False")

                        cv2.putText(frame, f"dist: {dist:.1f}", (10,30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)

            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners_list, ids)
            cv2.rectangle(frame, (bx,by), (bx+box_size,by+box_size), (0,255,0),2)
            cv2.imshow("Camera live stream", frame)
            if cv2.waitKey(1) == ord('q'):
                self.sendFrame = True

        self.cap.release()
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
