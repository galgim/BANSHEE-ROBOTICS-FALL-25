import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import threading
import asyncio
import ssl
import websockets
from std_msgs.msg import Int8

class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")
        self.x_values_equal = 0
        self.arucoID = None

        # ROS 2 subscription
        self.subscription = self.create_subscription(
            Int8, 
            'arucoID', 
            self.arucoSubscriber, 
            10
        )

        # Start the camera thread
        self.camera_thread = threading.Thread(target=self.cameraRun)
        self.camera_thread.start()

        self.get_logger().info("Camera node initialized")

    def arucoSubscriber(self, msg):
        self.arucoID = int(msg.data)
        self.get_logger().info(f"Received Aruco ID: {self.arucoID}")

    def cameraRun(self):
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
        detector_parameters = cv2.aruco.DetectorParameters()
        refine_parameters = cv2.aruco.RefineParameters()
        cap = cv2.VideoCapture(0)

        if not cap.isOpened():
            self.get_logger().error("Cannot open camera")
            return

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                self.get_logger().error("Can't receive frame. Exiting ...")
                break

            # Get frame dimensions and setup for marker detection
            height, width, _ = frame.shape
            box_size = 50
            box_x = int((width - box_size) / 2)
            box_y = int((height - box_size) / 2)

            # Detect ArUco markers
            detector = cv2.aruco.ArucoDetector(aruco_dict, detector_parameters, refine_parameters)
            marker_corners, marker_ids, _ = detector.detectMarkers(frame)

            # Only process if an Aruco ID is set via the subscriber
            if marker_ids is not None and self.arucoID is not None:
                for ids, corners in zip(marker_ids, marker_corners):
                    if ids == self.arucoID:
                        corner1_x = int(corners[0][0][0])
                        corner2_x = int(corners[0][2][0])
                        middle_x = (corner1_x + corner2_x) // 2
                        distance = abs(middle_x - int(width / 2))

                        cv2.putText(frame, f"distance: {distance}", (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

                        # Compute overlap ratio
                        middle_box = np.array([[box_x, box_y], [box_x + box_size, box_y + box_size]])
                        aruco_box = np.int32(corners[0])
                        intersection_area = cv2.contourArea(cv2.convexHull(np.concatenate([middle_box, aruco_box])))
                        union_area = box_size ** 2 + cv2.contourArea(cv2.convexHull(aruco_box)) - intersection_area
                        if union_area == 0:
                            union_area = 0.01
                        overlap_ratio = intersection_area / union_area

                        if overlap_ratio >= 0.98:
                            # Logic for arm activation (if needed)
                            pass

                        cv2.putText(frame, f"Overlap Ratio: {overlap_ratio:.2%}",
                                    (int(corners[0][:, 0].mean()), int(corners[0][:, 1].mean()) + 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Draw markers and the center box
            if marker_ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, marker_corners, marker_ids)
            cv2.rectangle(frame, (box_x, box_y), (box_x + box_size, box_y + box_size), (0, 255, 0), 2)

            # Display the frame
            cv2.imshow("Camera live stream", frame)

            if cv2.waitKey(1) == ord('q'):
                exit()
                break

        cap.release()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)  # Keep the node alive
    finally:
        node.camera_thread.join()  # Wait for the camera thread to finish
        rclpy.shutdown()

if __name__ == '__main__':
    main()
