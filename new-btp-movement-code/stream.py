# import cv2
# import socket
# import pickle
# import numpy as np

# s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# s.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1000000)
# serverip = "192.168.1.89"
# serverport = 9999  # Make sure this matches the port the server is listening on
# cap = cv2.VideoCapture(0)
# cap.set(3,640)
# cap.set(4,480)
# print("sending at:", s)
# while True:
#     ret, photo = cap.read()
#     cv2.imshow('streaming', photo)
#     ret, buffer = cv2.imencode(".jpg", photo, [int(cv2.IMWRITE_JPEG_QUALITY), 30])
#     x_as_bytes = pickle.dumps(buffer)
#     s.sendto(x_as_bytes, (serverip, serverport))
#     key = cv2.waitKey(1)
#     if key == 27:
#         break

# cv2.destroyAllWindows()
# cap.release()
import cv2
import socket
import pickle
from realsense_depth import DepthCamera  # Assuming this is your custom class for capturing depth frames

class ArDetector:
    def __init__(self):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 10000000)
        self.server_ip = "192.168.1.89"
        self.server_port = 9998
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.video_capture = DepthCamera()  # Use your custom class for capturing depth frames

    def run(self):
        while True:
            ret, depth_frame, color_frame = self.video_capture.get_frame()
            ret, buffer = cv2.imencode(".jpg", color_frame, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
            data = pickle.dumps(buffer)
            self.s.sendto(data, (self.server_ip, self.server_port))

            # Your AR marker detection and processing code here
            # ...

            cv2.imshow('ArUco Marker Detection with Confidence', color_frame)
            key = cv2.waitKey(1)
            if key == 27:
                break

        # Release resources before exiting
        self.video_capture.release()
        cv2.destroyAllWindows()
        self.s.close()

if __name__ == "__main__":
    detector = ArDetector()
    detector.run()