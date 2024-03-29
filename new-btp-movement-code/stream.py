import cv2
from realsense_depth import *
import matplotlib.pyplot as plt     
from cv2 import aruco
import pyrealsense2
import numpy as np
import pickle
import socket

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1000000)
serverip = "10.110.141.161"
serverport = 9998  # Make sure this matches the port the server is listening on

# 1. Load the ArUco dictionary
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)  # Choose your desired dictionary
parameters = cv2.aruco.DetectorParameters_create()

# 2. Access video capture
video_capture = DepthCamera()  # Use 0 for webcam, or file path for a video file


print("sending at:", s)
while True:
    ret, depth_frame,color_frame = video_capture.get_frame()
    cv2.imshow('streaming', color_frame)
    #set middle box height and weight
    height, width, _ = color_frame.shape
    box_size = 50
    box_x = int((width - box_size) / 2)
    box_y = int((height - box_size) / 2)

    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(color_frame, aruco_dict, parameters=parameters)
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 30]
    ret, color_buffer = cv2.imencode(".jpg", color_frame, encode_param)
    color_data = color_buffer.tobytes()

    # Serialize data
    data = {
        "color_data": color_data,
        "depth_frame": depth_frame,
        "aruco_markers": (corners, ids)
    }
    serialized_data = pickle.dumps(data)

    # Send serialized data
    s.sendto(serialized_data, (serverip, serverport))
    key = cv2.waitKey(1)
    if key == 27:
        break

cv2.destroyAllWindows()
video_capture.release()
# import cv2
# import socket
# import pickle
# from realsense_depth import DepthCamera  # Assuming this is your custom class for capturing depth frames

# class ArDetector:
#     def __init__(self):
#         self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#         self.s.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 10000000)
#         self.server_ip = "192.168.1.89"
#         self.server_port = 9998
#         self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)
#         self.parameters = cv2.aruco.DetectorParameters_create()
#         self.video_capture = DepthCamera()  # Use your custom class for capturing depth frames

#     def run(self):
#         while True:
#             ret, depth_frame, color_frame = self.video_capture.get_frame()
#             ret, buffer = cv2.imencode(".jpg", color_frame, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
#             data = pickle.dumps(buffer)
#             self.s.sendto(data, (self.server_ip, self.server_port))

#             # Your AR marker detection and processing code here
#             # ...

#             cv2.imshow('ArUco Marker Detection with Confidence', color_frame)
#             key = cv2.waitKey(1) 
#             if key == 27: 
#                 break

#         # Release resources before exiting
#         self.video_capture.release()
#         cv2.destroyAllWindows()
#         self.s.close()

# if __name__ == "__main__":
#     detector = ArDetector()
#     detector.run()