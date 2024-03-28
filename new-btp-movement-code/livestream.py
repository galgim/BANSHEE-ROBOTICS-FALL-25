import cv2
import numpy as np
import pickle
import socket

s=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
ip="192.168.1.89"
port=6666
s.bind((ip,port))

# 1. Load the ArUco dictionary
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)  # Choose your desired dictionary
parameters = cv2.aruco.DetectorParameters_create()

while True:
    x=s.recvfrom(1000000)
    clientip=x[1][0]
    data=x[0]

    data=pickle.loads(data)

    img=cv2.imdecode(data,cv2.IMREAD_COLOR)
    cv2.imshow('img server',img)
   
    key= cv2.waitKey(1)
    if key==27:
        break
cv2.destroyAllWindows()