import cv2
import numpy as np
import pickle
import socket
from realsense_depth import *

class livestream:
    s=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ip="192.168.1.89"
    port=6666
    s.bind((ip,port))
    while True:
        x=s.recvfrom(1000000)
        clientip=x[1][0]
        data=x[0]

        data=pickle.loads(data)

        img=cv2.imdecode(data,cv2.IMREAD_COLOR)
        cv2.imshow('Img Server', img)
        key= cv2.waitKey(1)
        if key==27:
            break
    cv2.destroyAllWindows()