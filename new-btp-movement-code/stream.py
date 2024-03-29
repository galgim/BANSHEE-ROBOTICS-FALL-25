import cv2
import socket
import pickle

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
serverip = "192.168.1.95"
serverport = 9998  # Make sure this matches the port the server is listening on
cap = cv2.VideoCapture(0)

while True:
    ret, photo = cap.read()
    cv2.imshow('streaming', photo)
    ret, buffer = cv2.imencode(".jpg", photo, [int(cv2.IMWRITE_JPEG_QUALITY), 30])
    x_as_bytes = pickle.dumps(buffer)
    s.sendto(x_as_bytes, (serverip, serverport))
    key = cv2.waitKey(1)
    if key == 27:
        break

cv2.destroyAllWindows()
cap.release()