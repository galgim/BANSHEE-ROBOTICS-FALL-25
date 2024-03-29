import cv2
import socket
import pickle

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
ip = "192.168.1.89"
port = 9998  # Change the port number
s.bind(('', port))
print("Listening at:", s)

while True:
      x = s.recvfrom(1000000)
      data = x[0]
      print("streaming")
      data = pickle.loads(data)
      data = cv2.imdecode(data, cv2.IMREAD_COLOR)
      cv2.imshow('server', data)
      
      key = cv2.waitKey(1)
      if key == 27:
        break

cv2.destroyAllWindows()