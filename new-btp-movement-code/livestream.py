import cv2
import socket
import pickle

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
ip = "10.110.141.161"
port = 9998  # Change the port number
s.bind(('', port))
print("Listening at:", s)

while True:
    try:
        x = s.recvfrom(1000000)
        data = x[0]
        print("Data received.")
        data = pickle.loads(data)
        data = cv2.imdecode(data, cv2.IMREAD_COLOR)
        cv2.imshow('server', data)
        
        if cv2.waitKey(10) == 13:
            break
    except socket.error as e:
        print(f"Error receiving data: {e}")

cv2.destroyAllWindows()