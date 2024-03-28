import cv2
import socket
import pickle

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
serverip = "10.110.141.161"
serverport = 9998  # Make sure this matches the port the server is listening on
cap = cv2.VideoCapture(0)

while True:
    ret, photo = cap.read()
    cv2.imshow('streaming', photo)
    
    ret, buffer = cv2.imencode(".jpg", photo, [int(cv2.IMWRITE_JPEG_QUALITY), 30])
    x_as_bytes = pickle.dumps(buffer)
    
    try:
        print("Sending data...")
        s.sendto(x_as_bytes, (serverip, serverport))
        print("Data sent successfully.")
    except socket.error as e:
        print(f"Error sending data: {e}")
    
    if cv2.waitKey(10) == 13:
        break

cv2.destroyAllWindows()
cap.release()