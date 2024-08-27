import cv2

aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)  # Choose your desired dictionary
parameters = cv2.aruco.DetectorParameters_create()

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
    exit()

while cap.isOpened():
    ret, frame = cap.read()

    if not ret:
        print("Can't receive frame. Exiting ...")
        break

    # boolean to determine if arm is in right positiono with battery
    armstart=False
    # Get frame dimensions
    height, width, _ = frame.shape

    #set middle box height and weight
    box_size = 50
    box_x = int((width - box_size) / 2)
    box_y = int((height - box_size) / 2)

    # Draw the center box on the frame
    cv2.rectangle(frame, (box_x, box_y), (box_x + box_size, box_y + box_size), (0, 255, 0), 2)
    
    cv2.imshow("Camera live stream", frame)

    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()