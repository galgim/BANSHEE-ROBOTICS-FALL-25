import asyncio
import websockets
import cv2

async def send_frames():
    #uri = "ws://172.233.146.163:3000/sending_frames"  # WebSocket endpoint for reading frames
    uri = "ws://rgs.bansheeuav.tech:3000/sending_frames"
    #uri = "ws://127.0.0.1:3000/sending_frames"
    async with websockets.connect(uri) as websocket:
        print("Connected to WebSocket server for sending frames")
        cap = cv2.VideoCapture(0)
        while True:
            try:
                ret, frame = cap.read()
                if not ret: 
                    break
                frame_bytes = cv2.imencode('.jpg', frame)[1].tobytes()
                ret = await websocket.send(frame_bytes)
                print("Sent frame")
            except Exception as e:
                print(e)
                break

asyncio.run(send_frames())