import socket
import struct
import cv2
import numpy as np

# Listen for incoming frames from app
server = socket.socket()
server.bind(('127.0.0.1', 34567))
server.listen(1)
print("[Vision] Waiting for App to connect...")

client, addr = server.accept()
print(f"[Vision] App connected from {addr}")

# Load models
face_cascade = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")
net = cv2.dnn.readNetFromCaffe(
    "MobileNetSSD_deploy.prototxt",
    "MobileNetSSD_deploy.caffemodel"
)

classNames = [
    "background", "aeroplane", "bicycle", "bird", "boat", "bottle",
    "bus", "car", "cat", "chair", "cow", "diningtable", "dog", "horse",
    "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"
]

while True:
    # Receive size
    size_data = client.recv(4)
    if len(size_data) < 4:
        break
    frame_size = struct.unpack('>i', size_data)[0]

    # Receive frame
    frame_data = b''
    while len(frame_data) < frame_size:
        frame_data += client.recv(frame_size - len(frame_data))

    # Decode frame
    np_arr = np.frombuffer(frame_data, np.uint8)
    frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    if frame is None:
        print("[Vision] Failed to decode frame")
        continue

    # Face Detection (green)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.2, 4)
    for (x, y, w, h) in faces:
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # Object Detection (red)
    blob = cv2.dnn.blobFromImage(frame, 0.007843, (300, 300), 127.5)
    net.setInput(blob)
    detections = net.forward()

    for i in range(detections.shape[2]):
        confidence = detections[0, 0, i, 2]
        if confidence > 0.5:
            idx = int(detections[0, 0, i, 1])
            label = classNames[idx] if idx < len(classNames) else "unknown"

            box = detections[0, 0, i, 3:7] * [
                frame.shape[1], frame.shape[0], frame.shape[1], frame.shape[0]
            ]
            (startX, startY, endX, endY) = box.astype("int")
            cv2.rectangle(frame, (startX, startY), (endX, endY), (0, 0, 255), 2)
            cv2.putText(frame, label, (startX, startY - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    # Encode processed frame
    success, processed_jpeg = cv2.imencode('.jpg', frame)
    if not success:
        print("[Vision] Failed to encode processed frame")
        continue

    # Send processed frame back
    processed_bytes = processed_jpeg.tobytes()
    client.sendall(struct.pack('>i', len(processed_bytes)))
    client.sendall(processed_bytes)
