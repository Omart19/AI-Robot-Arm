# Vision.py (Optimized version)

import socket
import struct
import cv2
import numpy as np

# Start socket
server = socket.socket()
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind(("127.0.0.1", 34567))
server.listen(1)

print("[Vision.py] Waiting for app connection...")
client, addr = server.accept()
print(f"[Vision.py] App connected from {addr}")


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
    # --- Receive frame from app ---
    size_data = client.recv(4)
    if len(size_data) < 4:
        print("[Vision.py] size data below 4")
        continue

    packet_size = struct.unpack('>I', size_data)[0]
    frame_data = b""
    while len(frame_data) < packet_size:
        frame_data += client.recv(packet_size - len(frame_data))

    # --- Decode JPEG ---
    np_arr = np.frombuffer(frame_data, dtype=np.uint8)
    frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    if frame is None:
        print("[Vision.py] Failed to decode frame")
        continue

    # Flip if needed
    frame = cv2.rotate(frame, cv2.ROTATE_180)

    # --- Create a smaller frame for faster processing ---
    small_frame = cv2.resize(frame, (640, 360))  # much faster detection
    scale_x = frame.shape[1] / small_frame.shape[1]
    scale_y = frame.shape[0] / small_frame.shape[0]

    # --- Face Detection ---
    gray = cv2.cvtColor(small_frame, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.2, 4)

    for (x, y, w, h) in faces:
        cv2.rectangle(
            frame,
            (int(x * scale_x), int(y * scale_y)),
            (int((x + w) * scale_x), int((y + h) * scale_y)),
            (0, 255, 0),
            2
        )

    # --- Object Detection ---
    blob = cv2.dnn.blobFromImage(small_frame, 0.007843, (300, 300), 127.5)
    net.setInput(blob)
    detections = net.forward()

    for i in range(detections.shape[2]):
        confidence = detections[0, 0, i, 2]
        if confidence > 0.5:
            idx = int(detections[0, 0, i, 1])
            label = classNames[idx] if idx < len(classNames) else "unknown"

            box = detections[0, 0, i, 3:7] * [
                small_frame.shape[1], small_frame.shape[0],
                small_frame.shape[1], small_frame.shape[0]
            ]
            (startX, startY, endX, endY) = box.astype("int")

            # Scale coordinates back
            startX = int(startX * scale_x)
            startY = int(startY * scale_y)
            endX = int(endX * scale_x)
            endY = int(endY * scale_y)

            cv2.rectangle(frame, (startX, startY), (endX, endY), (0, 0, 255), 2)
            cv2.putText(frame, label, (startX, startY - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    # --- Re-encode processed full-size frame ---
    success, processed_jpeg = cv2.imencode('.jpg', frame)
    if not success:
        print("[Vision.py] JPEG encode failed")
        continue

    processed_bytes = processed_jpeg.tobytes()

    # --- Send processed frame back ---
    client.sendall(struct.pack('>I', len(processed_bytes)))
    client.sendall(processed_bytes)
