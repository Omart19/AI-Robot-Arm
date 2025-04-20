import cv2
import socket
import struct
import sys

print("? vision.py starting...", flush=True)

# Setup socket
server = socket.socket()
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind(("127.0.0.1", 23456))
server.listen(1)
print("? bound to port 23456", flush=True)

client, addr = server.accept()
print(f"? client connected from {addr}", flush=True)

# Capture stream from libcamera-vid
cap = cv2.VideoCapture("pipe:0")
if not cap.isOpened():
    print("? Could not open stream", flush=True)
    sys.exit(1)

print("? Camera stream opened from pipe:0", flush=True)

# Face detection
face_cascade = cv2.CascadeClassifier("/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml")
print("? Loaded face cascade?", not face_cascade.empty(), flush=True)

# Load object detection model (MobileNet SSD)
net = cv2.dnn.readNetFromCaffe(
    "/home/ergy/Shared/RobotAIArm9/PythonScripts/MobileNetSSD_deploy.prototxt",
    "/home/ergy/Shared/RobotAIArm9/PythonScripts/MobileNetSSD_deploy.caffemodel"
)
print("? Loaded MobileNet SSD", flush=True)

# List of class names
classNames = [
    "background", "aeroplane", "bicycle", "bird", "boat", "bottle",
    "bus", "car", "cat", "chair", "cow", "diningtable", "dog", "horse",
    "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"
]

while True:
    ret, frame = cap.read()
    if not ret:
        print("? Frame read failed", flush=True)
        continue

    # Flip 180if camera is upside-down
    frame = cv2.rotate(frame, cv2.ROTATE_180)

    # Face detection (green boxes)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.2, 4)
    #print(f"? Detected {len(faces)} face(s)", flush=True)
    for (x, y, w, h) in faces:
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 4)  # green

    # Object detection (red boxes)
    blob = cv2.dnn.blobFromImage(frame, 0.007843, (300, 300), 127.5)
    net.setInput(blob)
    detections = net.forward()

    for i in range(detections.shape[2]):
        confidence = detections[0, 0, i, 2]
        if confidence > 0.5:
            idx = int(detections[0, 0, i, 1])
            label = classNames[idx] if idx < len(classNames) else "unknown"

            box = detections[0, 0, i, 3:7] * [frame.shape[1], frame.shape[0], frame.shape[1], frame.shape[0]]
            (startX, startY, endX, endY) = box.astype("int")

            cv2.rectangle(frame, (startX, startY), (endX, endY), (0, 0, 255), 3)  # red
            cv2.putText(frame, label, (startX, startY - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    # Encode and send JPEG
    success, jpeg = cv2.imencode('.jpg', frame)
    if not success:
        print("? JPEG encode failed", flush=True)
        continue

    jpeg_bytes = jpeg.tobytes()
    try:
        client.sendall(struct.pack(">I", len(jpeg_bytes)))
        client.sendall(jpeg_bytes)
        #print(f"? Sent frame: {len(jpeg_bytes)} bytes", flush=True)
    except Exception as e:
        print(f"? Socket send failed: {e}", flush=True)
        break
