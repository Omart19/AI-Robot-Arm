# Vision.py (Optimized version)

import socket
import struct
import cv2
import numpy as np
import json # For JSON serialization
import time # Ensure time is imported if used for debugging

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

try:  # <<< Add try-finally to ensure client closes on error >>>
    while True:
        # --- Receive frame from app ---
        size_data = client.recv(4)
        if len(size_data) < 4:
            print("[Vision.py] Client disconnected or sent insufficient size data.")
            break  # <<< Exit loop if client disconnects >>>

        packet_size = struct.unpack('>I', size_data)[0]
        frame_data = b""
        while len(frame_data) < packet_size:
            chunk = client.recv(packet_size - len(frame_data))
            if not chunk:
                print("[Vision.py] Client disconnected during frame data receive.")
                raise ConnectionAbortedError("Client disconnected")  # <<< Raise error to exit outer loop >>>
            frame_data += chunk

        # --- Decode JPEG ---
        np_arr = np.frombuffer(frame_data, dtype=np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is None:
            print("[Vision.py] Failed to decode frame")
            # Send back an empty detection list and no image, or a minimal error indicator
            error_detections = []
            error_detections_json = json.dumps(error_detections).encode('utf-8')
            client.sendall(struct.pack('>I', len(error_detections_json)))
            client.sendall(error_detections_json)
            client.sendall(struct.pack('>I', 0))  # 0-length image
            continue

        # Flip if needed
        frame = cv2.rotate(frame, cv2.ROTATE_180)

        # --- Create a smaller frame for faster processing ---
        small_frame = cv2.resize(frame, (640, 360))
        scale_x = frame.shape[1] / small_frame.shape[1]
        scale_y = frame.shape[0] / small_frame.shape[0]

        detections_list = []  # <<< Initialize list to store detection data >>>

        # --- Face Detection ---
        gray = cv2.cvtColor(small_frame, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, 1.2, 4)

        for (x, y, w, h) in faces:
            # Scaled coordinates for drawing
            draw_x1, draw_y1 = int(x * scale_x), int(y * scale_y)
            draw_x2, draw_y2 = int((x + w) * scale_x), int((y + h) * scale_y)
            cv2.rectangle(frame, (draw_x1, draw_y1), (draw_x2, draw_y2), (0, 255, 0), 2)
            # <<< Add face detection to list (using original frame coordinates) >>>
            detections_list.append({
                "label": "face",
                "confidence": 1.0,  # Haar cascades don't typically give confidence
                "box": [draw_x1, draw_y1, draw_x2, draw_y2]  # x1, y1, x2, y2
            })

        # --- Object Detection ---
        blob = cv2.dnn.blobFromImage(small_frame, 0.007843, (300, 300), 127.5)
        net.setInput(blob)
        detections_output = net.forward()  # Renamed to avoid conflict

        for i in range(detections_output.shape[2]):
            confidence = float(detections_output[0, 0, i, 2])  # <<< Convert to float for JSON >>>
            if confidence > 0.5:
                idx = int(detections_output[0, 0, i, 1])
                label = classNames[idx] if idx < len(classNames) else "unknown"

                box = detections_output[0, 0, i, 3:7] * [
                    small_frame.shape[1], small_frame.shape[0],
                    small_frame.shape[1], small_frame.shape[0]
                ]
                (startX_small, startY_small, endX_small, endY_small) = box.astype("int")

                # Scale coordinates back to original frame size
                startX = int(startX_small * scale_x)
                startY = int(startY_small * scale_y)
                endX = int(endX_small * scale_x)
                endY = int(endY_small * scale_y)

                cv2.rectangle(frame, (startX, startY), (endX, endY), (0, 0, 255), 2)
                cv2.putText(frame, f"{label}: {confidence:.2f}", (startX, startY - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                # <<< Add object detection to list >>>
                detections_list.append({
                    "label": label,
                    "confidence": confidence,
                    "box": [startX, startY, endX, endY]  # x1, y1, x2, y2
                })

        # --- Re-encode processed full-size frame ---
        success, processed_jpeg_encoded = cv2.imencode('.jpg', frame)  # Renamed
        if not success:
            print("[Vision.py] JPEG encode failed")
            # Send back an empty detection list and no image
            error_detections = []
            error_detections_json = json.dumps(error_detections).encode('utf-8')
            client.sendall(struct.pack('>I', len(error_detections_json)))
            client.sendall(error_detections_json)
            client.sendall(struct.pack('>I', 0))  # 0-length image
            continue

        processed_bytes = processed_jpeg_encoded.tobytes()

        # --- Prepare JSON data ---
        detections_json_string = json.dumps(detections_list)
        detections_json_bytes = detections_json_string.encode('utf-8')

        # --- Send detection JSON data back ---
        # 1. Send length of JSON data
        client.sendall(struct.pack('>I', len(detections_json_bytes)))
        # 2. Send JSON data
        client.sendall(detections_json_bytes)

        # --- Send processed frame back (as before) ---
        # 3. Send length of image data
        client.sendall(struct.pack('>I', len(processed_bytes)))
        # 4. Send image data
        client.sendall(processed_bytes)

        # print(f"[Vision.py] Sent detections: {len(detections_list)}, Image size: {len(processed_bytes)}")


except (ConnectionAbortedError, ConnectionResetError, BrokenPipeError) as e:
    print(f"[Vision.py] Connection error: {e}")
except Exception as e:
    print(f"[Vision.py] An unexpected error occurred: {e}")
finally:
    print("[Vision.py] Closing client socket.")
    client.close()
    server.close()
    print("[Vision.py] Server shut down.")