# Pi Camera + Command Server
# Very lightweight - streams MJPEG frames over TCP and listens for Arduino commands

import socket
import threading
import subprocess
import serial
import struct
import time

# === Configuration ===
CAMERA_PORT = 23456       # Port for MJPEG streaming
COMMAND_PORT = 23457       # Port for receiving control commands
SERIAL_PORT = '/dev/ttyACM0'  # Arduino USB port
BAUD_RATE = 9600
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
FRAME_RATE = 26


import smbus2
import time

class MagneticEncoderController:
    def __init__(self, bus_id=1, tca_address=0x70):
        self.bus_id = bus_id
        self.tca_address = tca_address
        self.sensor_addresses = {
            0: 0x36,  # Channel 0
            1: 0x36,  # Channel 1
            2: 0x36,  # Channel 2
            3: 0x36   # Channel 3
        }
        self.bus = smbus2.SMBus(bus_id)

    def select_tca_channel(self, channel):
        """Selects the TCA9548A multiplexer channel."""
        channel_mask = 1 << channel
        self.bus.write_byte(self.tca_address, channel_mask)
        time.sleep(0.01)  # Small delay to stabilize
        # print(f"[I2C] Switched to TCA Channel {channel}")

    def read_sensor_value(self, sensor_address):
        """Reads 12-bit angle value from AS5600 sensor."""
        try:
            # Send pointer to raw angle register
            self.bus.write_byte(sensor_address, 0x0C)
            # Read 2 bytes
            data = self.bus.read_i2c_block_data(sensor_address, 0x0C, 2)
            # Combine MSB and LSB into 12-bit value
            raw_angle = ((data[0] << 8) | data[1]) & 0x0FFF
            return raw_angle
        except Exception as e:
            print(f"[I2C ERROR] Failed to read from sensor at 0x{sensor_address:X}: {e}")
            return -1

    def read_all_encoders(self):
        """Reads all 4 encoders through the TCA9548A."""
        angles = []
        for channel in self.sensor_addresses.keys():
            try:
                self.select_tca_channel(channel)
                angle = self.read_sensor_value(self.sensor_addresses[channel])
                angles.append(angle)
            except Exception as e:
                print(f"[I2C ERROR] Channel {channel}: {e}")
                angles.append(-1)  # Error fallback
        return angles

# === Shared Variables and Lock ===
latest_frame_data = None
latest_encoder_readings_str = "ENCODERS:-1,-1,-1,-1\n" # Default/initial value
frame_lock = threading.Lock()
keep_running = True # Flag to signal threads to stop
# === Start libcamera-vid as a subprocess ===
print("[INFO] Starting libcamera-vid...")
# Initial start - might move into frame_reader_thread for robustness
try:
    cam_proc = subprocess.Popen(
        ["libcamera-vid", "-t", "0", "--codec", "mjpeg", "-o", "-",
         "--width", str(FRAME_WIDTH), "--height", str(FRAME_HEIGHT),
         "--framerate", str(FRAME_RATE), "--nopreview",
         "--mode", "2304:1296:SRGGB10"], # Added your specific mode back
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE, # Keep stderr if you want to monitor errors
    )
except FileNotFoundError:
    print("[ERROR] libcamera-vid command not found. Make sure it's installed and in PATH.")
    cam_proc = None
except Exception as e:
    print(f"[ERROR] Failed to start libcamera-vid: {e}")
    cam_proc = None

# === Open Arduino Serial Connection ===
print(f"[INFO] Connecting to Arduino at {SERIAL_PORT}...")
try:
    arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)  # Wait for Arduino reset
    print("[INFO] Arduino connected.")
except serial.SerialException as e:
    print(f"[ERROR] Failed to open Arduino serial: {e}")
    arduino = None

# === Thread for Reading Camera Frames and Encoders ===
def frame_reader_thread():
    global latest_frame_data, latest_encoder_readings_str, keep_running, cam_proc

    if not cam_proc or cam_proc.stdout is None:
        print("[ERROR] Camera process not started correctly. Frame reader thread exiting.")
        keep_running = False
        return

    encoder_controller = MagneticEncoderController()
    buffer = b""
    print("[INFO] Frame reader thread started.")

    while keep_running:
        try:
            chunk = cam_proc.stdout.read(4096) # Read from camera process stdout
            if not chunk:
                print("[WARNING] Camera process stdout stream ended.")
                # Optionally try to restart cam_proc here if desired
                time.sleep(1) # Avoid busy-looping if process dies
                continue # Or break if you want the thread to exit

            buffer += chunk

            # Find the start and end markers for MJPEG frames
            start = buffer.find(b'\xff\xd8')
            end = buffer.find(b'\xff\xd9')

            if start != -1 and end != -1 and end > start:
                frame = buffer[start:end + 2]
                buffer = buffer[end + 2:] # Keep the remainder

                # Read encoders *once* per frame found
                encoder_readings = encoder_controller.read_all_encoders()
                temp_encoder_str = f"ENCODERS:{encoder_readings[0]},{encoder_readings[1]},{encoder_readings[2]},{encoder_readings[3]}\n"

                # Update shared variables under lock
                with frame_lock:
                    latest_frame_data = frame
                    latest_encoder_readings_str = temp_encoder_str

        except Exception as e:
            print(f"[ERROR] Error in frame reader thread: {e}")
            # Decide if the error is fatal or recoverable
            time.sleep(0.5) # Avoid spamming errors

    print("[INFO] Frame reader thread stopped.")
    if cam_proc:
        cam_proc.terminate() # Ensure camera stops if reader stops

# === Thread for Handling Individual Client Connections ===
def handle_client(client_socket, addr):
    global latest_frame_data, latest_encoder_readings_str, frame_lock
    print(f"[CAMERA] Client connected from {addr}")
    send_interval = 1.0 / FRAME_RATE # Target interval between frames

    try:
        with client_socket:
            while keep_running:
                #print(f"[HANDLE_CLIENT {addr}] Loop start") # ADDED
                start_time = time.monotonic()
                local_frame = None
                local_encoder_str = ""

                #print(f"[HANDLE_CLIENT {addr}] Acquiring lock...") # ADDED
                with frame_lock:
                    #print(f"[HANDLE_CLIENT {addr}] Lock acquired.") # ADDED
                    if latest_frame_data:
                        local_frame = latest_frame_data
                        local_encoder_str = latest_encoder_readings_str
                        #print(f"[HANDLE_CLIENT {addr}] Got frame (size={len(local_frame)}), encoder='{local_encoder_str.strip()}'") # ADDED
                    else:
                        print(f"[HANDLE_CLIENT {addr}] No frame data available yet.") # ADDED
                    # Release lock happens automatically with 'with' statement
                    print(f"[HANDLE_CLIENT {addr}] Lock released.") # ADDED

                if local_frame:
                    try:
                        #print(f"[HANDLE_CLIENT {addr}] Encoding packet...") # ADDED
                        encoder_bytes = local_encoder_str.encode('utf-8')
                        packet = encoder_bytes + local_frame
                        packet_size = len(packet)
                        #print(f"[HANDLE_CLIENT {addr}] Packet size: {packet_size}") # ADDED

                        #print(f"[HANDLE_CLIENT {addr}] Sending size...") # ADDED
                        client_socket.sendall(struct.pack('>I', packet_size))
                        #print(f"[HANDLE_CLIENT {addr}] Sending packet...") # ADDED
                        client_socket.sendall(packet)
                        #print(f"[HANDLE_CLIENT {addr}] Send complete.") # ADDED

                    except (socket.error, BrokenPipeError, ConnectionResetError) as e:
                        #print(f"[HANDLE_CLIENT {addr}] Socket error during send: {e}") # MODIFIED
                        break
                    except Exception as e:
                        #print(f"[HANDLE_CLIENT {addr}] UNEXPECTED error during send: {e}") # ADDED
                        break # Also break on other unexpected errors here

                else:
                    # If no frame, still sleep to prevent busy-waiting
                    pass

                # Sleep logic
                elapsed_time = time.monotonic() - start_time
                sleep_time = max(0, send_interval - elapsed_time)
                #print(f"[HANDLE_CLIENT {addr}] Calculated sleep: {sleep_time:.4f} (elapsed: {elapsed_time:.4f})") # ADDED
                time.sleep(sleep_time)
                #print(f"[HANDLE_CLIENT {addr}] Woke up from sleep.") # ADDED

    except Exception as e:
        # Catch broader exceptions that might crash the thread outside the send block
         print(f"[HANDLE_CLIENT {addr}] UNHANDLED EXCEPTION IN THREAD: {e}") # ADDED
    finally:
        print(f"[CAMERA] Cleaning up connection for {addr}")
        client_socket.close()


# === MJPEG Streaming Server (Accepts connections, starts client handlers) ===
def camera_stream_server():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        server.bind(('0.0.0.0', CAMERA_PORT))
        server.listen(5) # Allow a small backlog of connections
        #print(f"[CAMERA] Listening on port {CAMERA_PORT}")
    except Exception as e:
        #print(f"[ERROR] Failed to bind camera server socket: {e}")
        return # Cannot continue

    while keep_running:
        try:
            client_socket, addr = server.accept()
            # Start a new thread for each client
            client_thread = threading.Thread(target=handle_client, args=(client_socket, addr), daemon=True)
            client_thread.start()
        except Exception as e:
            if keep_running: # Avoid error message if we are shutting down
                print(f"[ERROR] Error accepting camera client connection: {e}")
            break # Exit server loop on major error

    print("[INFO] Camera stream server stopped.")
    server.close()
# === Command Server (for Arduino commands) ===
def command_server():
       server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
       server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
       server.bind(('0.0.0.0', COMMAND_PORT))
       server.listen(1)
       print(f"[COMMAND] Listening on port {COMMAND_PORT}")

       while keep_running:
           try:
               client_socket, addr = server.accept()
               print(f"[COMMAND] {time.time()} Command client connected from {addr}")
               with client_socket:
                   while keep_running:
                       try:
                           print(f"[COMMAND] {time.time()} Waiting for data from {addr}")
                           data = client_socket.recv(1024)
                           if not data:
                               print(f"[COMMAND] {time.time()} Client {addr} disconnected.")
                               break
                           print(f"[COMMAND] {time.time()} Received raw data from {addr}: {data!r}") # Log raw bytes!
                           command = data.decode('utf-8').strip()
                           command = command.lstrip('\ufeff')
                           print(f"[COMMAND] {time.time()} Decoded command from {addr}: {command}")
                           if arduino:
                               try:
                                   arduino.write((command + '\n').encode('utf-8'))
                                   print(f"[COMMAND] {time.time()} Sent to Arduino: {command!r}") # Log raw bytes!
                               except serial.SerialException as e:
                                   print(f"[COMMAND] {time.time()} Serial write error: {e}")
                       except socket.error as e:
                           print(f"[COMMAND] {time.time()} Socket error with client {addr}: {e}")
                           break
                       except Exception as e:
                           print(f"[COMMAND] {time.time()} Error handling client {addr}: {e}")
                           break
           except socket.error as e:
               if keep_running:
                   print(f"[COMMAND] {time.time()} Server socket error: {e}")
               break
           except Exception as e:
               if keep_running:
                   print(f"[COMMAND] {time.time()} Unexpected server error: {e}")
               break

       print(f"[COMMAND] {time.time()} Command server stopped.")
       server.close()

# === Start Threads ===
if cam_proc: # Only start servers if camera started successfully
    threading.Thread(target=frame_reader_thread, daemon=True).start()
    threading.Thread(target=camera_stream_server, daemon=True).start()
    threading.Thread(target=command_server, daemon=True).start()
else:
    print("[ERROR] Camera process failed to start. Servers not launched.")

# === Main loop ===
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("\n[INFO] Shutting down...")
    cam_proc.terminate()
    if arduino:
        arduino.close()# === Main loop & Cleanup ===
try:
    while keep_running:
        # Keep main thread alive, threads are daemonic
        # Check if threads are alive if needed
        time.sleep(1)
except KeyboardInterrupt:
    print("\n[INFO] Shutdown requested by user (Ctrl+C)...")
finally:
    print("[INFO] Stopping threads and cleaning up...")
    keep_running = False # Signal threads to stop
    time.sleep(1.5) # Give threads a moment to potentially exit gracefully

    if cam_proc:
        print("[INFO] Terminating camera process...")
        cam_proc.terminate()
        try:
            cam_proc.wait(timeout=2) # Wait briefly for termination
        except subprocess.TimeoutExpired:
            print("[WARNING] Camera process did not terminate gracefully, killing.")
            cam_proc.kill()
        cam_proc = None

    if arduino and arduino.is_open:
        print("[INFO] Closing Arduino connection...")
        arduino.close()
        arduino = None

    print("[INFO] Shutdown complete.")
