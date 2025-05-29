import socket
import threading
import subprocess
import serial
import struct
import time
import smbus2  # For I2C communication

# === Configuration ===
CAMERA_PORT = 23456
COMMAND_PORT = 23457
SERIAL_PORT_ARDUINO = '/dev/ttyACM0'
BAUD_RATE_ARDUINO = 9600
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
FRAME_RATE = 26
SERIAL_PORT_TOF_CAMERA = '/dev/serial0'
BAUD_RATE_TOF_CAMERA = 115200
I2C_BUS_ID = 1
TCA_ADDRESS = 0x70
ENCODER_SENSOR_I2C_ADDRESS = 0x36
ENCODER_CHANNELS = {
    0: ENCODER_SENSOR_I2C_ADDRESS, 1: ENCODER_SENSOR_I2C_ADDRESS,
    2: ENCODER_SENSOR_I2C_ADDRESS, 3: ENCODER_SENSOR_I2C_ADDRESS
}
TOF_HAND_I2C_ADDRESS = 0x29
TOF_HAND_MUX_CHANNEL = 4
TOF_READ_INTERVAL = 0.05  # For UART ToF thread loop

VL6180X_REG_SYSTEM_FRESH_OUT_OF_RESET = 0x0016
VL6180X_REG_SYSRANGE_START = 0x0018
VL6180X_REG_RESULT_INTERRUPT_STATUS_GPIO = 0x004F
VL6180X_REG_RESULT_RANGE_VAL = 0x0062
VL6180X_REG_SYSTEM_INTERRUPT_CLEAR = 0x0015

TOF050F_REG_RANGE_MODE = 0x0004
TOF050F_SLAVE_ID = 0x01


class I2CMultiplexerController:
    def __init__(self, bus_id=I2C_BUS_ID, tca_address=TCA_ADDRESS):
        self.bus_id = bus_id
        self.tca_address = tca_address
        self.bus = None
        self.is_initialized = False
        self.last_selected_channel = -1
        # self._lock = threading.Lock() # Lock not strictly needed if only one thread accesses MUX
        try:
            self.bus = smbus2.SMBus(self.bus_id)
            try:  # Test MUX presence
                self.bus.write_byte(self.tca_address, 1 << 0)  # Select channel 0
                current_mux_state = self.bus.read_byte(self.tca_address)
                print(
                    f"[I2C-MUX] SMBus {self.bus_id} initialized. MUX at 0x{self.tca_address:X} responded (Ch0, state: 0x{current_mux_state:X}).")
                self.is_initialized = True
            except Exception as mux_init_e:
                print(f"[I2C-MUX FATAL ERROR] MUX at 0x{self.tca_address:X} NOT RESPONDING: {mux_init_e}")
        except FileNotFoundError:
            print(f"[I2C-MUX FATAL ERROR] SMBus {self.bus_id} not found. Is I2C enabled & smbus2 installed?")
        except Exception as e:
            print(f"[I2C-MUX FATAL ERROR] Failed to initialize SMBus for MUX: {e}")

    def select_channel(self, channel, retries=1):
        if not self.is_initialized: return False
        if channel == self.last_selected_channel: return True
        for attempt in range(retries + 1):
            try:
                if not (0 <= channel <= 7):
                    print(f"[I2C-MUX ERROR] Channel {channel} out of range (0-7).")
                    return False
                self.bus.write_byte(self.tca_address, 1 << channel)
                time.sleep(0.001)  # Minimal MUX switch delay (1ms)
                self.last_selected_channel = channel
                return True
            except Exception as e:
                print(
                    f"[I2C-MUX ERROR Att.{attempt + 1}] Failed to select Ch {channel} on MUX 0x{self.tca_address:X}: {e}")
                if attempt < retries:
                    time.sleep(0.01)
                else:
                    self.last_selected_channel = -1; return False
        return False

    def get_smbus_instance(self):
        return self.bus if self.is_initialized else None


class MagneticEncoderController:
    def __init__(self, i2c_controller, sensor_config_map):
        self.i2c_controller = i2c_controller
        self.sensor_config = sensor_config_map

    def _read_as5600_angle(self, sensor_i2c_address):
        bus = self.i2c_controller.get_smbus_instance()
        if not bus: return -1
        try:
            data = bus.read_i2c_block_data(sensor_i2c_address, 0x0C, 2)
            return ((data[0] << 8) | data[1]) & 0x0FFF
        except Exception:
            return -1

    def read_all_encoders(self):
        angles = [-1] * len(self.sensor_config)
        if not self.i2c_controller.is_initialized: return angles
        for i, channel in enumerate(self.sensor_config.keys()):
            if self.i2c_controller.select_channel(channel):
                angles[i] = self._read_as5600_angle(self.sensor_config[channel])
            else:
                angles[i] = -2
        return angles


class I2CHandToFSensorController:
    VL6180X_REG_SYSTEM_FRESH_OUT_OF_RESET = 0x0016
    VL6180X_REG_SYSRANGE_START = 0x0018
    VL6180X_REG_RESULT_INTERRUPT_STATUS_GPIO = 0x004F
    VL6180X_REG_RESULT_RANGE_VAL = 0x0062
    VL6180X_REG_SYSTEM_INTERRUPT_CLEAR = 0x0015

    def __init__(self, i2c_mux_controller, mux_channel, sensor_i2c_address=TOF_HAND_I2C_ADDRESS,
                 sensor_name="HandToF_I2C"):
        self.i2c_mux_controller = i2c_mux_controller
        self.mux_channel = mux_channel
        self.sensor_i2c_address = sensor_i2c_address
        self.sensor_name = sensor_name
        self.is_initialized = False
        self.bus = self.i2c_mux_controller.get_smbus_instance()

        if self.i2c_mux_controller.is_initialized and self.bus:
            print(
                f"[{self.sensor_name}] Attempting to initialize on MUX Ch:{self.mux_channel} Addr:0x{self.sensor_i2c_address:X}")
            if self.i2c_mux_controller.select_channel(self.mux_channel):  # Select channel BEFORE init
                self.is_initialized = self._initialize_vl6180x_chip()
                if not self.is_initialized:
                    print(f"[{self.sensor_name}] CRITICAL: _initialize_vl6180x_chip FAILED.")
            else:
                print(
                    f"[{self.sensor_name}] CRITICAL: FAILED to select MUX channel {self.mux_channel} for initialization.")
        else:
            print(f"[{self.sensor_name}] CRITICAL: I2C MUX or bus not ready for init.")

    def _write_to_chip_16bit_reg_8bit_val(self, register_address_16bit, value_8bit):
        if not self.bus: return False
        reg_msb = (register_address_16bit >> 8) & 0xFF
        reg_lsb = register_address_16bit & 0xFF
        try:
            msg_w = smbus2.i2c_msg.write(self.sensor_i2c_address, [reg_msb, reg_lsb, value_8bit])
            self.bus.i2c_rdwr(msg_w)
            return True
        except OSError as e:
            print(
                f"[{self.sensor_name} I2C WRITE ERROR] MUX Ch:{self.mux_channel}, Addr:0x{self.sensor_i2c_address:X}, Reg:0x{register_address_16bit:04X}, Val:0x{value_8bit:02X}. Error: {e}")
            return False

    def _read_from_chip_16bit_reg_8bit_val(self, register_address_16bit):
        if not self.bus: return None
        reg_msb = (register_address_16bit >> 8) & 0xFF
        reg_lsb = register_address_16bit & 0xFF
        msg_write_addr = smbus2.i2c_msg.write(self.sensor_i2c_address, [reg_msb, reg_lsb])
        msg_read_val = smbus2.i2c_msg.read(self.sensor_i2c_address, 1)
        try:
            self.bus.i2c_rdwr(msg_write_addr, msg_read_val)
            data_list = list(msg_read_val)
            return data_list[0] if data_list else None
        except OSError as e:
            print(
                f"[{self.sensor_name} I2C READ ERROR] MUX Ch:{self.mux_channel}, Addr:0x{self.sensor_i2c_address:X}, Reg:0x{register_address_16bit:04X}. Error: {e}")
            return None

    def _initialize_vl6180x_chip(self):
        print(
            f"[{self.sensor_name} INIT_DBG] VL6180X _initialize_vl6180x_chip started (MUX Ch {self.mux_channel} should be active).")

        def log_init_op(op_type, reg, val=None, success=True, read_val=None, is_critical=True):
            status = "OK" if success else "FAIL"
            details = ""
            if op_type == "write":
                details = f"Write Reg:0x{reg:04X} Val:0x{val:02X}"
            elif op_type == "read":
                read_val_str = f"0x{read_val:02X}" if read_val is not None else "None (I2C Error)"
                details = f"Read Reg:0x{reg:04X} ... Value: {read_val_str}"
            print(f"[{self.sensor_name} INIT_DBG] {details} ... {status}")
            if not success and is_critical:
                print(f"[{self.sensor_name} INIT_FAIL] CRITICAL failure at {details}.")
            return success

        # Check FRESH_OUT_OF_RESET status
        val_0x016 = self._read_from_chip_16bit_reg_8bit_val(0x0016)
        if not log_init_op("read", 0x0016, read_val=val_0x016, success=(val_0x016 is not None)): return False

        # Mandatory ST Recommended Sequence (condensed, add checks for all if issues persist)
        # Only showing critical path checks for brevity here, ideally check all
        if not log_init_op("write", 0x0207, 0x01, self._write_to_chip_16bit_reg_8bit_val(0x0207, 0x01)): return False
        # ... (all other ST recommended sequence writes) ...
        # For example:
        # self._write_to_chip_16bit_reg_8bit_val(0x0208, 0x01)
        # ... many writes ...
        # self._write_to_chip_16bit_reg_8bit_val(0x0030, 0x00)
        # The full list of writes is in your previous code. Each should be wrapped in log_init_op.
        # Let's assume they are written here for now.
        # This is the ST recommended sequence for VL6180X (from AN4545 and common libraries)
        init_sequence = [
            (0x0207, 0x01), (0x0208, 0x01), (0x0096, 0x00), (0x0097, 0xfd), (0x00e3, 0x00),
            (0x00e4, 0x04), (0x00e5, 0x02), (0x00e6, 0x01), (0x00e7, 0x03), (0x00f5, 0x02),
            (0x00d9, 0x05), (0x00db, 0xce), (0x00dc, 0x03), (0x00dd, 0xf8), (0x009f, 0x00),
            (0x00a3, 0x3c), (0x00b7, 0x00), (0x00bb, 0x3c), (0x00b2, 0x09), (0x00ca, 0x09),
            (0x0198, 0x01), (0x01b0, 0x17), (0x01ad, 0x00), (0x00ff, 0x05), (0x0100, 0x05),
            (0x0199, 0x05), (0x01a6, 0x1b), (0x01ac, 0x3e), (0x01a7, 0x1f), (0x0030, 0x00)
        ]
        for reg, val in init_sequence:
            if not log_init_op("write", reg, val, self._write_to_chip_16bit_reg_8bit_val(reg, val)): return False

        # Public registers - Tuning parameters (from ST AN4545 / example code)
        if not log_init_op("write", 0x0011, 0x10,
                           self._write_to_chip_16bit_reg_8bit_val(0x0011, 0x10)): return False  # SYSTEM__MODE_GPIO1
        if not log_init_op("write", 0x010A, 0x30, self._write_to_chip_16bit_reg_8bit_val(0x010A,
                                                                                         0x30)): return False  # READOUT__AVERAGING_SAMPLE_PERIOD
        if not log_init_op("write", 0x003F, 0x46, self._write_to_chip_16bit_reg_8bit_val(0x003F,
                                                                                         0x46)): return False  # SYSLAS__INTEGRATION_PERIOD
        if not log_init_op("write", 0x0031, 0xFF,
                           self._write_to_chip_16bit_reg_8bit_val(0x0031, 0xFF)): return False  # SYSALS__ANALOGUE_GAIN
        if not log_init_op("write", 0x001B, 0x09, self._write_to_chip_16bit_reg_8bit_val(0x001B,
                                                                                         0x09)): return False  # SYSRANGE__INTERMEASUREMENT_PERIOD (100ms)
        if not log_init_op("write", 0x001C, 0x31, self._write_to_chip_16bit_reg_8bit_val(0x001C,
                                                                                         0x31)): return False  # SYSRANGE__MAX_CONVERGENCE_TIME (49ms)
        if not log_init_op("write", 0x0014, 0x04, self._write_to_chip_16bit_reg_8bit_val(0x0014,
                                                                                         0x04)): return False  # SYSTEM__INTERRUPT_CONFIG_GPIO

        # Clear FRESH_OUT_OF_RESET
        if not log_init_op("write", self.VL6180X_REG_SYSTEM_FRESH_OUT_OF_RESET, 0x00,
                           self._write_to_chip_16bit_reg_8bit_val(self.VL6180X_REG_SYSTEM_FRESH_OUT_OF_RESET,
                                                                  0x00)): return False

        print(f"[{self.sensor_name} INIT_DBG] VL6180X initialization sequence finished successfully.")
        return True

    def get_distance_mm(self):
        if not self.is_initialized: return -6
        # MUX channel selection is expected to be handled by the calling thread (sensor_and_frame_reader_thread)
        # immediately before calling this function in a sequential operation.
        # Thus, self._select_mux_channel() is omitted here.

        try:
            if not self._write_to_chip_16bit_reg_8bit_val(self.VL6180X_REG_SYSRANGE_START, 0x01):  # Trigger single shot
                return -8

            timeout_ms = 35  # Max time to wait for a measurement
            start_poll_time = time.monotonic()
            while True:
                status_val = self._read_from_chip_16bit_reg_8bit_val(self.VL6180X_REG_RESULT_INTERRUPT_STATUS_GPIO)
                if status_val is None: return -9  # I2C Error during poll

                if (status_val & 0x04) != 0: break  # Bit 2: Range new sample ready

                if (time.monotonic() - start_poll_time) * 1000 > timeout_ms:
                    return -7  # Polling timeout
                time.sleep(0.001)  # Poll every 1ms

            distance_mm = self._read_from_chip_16bit_reg_8bit_val(self.VL6180X_REG_RESULT_RANGE_VAL)
            if distance_mm is None: return -10

            if not self._write_to_chip_16bit_reg_8bit_val(self.VL6180X_REG_SYSTEM_INTERRUPT_CLEAR,
                                                          0x07):  # Clear all interrupt bits
                print(f"[{self.sensor_name} WARNING] Failed to clear interrupt status after read.")
            return int(distance_mm)
        except Exception as e:
            print(f"[{self.sensor_name} ERROR] Unhandled exception in get_distance_mm: {e}")
            return -1


# UARTModbusToFSensorController class ... (remains unchanged) ...
class UARTModbusToFSensorController:
    def __init__(self, port, baudrate=BAUD_RATE_TOF_CAMERA, sensor_name="CameraToF_UART"):
        self.port_name = port
        self.baudrate = baudrate
        self.sensor_name = sensor_name
        self.serial_port = None
        self.is_initialized = False
        self.port_is_open = False
        self.buffer = bytearray()
        self.slave_id = TOF050F_SLAVE_ID
        # ... (rest of init)
        print(f"[{self.sensor_name}] Attempting to open {self.port_name} at {self.baudrate} baud.")
        try:
            if not self.port_name: raise ValueError("Serial port name not provided.")
            self.serial_port = serial.Serial(self.port_name, self.baudrate, timeout=0.1)  # timeout for reads
            if self.serial_port.is_open:
                print(f"[{self.sensor_name}] Serial port {self.port_name} is open.")
                self.port_is_open = True
                self.is_initialized = True
            else:
                print(f"[{self.sensor_name} ERROR] Failed to open {self.port_name}.")
        except serial.SerialException as e:
            print(f"[{self.sensor_name} FATAL ERROR] SerialException: {e}")
        except ValueError as e:
            print(f"[{self.sensor_name} FATAL ERROR] Invalid serial config: {e}")
        except Exception as e:
            print(f"[{self.sensor_name} FATAL ERROR] Unknown error opening {self.port_name}: {e}")

    def _calculate_crc16(self, data: bytearray):  # Unchanged
        crc = 0xFFFF
        for byte_val in data:
            crc ^= byte_val
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return struct.pack('<H', crc)

    def _send_modbus_write_command(self, register_address, value):  # Unchanged from your debugged version
        if not self.port_is_open: return None
        pdu = bytearray([0x06]);
        pdu.extend(struct.pack('>H', register_address));
        pdu.extend(struct.pack('>H', value))
        frame_no_crc = bytearray([self.slave_id]);
        frame_no_crc.extend(pdu)
        crc_bytes = self._calculate_crc16(frame_no_crc)
        full_frame_to_send = bytearray(frame_no_crc);
        full_frame_to_send.extend(crc_bytes)
        try:
            self.serial_port.reset_input_buffer();
            self.serial_port.reset_output_buffer()
            self.serial_port.write(full_frame_to_send)
            time.sleep(0.05)  # Give sensor time to respond
            response = self.serial_port.read(8)
            # if response: print(f"[{self.sensor_name} DEBUG RX for Write] Received {len(response)} bytes: {response.hex()}")
            # else: print(f"[{self.sensor_name} DEBUG RX for Write] No response received (timeout).")
            return response if response and response == full_frame_to_send else None
        except Exception as e:
            print(f"[{self.sensor_name} ERROR] in _send_modbus_write_command: {e}"); return None

    def set_active_range_mode(self, mode_value):  # Unchanged
        mode_map = {1: "High (0.2m)", 2: "Mid (0.4m)", 3: "Long (0.5m)"}
        crc_map = {1: b'\x09\xCB', 2: b'\x48\x0B', 3: b'\xCA\xC9'}
        if mode_value not in mode_map: print(f"[{self.sensor_name} ERROR] Invalid mode: {mode_value}"); return False
        print(f"[{self.sensor_name}] Setting range mode to: {mode_map[mode_value]}")
        response = self._send_modbus_write_command(TOF050F_REG_RANGE_MODE, mode_value)
        pdu = bytearray([0x06, TOF050F_REG_RANGE_MODE >> 8, TOF050F_REG_RANGE_MODE & 0xFF, 0x00, mode_value])
        frame_no_crc = bytearray([self.slave_id]);
        frame_no_crc.extend(pdu)
        expected_echo = bytearray(frame_no_crc);
        expected_echo.extend(crc_map[mode_value])
        if response and response == bytes(expected_echo):
            print(f"[{self.sensor_name}] Successfully set range mode to {mode_map[mode_value]}.");
            self.current_range_mode_set = mode_value;
            time.sleep(0.1);
            return True
        else:
            rx_hex = response.hex(' ') if response else "None";
            expected_hex = expected_echo.hex(' ')
            print(
                f"[{self.sensor_name} ERROR] Failed to set range mode {mode_map[mode_value]}. Exp: {expected_hex}, RX: {rx_hex}");
            return False

    def parse_distance_from_stream(self):  # With SerialException handling
        if not self.port_is_open or not self.serial_port or not self.serial_port.is_open: return None
        try:
            if self.serial_port.in_waiting > 0:
                data_read = self.serial_port.read(self.serial_port.in_waiting)
                if data_read: self.buffer.extend(data_read)
        except serial.SerialException as se:
            print(f"[{self.sensor_name} SERIAL EXCEPTION in parse_distance]: {se}")
            self.port_is_open = False;
            self.is_initialized = False;
            return None
        except Exception as e:
            print(f"[{self.sensor_name} ERROR] Unexpected exception during serial read in parse: {e}")
            self.port_is_open = False;
            self.is_initialized = False;
            return None

        distance_mm = None
        while len(self.buffer) >= 7:
            if self.buffer[0] == self.slave_id and self.buffer[1] == 0x03 and self.buffer[2] == 0x02:
                packet_bytes, self.buffer = self.buffer[:7], self.buffer[7:]
                if packet_bytes[-2:] == self._calculate_crc16(packet_bytes[:-2]):
                    distance_mm = struct.unpack('>H', packet_bytes[3:5])[0];
                    break
                else:
                    self.buffer.insert(0, packet_bytes[1])  # Re-add rest of failed packet minus first byte for re-sync
            else:
                self.buffer.pop(0)
        return distance_mm

    def close(self):  # Unchanged
        if self.serial_port and self.serial_port.is_open:
            print(f"[{self.sensor_name}] Closing serial port {self.port_name}.");
            self.serial_port.close()
            self.port_is_open = False;
            self.is_initialized = False


# === Shared Variables and Locks (Unchanged) ===
latest_frame_data = None
latest_sensor_data_str = f"ENCODERS:{','.join(['-1'] * len(ENCODER_CHANNELS))}:TOF_HAND_I2C:-1:TOF_CAM_UART:-1\n"
frame_lock = threading.Lock()
latest_tof_hand_dist_mm = -1
latest_tof_camera_dist_mm = -1
tof_data_lock = threading.Lock()
keep_running = True

# === libcamera-vid & Arduino Connection (Unchanged) ===
# ... (code for cam_proc and arduino as before) ...
print("[INFO] Starting libcamera-vid...")
try:
    cam_proc = subprocess.Popen(
        ["libcamera-vid", "-t", "0", "--codec", "mjpeg", "-o", "-",
         "--width", str(FRAME_WIDTH), "--height", str(FRAME_HEIGHT),
         "--framerate", str(FRAME_RATE), "--nopreview",
         "--mode", "4608:2592:SRGGB10",  # Or your preferred camera mode
        "--autofocus-mode", "continuous",   # <--- ADD THIS FOR CONTINUOUS AF
        "--autofocus-range", "macro",       # <--- Optional: Often good for general use
        "--autofocus-speed", "fast",    # <--- Optional: Can make AF more responsive
    ],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE,
    )
except FileNotFoundError:
    print("[ERROR] libcamera-vid command not found."); cam_proc = None
except Exception as e:
    print(f"[ERROR] Failed to start libcamera-vid: {e}"); cam_proc = None

print(f"[INFO] Connecting to Arduino at {SERIAL_PORT_ARDUINO}...")
arduino = None
try:
    arduino = serial.Serial(SERIAL_PORT_ARDUINO, BAUD_RATE_ARDUINO, timeout=1)
    time.sleep(2);
    print("[INFO] Arduino connected.")
except serial.SerialException as e:
    print(f"[ERROR] Failed to open Arduino serial: {e}")
except Exception as e:
    print(f"[ERROR] Error connecting to Arduino: {e}")


# === REMOVED Dedicated Thread for I2C Hand ToF Sensor ===

# === Dedicated Thread for UART Camera ToF Sensor (Stays, with improved error handling) ===
def tof_camera_uart_reader_loop(tof_camera_controller):
    global latest_tof_camera_dist_mm, keep_running
    print("[INFO] UART Camera ToF reader thread started.")
    while keep_running:
        if tof_camera_controller and tof_camera_controller.is_initialized and tof_camera_controller.port_is_open:
            try:
                parsed_dist = tof_camera_controller.parse_distance_from_stream()
                if parsed_dist is not None:
                    with tof_data_lock:
                        latest_tof_camera_dist_mm = parsed_dist
            except serial.SerialException as se:  # Catch directly from call if not handled inside
                print(f"[{tof_camera_controller.sensor_name} THREAD SERIAL EXCEPTION]: {se}")
                tof_camera_controller.port_is_open = False  # Mark port as bad
                tof_camera_controller.is_initialized = False
                with tof_data_lock:
                    latest_tof_camera_dist_mm = -15
                time.sleep(1)  # Avoid rapid retries on a bad port
            except Exception as e:
                print(f"[{tof_camera_controller.sensor_name} THREAD UNEXPECTED EXCEPTION]: {e}")
                with tof_data_lock:
                    latest_tof_camera_dist_mm = -16
                time.sleep(1)
        else:
            if keep_running and not (
                    tof_camera_controller and tof_camera_controller.is_initialized and tof_camera_controller.port_is_open):
                # If it should be running but controller is not OK, update error and sleep
                with tof_data_lock: latest_tof_camera_dist_mm = -12
                time.sleep(1)  # Wait if controller is not ready

        if keep_running:  # Only sleep if we are supposed to continue
            time.sleep(TOF_READ_INTERVAL)

    if tof_camera_controller:
        tof_camera_controller.close()
    print("[INFO] UART Camera ToF reader thread stopped.")


# === Thread for Reading Camera Frames, Encoders, AND I2C HAND TOF ===
def sensor_and_frame_reader_thread(passed_shared_i2c_mux_controller,
                                   tof_hand_controller_obj,  # I2CHandToFSensorController instance
                                   tof_camera_controller_obj):  # UARTModbusToFSensorController (not used by this thread directly for I2C)
    global latest_frame_data, latest_sensor_data_str, keep_running, cam_proc
    global latest_tof_hand_dist_mm  # This thread updates it
    # global latest_tof_camera_dist_mm # Read from global, updated by its own thread

    print("[INFO] Sensor thread: Initializing Sensor Controllers using PASSED MUX...")
    i2c_master_mux = passed_shared_i2c_mux_controller

    encoder_ctrl = None
    if i2c_master_mux and i2c_master_mux.is_initialized:
        encoder_ctrl = MagneticEncoderController(i2c_master_mux, ENCODER_CHANNELS)
        print("[INFO] Sensor thread: Encoder controller initialized with shared MUX.")
    else:
        print("[ERROR] Sensor thread: Passed I2C MUX controller not valid. Encoders will fail.")

    # tof_hand_controller_obj is already initialized in __main__
    if not (tof_hand_controller_obj and tof_hand_controller_obj.is_initialized):
        print("[ERROR] Sensor thread: Hand ToF controller not valid or not initialized. Hand ToF will fail.")

    buffer = b""
    print("[INFO] Sensor and Frame reader thread started (Encoders & I2C HandToF via shared MUX; UART ToF dedicated).")
    last_sensor_read_time = time.monotonic()
    sensor_read_interval = 0.03  # Target ~30Hz

    while keep_running:
        try:
            # --- MJPEG frame reading logic ---
            chunk = None
            if cam_proc and cam_proc.stdout:
                chunk = cam_proc.stdout.read(16384)
            if not chunk:
                if cam_proc and cam_proc.poll() is not None:
                    print("[ERROR] Sensor thread: libcamera-vid process terminated.")
                    keep_running = False;
                    break
            else:
                buffer += chunk
                start_idx, end_idx = buffer.find(b'\xff\xd8'), buffer.find(b'\xff\xd9')
                if start_idx != -1 and end_idx > start_idx:
                    frame, buffer = buffer[start_idx: end_idx + 2], buffer[end_idx + 2:]
                    with frame_lock:
                        latest_frame_data = frame
                elif len(buffer) > 2 * FRAME_WIDTH * FRAME_HEIGHT * 3:  # Adjusted buffer trim logic
                    print(f"[WARNING] Sensor thread: MJPEG buffer too large ({len(buffer)}), trimming.")
                    trim_at = buffer.rfind(b'\xff\xd8', 0, len(buffer) // 2)  # Find last SOI in first half
                    buffer = buffer[trim_at if trim_at != -1 else len(buffer) // 2:]
            # --- End MJPEG frame reading ---

            current_time = time.monotonic()
            if current_time - last_sensor_read_time >= sensor_read_interval:
                encoder_readings = [-1] * len(ENCODER_CHANNELS)
                local_tof_hand_value = -1
                local_tof_camera_value = -1  # Renamed to avoid conflict

                # --- Read Encoders (Uses MUX) ---
                if encoder_ctrl:
                    encoder_readings = encoder_ctrl.read_all_encoders()

                # --- Read I2C Hand ToF Sensor (Uses MUX) ---
                if tof_hand_controller_obj and tof_hand_controller_obj.is_initialized:
                    # Explicitly select MUX channel for HandToF BEFORE calling get_distance_mm
                    if tof_hand_controller_obj.i2c_mux_controller.select_channel(tof_hand_controller_obj.mux_channel):
                        local_tof_hand_value = tof_hand_controller_obj.get_distance_mm()
                    else:
                        print(
                            f"[{tof_hand_controller_obj.sensor_name} SENSOR_THREAD_ERR] Failed to select MUX Ch:{tof_hand_controller_obj.mux_channel}.")
                        local_tof_hand_value = -2  # MUX select error
                else:
                    local_tof_hand_value = -11  # Controller not ready/init failed

                with tof_data_lock:  # Update the global variable
                    latest_tof_hand_dist_mm = local_tof_hand_value

                # --- Get UART Camera ToF data (from its dedicated thread via global var) ---
                with tof_data_lock:
                    local_tof_camera_value = latest_tof_camera_dist_mm  # Read from global

                new_sensor_data_payload = (
                    f"ENCODERS:{','.join(map(str, encoder_readings))}"
                    f":TOF_HAND_I2C:{int(latest_tof_hand_dist_mm)}"
                    f":TOF_CAM_UART:{int(local_tof_camera_value)}\n"  # Use local_tof_camera_value
                )
                with frame_lock:
                    latest_sensor_data_str = new_sensor_data_payload
                last_sensor_read_time = current_time

            # Brief sleep if no camera data and not time for sensors, to yield CPU
            if not chunk and (time.monotonic() - last_sensor_read_time < sensor_read_interval):
                time.sleep(0.0005)  # 0.5ms

        except BrokenPipeError:  # ... (error handling as before) ...
            print("[ERROR] Sensor thread: Broken pipe (camera process likely died).")
            keep_running = False;
            break
        except Exception as e:  # ... (error handling as before) ...
            print(f"[ERROR] Sensor thread: Unhandled error: {type(e).__name__} - {e}")
            import traceback;
            traceback.print_exc();
            time.sleep(0.5)

    print("[INFO] Sensor and Frame reader thread (encoders, I2C ToF) gracefully stopped.")
    if cam_proc and cam_proc.poll() is None:  # ... (cleanup as before) ...
        print("[INFO] Sensor thread: Terminating camera process.")
        cam_proc.terminate()
        try:
            cam_proc.wait(timeout=0.5)
        except subprocess.TimeoutExpired:
            if cam_proc: cam_proc.kill()


# === Client Handling & Servers (Largely Unchanged) ===
# ... (handle_client, camera_stream_server, command_server functions as before) ...
def handle_client(client_socket, addr):  # Unchanged
    global latest_frame_data, latest_sensor_data_str, frame_lock, keep_running
    send_interval = 1.0 / FRAME_RATE;
    last_sent_frame_id = id(None);
    last_sensor_send_time = 0
    try:
        with client_socket:
            while keep_running:
                start_time = time.monotonic()
                current_frame_to_send, current_sensor_data_to_send = None, ""
                with frame_lock:
                    if latest_frame_data and (
                            id(latest_frame_data) != last_sent_frame_id or (start_time - last_sensor_send_time) > 0.1):
                        current_frame_to_send, current_sensor_data_to_send = latest_frame_data, latest_sensor_data_str
                        last_sent_frame_id, last_sensor_send_time = id(latest_frame_data), start_time
                if current_frame_to_send:
                    try:
                        sensor_bytes = current_sensor_data_to_send.encode('utf-8')
                        packet = sensor_bytes + current_frame_to_send
                        client_socket.sendall(struct.pack('>I', len(packet)));
                        client_socket.sendall(packet)
                    except (socket.error, BrokenPipeError, ConnectionResetError):
                        break
                    except Exception as e:
                        print(f"[CAMERA_SERVER] Error sending to {addr}: {e}"); break
                elapsed_time = time.monotonic() - start_time
                if (sleep_duration := send_interval - elapsed_time) > 0: time.sleep(sleep_duration)
    except Exception as e:
        print(f"[CAMERA_SERVER] Unhandled exception for {addr}: {e}")
    finally:
        client_socket.close()


def camera_stream_server():  # Unchanged
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM);
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        server_socket.bind(('0.0.0.0', CAMERA_PORT)); server_socket.listen(5); print(
            f"[CAMERA_SERVER] Listening on port {CAMERA_PORT}")
    except Exception as e:
        print(f"[ERROR] Bind camera server socket: {e}"); return
    while keep_running:
        try:
            server_socket.settimeout(1.0);
            client_socket, addr = server_socket.accept();
            server_socket.settimeout(None)
            threading.Thread(target=handle_client, args=(client_socket, addr), daemon=True,
                             name=f"Client_{addr[0]}_{addr[1]}").start()
        except socket.timeout:
            if not keep_running: break
        except Exception as e:
            if keep_running: print(f"[ERROR] Camera server accept: {e}"); break
    print("[INFO] Camera stream server stopped.");
    server_socket.close()


def command_server():  # Unchanged
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM);
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        server_socket.bind(('0.0.0.0', COMMAND_PORT)); server_socket.listen(1); print(
            f"[COMMAND_SERVER] Listening on port {COMMAND_PORT}")
    except Exception as e:
        print(f"[ERROR] Bind command server socket: {e}"); return
    while keep_running:
        client_socket_cmd = None
        try:
            server_socket.settimeout(1.0);
            client_socket_cmd, addr = server_socket.accept();
            server_socket.settimeout(None)
            with client_socket_cmd:
                while keep_running:
                    try:
                        client_socket_cmd.settimeout(1.0);
                        data = client_socket_cmd.recv(1024);
                        client_socket_cmd.settimeout(None)
                        if not data: break
                        command = data.decode('utf-8', errors='ignore').strip().lstrip('\ufeff')
                        if not command: continue
                        if arduino and arduino.is_open:
                            try:
                                arduino.write((command + '\n').encode('utf-8'))
                            except serial.SerialException as se:
                                print(f"[COMMAND_SERVER] Serial write error: {se}")
                        elif not arduino:
                            print("[COMMAND_SERVER] Arduino not connected.")
                    except socket.timeout:
                        if not keep_running: break
                    except (socket.error, ConnectionResetError):
                        break
                    except Exception as e:
                        print(f"[COMMAND_SERVER] Error handling command client {addr}: {e}"); break
        except socket.timeout:
            if not keep_running: break
        except Exception as e:
            if keep_running: print(f"[COMMAND_SERVER] Server accept: {e}"); break
        finally:
            if client_socket_cmd: client_socket_cmd.close()
    print("[INFO] Command server stopped.");
    server_socket.close()


# === Start Threads & Main Loop ===
if __name__ == '__main__':
    active_threads = []
    print("[INFO] Main: Creating centralized I2C Multiplexer Controller...")
    shared_i2c_mux_controller = I2CMultiplexerController(bus_id=I2C_BUS_ID)

    if not shared_i2c_mux_controller.is_initialized:
        print("[FATAL ERROR] Main: Centralized I2C MUX Controller failed. Some sensors will not work.")

    tof_hand_i2c_ctrl_main = I2CHandToFSensorController(
        i2c_mux_controller=shared_i2c_mux_controller,
        mux_channel=TOF_HAND_MUX_CHANNEL,
        sensor_i2c_address=TOF_HAND_I2C_ADDRESS,
        sensor_name="HandToF_I2C_Global"
    )
    if not tof_hand_i2c_ctrl_main.is_initialized:
        print(
            f"[ERROR] Main: {tof_hand_i2c_ctrl_main.sensor_name} FAILED TO INITIALIZE. It will not be read correctly.")
        with tof_data_lock: latest_tof_hand_dist_mm = -13  # Init failure

    tof_camera_uart_ctrl_main = UARTModbusToFSensorController(
        port=SERIAL_PORT_TOF_CAMERA,
        baudrate=BAUD_RATE_TOF_CAMERA,
        sensor_name="CameraToF_UART_Global"
    )
    if tof_camera_uart_ctrl_main.is_initialized:
        print("[INFO] Main: Setting UART ToF default range mode...")
        tof_camera_uart_ctrl_main.set_active_range_mode(2)  # e.g., Middle Distance

    if cam_proc:
        # --- I2C Hand ToF thread REMOVED ---
        print("[INFO] Main: I2C Hand ToF will be read in main sensor thread (SensorFrameReader).")

        if tof_camera_uart_ctrl_main.is_initialized:
            tof_camera_thread = threading.Thread(target=tof_camera_uart_reader_loop, args=(tof_camera_uart_ctrl_main,),
                                                 name="ToFCameraReader", daemon=True)
            active_threads.append(tof_camera_thread)
            tof_camera_thread.start()
        else:
            print("[ERROR] Main: Camera ToF UART Controller not initialized. Reader thread not started.")
            with tof_data_lock:
                latest_tof_camera_dist_mm = -14

        sensor_thread = threading.Thread(target=sensor_and_frame_reader_thread,
                                         args=(shared_i2c_mux_controller,
                                               tof_hand_i2c_ctrl_main,
                                               tof_camera_uart_ctrl_main),
                                         # Pass UART ToF obj, though not used for I2C by this thread
                                         name="SensorFrameReader", daemon=True)
        active_threads.append(sensor_thread)
        sensor_thread.start()

        cam_server_thread = threading.Thread(target=camera_stream_server, name="CameraStreamServer", daemon=True)
        active_threads.append(cam_server_thread)
        cam_server_thread.start()

        if arduino and arduino.is_open:
            cmd_server_thread = threading.Thread(target=command_server, name="CommandServer", daemon=True)
            active_threads.append(cmd_server_thread)
            cmd_server_thread.start()
        else:
            print("[INFO] Main: Arduino not connected/valid, command server not started.")
    else:
        print("[ERROR] Main: Camera process failed to start. Servers not launched.")

    try:
        while keep_running:
            # Monitor threads
            current_active_threads = [t for t in active_threads if t.is_alive()]
            if len(current_active_threads) < len(active_threads):
                for t in active_threads:
                    if not t.is_alive() and t in active_threads:  # Check if it was an original active thread
                        print(f"[ERROR] Main: Thread '{t.name}' is no longer alive. Initiating shutdown.")
                        keep_running = False;
                        break
                active_threads = current_active_threads  # Update list

            if cam_proc and cam_proc.poll() is not None and keep_running:
                print("[ERROR] Main: libcamera-vid process terminated unexpectedly. Shutting down.")
                keep_running = False

            if not keep_running: break
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n[INFO] Main: Shutdown requested by user (Ctrl+C)...")
    finally:
        print("[INFO] Main: Initiating shutdown sequence...")
        keep_running = False

        if cam_proc and cam_proc.poll() is None:
            print("[INFO] Main: Terminating camera process...")
            cam_proc.terminate()
            try:
                cam_proc.wait(timeout=1)
            except subprocess.TimeoutExpired:
                if cam_proc: print(
                    "[WARNING] Main: Camera process did not terminate gracefully, killing."); cam_proc.kill()

        print(f"[INFO] Main: Waiting for {len(active_threads)} threads to stop...")
        for t in active_threads:  # Join all threads that were started
            if t.is_alive():
                try:
                    t.join(timeout=1.5)  # Slightly shorter join timeout for individual threads
                    if t.is_alive(): print(f"[WARNING] Main: Thread '{t.name}' did not stop in time.")
                except Exception as e:
                    print(f"[ERROR] Main: Error joining thread {t.name}: {e}")

        if arduino and arduino.is_open:
            print("[INFO] Main: Closing Arduino connection...");
            arduino.close()

        print("[INFO] Main: Shutdown complete.")