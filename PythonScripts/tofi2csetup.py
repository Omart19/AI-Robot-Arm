import serial
import struct
import time

# --- CRC-16/MODBUS Calculation ---
def crc16_modbus(data: bytes) -> bytes:
    crc = 0xFFFF
    poly = 0xA001
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ poly
            else:
                crc = crc >> 1
    return struct.pack('<H', crc)

# --- Sensor Configuration ---
SERIAL_PORT = '/dev/ttyS0'  # Or your actual serial port
BAUD_RATE = 115200
SLAVE_ID = 0x01

# --- Modbus Command to Switch to I2C Mode ---
# Register 0x0009, Value 0x0001
REGISTER_TO_WRITE = 0x0009
VALUE_TO_WRITE = 0x0001

# Frame before CRC:
# Slave ID, Func Code (0x06), Register Addr_Hi, Register Addr_Lo, Value_Hi, Value_Lo
command_frame_no_crc = struct.pack(
    '>BBHH',  # Format: byte, byte, unsigned short (reg addr), unsigned short (value)
    SLAVE_ID,
    0x06, # Function code for Write Single Register
    REGISTER_TO_WRITE,
    VALUE_TO_WRITE
)

crc_bytes = crc16_modbus(command_frame_no_crc)
switch_to_i2c_command = command_frame_no_crc + crc_bytes

print(f"Command to send (hex): {switch_to_i2c_command.hex().upper()}")
# Expected: 0106000900019808

try:
    ser = serial.Serial(
        port=SERIAL_PORT,
        baudrate=BAUD_RATE,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1
    )
    print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")

    ser.write(switch_to_i2c_command)
    print(f"Sent command to switch to I2C mode: {switch_to_i2c_command.hex().upper()}")

    # The sensor should respond acknowledging the write command.
    # The response to a successful write single register command is an echo of the request.
    time.sleep(0.1) # Give sensor time to respond and process
    if ser.in_waiting > 0:
        response = ser.read(ser.in_waiting)
        print(f"Received response (hex): {response.hex().upper()}")
        if response == switch_to_i2c_command:
            print("Successfully switched to I2C mode (sensor acknowledged).")
            print("You will now need to disconnect from serial and connect via I2C pins (SDA/SCL).")
        else:
            print("Unexpected response from sensor.")
    else:
        print("No response from sensor. It might have switched mode and stopped serial communication.")
        print("Try connecting via I2C pins (SDA/SCL).")

except serial.SerialException as e:
    print(f"Serial error: {e}")
except KeyboardInterrupt:
    print("Exiting...")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("Serial port closed.")