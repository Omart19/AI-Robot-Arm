import smbus2 # For I2C communication
import time

# --- Configuration ---
I2C_BUS_NUMBER = 1  # Raspberry Pi I2C bus (usually 1 for GPIO I2C)
MULTIPLEXER_ADDRESS = 0x70 # Address of your I2C multiplexer
TOF_EXPECTED_I2C_ADDRESS = 0x29 # Default I2C address of the VL6180X sensor core

# VL6180X Register for checking device identity
VL6180X_REG_IDENTIFICATION_MODEL_ID = 0x0000
VL6180X_EXPECTED_MODEL_ID = 0xB4 # Expected value for VL6180X

bus = None

def select_multiplexer_channel(bus, channel):
    """
    Selects a specific channel on the I2C multiplexer (e.g., TCA9548A).
    For TCA9548A, channel 0 is bit 0 (value 0x01), channel 1 is bit 1 (value 0x02), etc.
    Writing 0x00 deselects all channels.
    """
    if not (0 <= channel <= 7): # Assuming an 8-channel multiplexer
        print(f"Error: Channel {channel} is out of range (0-7).")
        return False
    try:
        control_byte = 1 << channel
        bus.write_byte(MULTIPLEXER_ADDRESS, control_byte)
        # print(f"Multiplexer: Switched to channel {channel} (control byte 0x{control_byte:02X}).")
        time.sleep(0.05)  # Short delay to allow the multiplexer to switch
        return True
    except OSError as e:
        print(f"Error: Could not write to multiplexer at 0x{MULTIPLEXER_ADDRESS:02X} to select channel {channel}. {e}")
        return False

def check_for_tof_sensor(bus, address):
    """
    Checks for the VL6180X sensor by trying to read its model ID.
    """
    try:
        # VL6180X registers are 16-bit. Write MSB then LSB of register address.
        reg_msb = (VL6180X_REG_IDENTIFICATION_MODEL_ID >> 8) & 0xFF
        reg_lsb = VL6180X_REG_IDENTIFICATION_MODEL_ID & 0xFF

        # Create a write transaction [reg_msb, reg_lsb] to set the register pointer
        msg_write = smbus2.i2c_msg.write(address, [reg_msb, reg_lsb])
        # Create a read transaction to read 1 byte (the model ID)
        msg_read = smbus2.i2c_msg.read(address, 1)

        # Perform the combined write-then-read transaction
        bus.i2c_rdwr(msg_write, msg_read)
        
        model_id = list(msg_read)[0]

        if model_id == VL6180X_EXPECTED_MODEL_ID:
            print(f"   SUCCESS: Found VL6180X (TOF sensor) at 0x{address:02X}. Model ID: 0x{model_id:02X}.")
            return True
        else:
            print(f"   INFO: Device found at 0x{address:02X}, but Model ID is 0x{model_id:02X} (expected 0x{VL6180X_EXPECTED_MODEL_ID:02X}). May not be the TOF sensor.")
            return False
    except OSError:
        # This typically means no device ACKed at this address on the current channel
        return False
    except Exception as e:
        print(f"   Error checking for TOF sensor at 0x{address:02X}: {e}")
        return False

def main():
    global bus
    print("Attempting to find TOF050F sensor (VL6180X core) through I2C Multiplexer...")
    print(f"Multiplexer Address: 0x{MULTIPLEXER_ADDRESS:02X}")
    print(f"Expected TOF Sensor I2C Address: 0x{TOF_EXPECTED_I2C_ADDRESS:02X}")
    print("---")
    print("IMPORTANT: Ensure your TOF050F sensor has been switched to I2C mode first via serial command!")
    print("---")

    try:
        bus = smbus2.SMBus(I2C_BUS_NUMBER)
    except FileNotFoundError:
        print(f"Error: I2C bus {I2C_BUS_NUMBER} not found. Is I2C enabled on your Raspberry Pi?")
        return
    except Exception as e:
        print(f"Error initializing I2C bus: {e}")
        return

    found_sensor_channel = -1

    for channel in range(8): # For an 8-channel multiplexer like TCA9548A
        print(f"\nSelecting multiplexer channel {channel}...")
        if not select_multiplexer_channel(bus, channel):
            print("   Skipping scan on this channel due to selection error.")
            continue
        
        print(f"   Scanning for TOF sensor at 0x{TOF_EXPECTED_I2C_ADDRESS:02X} on channel {channel}...")
        if check_for_tof_sensor(bus, TOF_EXPECTED_I2C_ADDRESS):
            found_sensor_channel = channel
            break # Sensor found, no need to scan other channels

    if found_sensor_channel != -1:
        print(f"\n--- Sensor Discovery Complete ---")
        print(f"TOF050F sensor (VL6180X core) was FOUND on multiplexer channel {found_sensor_channel} at I2C address 0x{TOF_EXPECTED_I2C_ADDRESS:02X}.")
        print("You can now communicate with it by ensuring this channel remains selected.")
    else:
        print(f"\n--- Sensor Discovery Complete ---")
        print("TOF050F sensor was NOT found on any channel of the multiplexer.")
        print("Troubleshooting suggestions:")
        print("  1. Verify the TOF050F is correctly switched to I2C mode.")
        print("  2. Check all wiring: sensor to multiplexer, and multiplexer to Raspberry Pi.")
        print("  3. Ensure the multiplexer itself is working (it should appear at 0x{MULTIPLEXER_ADDRESS:02X} with `i2cdetect -y {I2C_BUS_NUMBER}`).")
        print("  4. Confirm the TOF sensor's I2C address is indeed 0x{TOF_EXPECTED_I2C_ADDRESS:02X} when in I2C mode.")
        print("  5. Make sure the sensor is powered correctly.")

    # Optional: Deselect all channels on the multiplexer when done
    # try:
    #     bus.write_byte(MULTIPLEXER_ADDRESS, 0x00)
    #     print("\nMultiplexer: All channels deselected.")
    # except OSError:
    #     pass # Ignore if it fails, might have been an issue earlier

    if bus:
        bus.close()

if __name__ == "__main__":
    main()