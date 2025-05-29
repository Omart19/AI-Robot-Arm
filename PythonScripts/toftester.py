import smbus2
import time

# I2C Configuration
I2C_BUS_ID = 1
TCA_ADDRESS = 0x70  # I2C address of the TCA9548A multiplexer
TOF_SENSOR_I2C_ADDRESS = 0x29
CHANNEL_TO_TEST = 4 # Or 5, for your other sensor

# VL6180X Identification Register
VL6180X_REG_IDENTIFICATION_MODEL_ID = 0x000
EXPECTED_MODEL_ID = 0xB4 # Standard Model ID for VL6180X

bus = None
try:
    bus = smbus2.SMBus(I2C_BUS_ID)
    print(f"Successfully opened I2C bus {I2C_BUS_ID}.")

    # Select the channel on the multiplexer
    print(f"Selecting channel {CHANNEL_TO_TEST} on multiplexer (0x{TCA_ADDRESS:X})...")
    bus.write_byte(TCA_ADDRESS, 1 << CHANNEL_TO_TEST)
    time.sleep(0.1) # Give a moment for the channel to switch

    print(f"Attempting to read Model ID from sensor at 0x{TOF_SENSOR_I2C_ADDRESS:X} on channel {CHANNEL_TO_TEST} from register 0x{VL6180X_REG_IDENTIFICATION_MODEL_ID:03X}...")
    
    model_id = bus.read_byte_data(TOF_SENSOR_I2C_ADDRESS, VL6180X_REG_IDENTIFICATION_MODEL_ID)
    print(f"Read Model ID: {model_id:#04x}")

    if model_id == EXPECTED_MODEL_ID:
        print("SUCCESS: Correct Model ID (0xB4) read from VL6180X.")
        print("This confirms basic I2C communication to the sensor on this channel is likely working.")
    else:
        print(f"WARNING: Model ID read ({model_id:#04x}) does not match expected VL6180X ID ({EXPECTED_MODEL_ID:#04x}).")
        print("This could indicate a problem with the sensor, wiring, or it might not be a VL6180X / not in direct I2C mode.")

except FileNotFoundError:
    print(f"ERROR: I2C bus {I2C_BUS_ID} not found. Is I2C enabled? Is smbus2 installed?")
except OSError as e:
    print(f"I2C ERROR: {e}")
    print("This could be due to incorrect wiring, wrong I2C address, sensor not powered, sensor not responding, or multiplexer issue.")
except Exception as e:
    print(f"An unexpected error occurred: {e}")
finally:
    if bus:
        # Deselect channel (optional, but good practice)
        # bus.write_byte(TCA_ADDRESS, 0) 
        bus.close()
        print("I2C bus closed.")