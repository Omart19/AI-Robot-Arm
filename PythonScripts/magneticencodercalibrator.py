import smbus2
import time

class AS5600_Calibrator:
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
        self.REG_RAW_ANGLE_H = 0x0C
        self.REG_RAW_ANGLE_L = 0x0D
        self.REG_ZPOS_H = 0x01
        self.REG_ZPOS_L = 0x02
        self.REG_BURN = 0xFF
        self.REG_STATUS = 0x0B
        self.BURN_ANGLE_CMD = 0x80

    def _select_channel(self, channel_num):
        try:
            self.bus.write_byte(self.tca_address, 1 << channel_num)
            time.sleep(0.01) # Brief delay for channel switch
            return True
        except OSError as e:
            print(f"Error selecting channel {channel_num} on multiplexer: {e}")
            return False

    def calibrate_sensor_zero_position(self, channel_num, sensor_i2c_address, confirm_burn=True):
        print(f"\n--- Calibrating Sensor on Channel {channel_num} (Address: {hex(sensor_i2c_address)}) ---")

        if not self._select_channel(channel_num):
            return False

        # 1. Verify Magnet Detection
        try:
            status_val = self.bus.read_byte_data(sensor_i2c_address, self.REG_STATUS)
            if not (status_val & 0b00100000): # MD bit (bit 5)
                print("Magnet not detected. Skipping calibration for this sensor.")
                return False
            print("Magnet detected.")
        except OSError as e:
            print(f"Error reading STATUS: {e}")
            return False

        # 2. Read RAW ANGLE
        try:
            raw_angle_h = self.bus.read_byte_data(sensor_i2c_address, self.REG_RAW_ANGLE_H)
            raw_angle_l = self.bus.read_byte_data(sensor_i2c_address, self.REG_RAW_ANGLE_L)
            current_raw_angle = (raw_angle_h << 8) | raw_angle_l
            print(f"Current Raw Angle: {current_raw_angle}")
        except OSError as e:
            print(f"Error reading RAW ANGLE: {e}")
            return False

        # 3. Set ZPOS
        zpos_h_val = (current_raw_angle >> 8) & 0xFF
        zpos_l_val = current_raw_angle & 0xFF
        try:
            self.bus.write_byte_data(sensor_i2c_address, self.REG_ZPOS_H, zpos_h_val)
            self.bus.write_byte_data(sensor_i2c_address, self.REG_ZPOS_L, zpos_l_val)
            time.sleep(0.002) # Wait 2ms
            print(f"ZPOS register set to: {current_raw_angle}")
        except OSError as e:
            print(f"Error writing ZPOS: {e}")
            return False

        # 4. Burn Angle
        if confirm_burn:
            user_input = input(f"PERMANENT BURN: Type 'YES' to burn ZPOS={current_raw_angle} for sensor on channel {channel_num}: ").strip()
            if user_input != "YES":
                print("Burn operation cancelled by user.")
                return False
        
        try:
            self.bus.write_byte_data(sensor_i2c_address, self.REG_BURN, self.BURN_ANGLE_CMD)
            time.sleep(0.002) # Wait 2ms
            print("Burn_Angle command sent successfully.")
            # Add verification logic here if desired
            return True
        except OSError as e:
            print(f"Error sending BURN_ANGLE command: {e}")
            return False

    def calibrate_all_sensors(self):
        for channel, address in self.sensor_addresses.items():
            # Ensure magnets are in their final fixed positions before running this.
            # You might want to add a delay or prompt between sensors.
            self.calibrate_sensor_zero_position(channel, address, confirm_burn=True)
            print("-" * 30)
            time.sleep(1) # Pause for a second before doing the next one

# Example Usage:
if __name__ == "__main__":
    calibrator = AS5600_Calibrator()
    # Ensure your RPi I2C bus is correctly configured and devices are connected.
    # The magnets should be in their PERMANENT FIXED POSITIONS for this calibration.
    calibrator.calibrate_all_sensors()