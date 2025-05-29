using System;
using System.Collections.Generic;
using System.Device.I2c;
using Avalonia.Controls;
using Dock.Model.Avalonia.Core;
using Iot.Device.Tca954x;

namespace RobotAIArm.Controllers
{
    public class MagneticEncoderController : DockWindow
    {
        public MagneticEncoderController()
        {
            Console.WriteLine("[MagneticEncoderController] Constructor called. Initializing states..."); // DEBUG
            for (int i = 0; i < 4; i++)
            {
                _encoderStates[i] = new EncoderState();
            }
            Console.WriteLine("[MagneticEncoderController] Encoder states initialized."); // DEBUG
                                                                                          // Consider initializing _i2cBus here if you adopt that change later
        }
        public class EncoderState
        {
            public int PreviousRawAngle { get; set; } = -1; // Initialize to an invalid state
            public int RotationCount { get; set; } = 0;
            public int CalibratedRawAt90Degrees { get; set; } = -1; // Raw value that means 90 degrees
            public bool IsCalibrated { get; set; } = false;
        }
        private Dictionary<int, EncoderState> _encoderStates = new Dictionary<int, EncoderState>();
        private const int MAX_RAW_VALUE = 4095; // 12-bit resolution (0-4095)
        private const int HALF_RAW_VALUE = MAX_RAW_VALUE / 2; // For rollover detection (approx 2048)

        public List<double> ReadDataFromSensors()
        {
            Console.WriteLine("[MagneticEncoderController] ReadDataFromSensors() CALLED."); // DEBUG
            int busId = 1;
            int tcaAddress = 0x70;
            var sensorAddresses = new Dictionary<int, int> { { 0, 0x36 }, { 1, 0x36 }, { 2, 0x36 }, { 3, 0x36 } };
            var processedAngles = new List<double>();
            I2cBus i2cBus = null; // Initialize to null

            try // Add a try-catch around I2cBus.Create
            {
                i2cBus = I2cBus.Create(busId);
                Console.WriteLine("[MagneticEncoderController] I2cBus.Create successful."); // DEBUG
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[MagneticEncoderController] CRITICAL ERROR creating I2cBus: {ex.Message}"); // DEBUG
                                                                                                                // Return empty or list of NaNs if bus creation fails, to avoid further errors
                for (int i = 0; i < sensorAddresses.Count; i++) processedAngles.Add(double.NaN);
                return processedAngles;
            }

            using (i2cBus) // Ensure it's disposed
            {
                foreach (var entry in sensorAddresses)
                {
                    int channel = entry.Key;
                    int sensorI2cAddress = entry.Value;
                    try
                    {
                        Console.WriteLine($"[MagneticEncoderController] Processing channel {channel}."); // DEBUG
                        SelectTcaChannel(i2cBus, tcaAddress, channel);
                        int rawAngle = ReadSensorValue(i2cBus, sensorI2cAddress); // This already has a debug line

                        if (_encoderStates.TryGetValue(channel, out EncoderState? state) && state.IsCalibrated) // Added null check for state
                        {
                            Console.WriteLine($"[MagneticEncoderController] Channel {channel} is calibrated. Getting processed angle."); // DEBUG
                            processedAngles.Add(GetProcessedAngleInDegrees(channel, rawAngle));
                        }
                        else
                        {
                            Console.WriteLine($"[MagneticEncoderController] Channel {channel} NOT calibrated or state not found. Adding raw conversion."); // DEBUG
                            double rawDegrees = (rawAngle / (double)MAX_RAW_VALUE) * 360.0;
                            processedAngles.Add(rawDegrees);
                        }
                    }
                    catch (Exception ex)
                    {
                        Console.WriteLine($"[MagneticEncoderController] Error processing channel {channel}: {ex.Message}"); // DEBUG
                        processedAngles.Add(double.NaN);
                    }
                }
            }
            Console.WriteLine($"[MagneticEncoderController] ReadDataFromSensors() RETURNING: {string.Join(", ", processedAngles)}"); // DEBUG
            return processedAngles;
        }
        public double GetProcessedAngleInDegrees(int channel, int newRawAngle)
        {
            if (!_encoderStates.TryGetValue(channel, out EncoderState state))
            {
                // Should not happen if initialized correctly
                Console.WriteLine($"Error: No state found for channel {channel} during processing.");
                return double.NaN; // Or throw an exception
            }

            if (!state.IsCalibrated)
            {
                // Optionally, return raw degrees if not calibrated, or NaN
                // For now, let's return raw conversion until calibrated for the 90-degree point.
                // Or, you could have a default behavior.
                // The user specifically wants the 90-degree point, so calibration is key.
                Console.WriteLine($"Warning: Channel {channel} is not calibrated. Angle may not be relative to 90-degree start.");
                // If you still want to show unwrapped angle before calibration:
                // Fall through to unwrapping logic but skip the 90-degree offset part.
                // However, the prompt implies calibration is the first step for the desired 90-degree point.
                // For simplicity of this example, let's assume it should be calibrated.
                // If not, the offset calculation might be based on an uninitialized CalibratedRawAt90Degrees.
                // A robust way is to return NaN or a specific status if not calibrated.
                return (newRawAngle / (double)MAX_RAW_VALUE) * 360.0; // Raw conversion if not calibrated
            }

            // Initialize PreviousRawAngle on first read after calibration or if it was reset
            if (state.PreviousRawAngle == -1)
            {
                state.PreviousRawAngle = newRawAngle;
            }

            // --- Rollover Detection (Unwrapping) ---
            int deltaRaw = newRawAngle - state.PreviousRawAngle;

            if (deltaRaw > HALF_RAW_VALUE) // e.g., 350 -> 10 degrees (0xFFF -> 0x00A), newRawAngle is small, prev is large
            {
                state.RotationCount--; // Crossed from high to low (e.g. 4090 to 10 is a forward roll)
            }
            else if (deltaRaw < -HALF_RAW_VALUE) // e.g., 10 -> 350 degrees (0x00A -> 0xFFF), newRawAngle is large, prev is small
            {
                state.RotationCount++; // Crossed from low to high (e.g. 10 to 4090 is a backward roll)
            }
            state.PreviousRawAngle = newRawAngle;

            // --- Calculate Continuous Angle ---
            // This is the "absolute" raw value accumulated across rotations
            long continuousRawValue = (long)newRawAngle + ((long)state.RotationCount * (MAX_RAW_VALUE + 1));

            // --- Convert to Degrees and Apply 90-Degree Offset ---
            // Convert the calibrated 90-degree point to its equivalent "zero-rotation" degree value
            double calibratedOffsetDegrees = (state.CalibratedRawAt90Degrees / (double)MAX_RAW_VALUE) * 360.0;

            // Convert the current continuous raw value to degrees
            double currentContinuousDegrees = (continuousRawValue / (double)MAX_RAW_VALUE) * 360.0;

            // The final angle is the current continuous angle, shifted so that
            // the calibrated point becomes 90 degrees.
            double finalAngle = currentContinuousDegrees - calibratedOffsetDegrees + 90.0;

            return finalAngle;
        }

        public void CalibrateSensorTo90Degrees(int channel, int currentRawAngle)
        {
            if (_encoderStates.TryGetValue(channel, out EncoderState state))
            {
                state.CalibratedRawAt90Degrees = currentRawAngle;
                state.RotationCount = 0; // Reset rotation count on calibration
                state.PreviousRawAngle = currentRawAngle; // Set previous angle to current for future unwrapping
                state.IsCalibrated = true;
                Console.WriteLine($"Channel {channel} calibrated: Raw value {currentRawAngle} is now the 90-degree point.");
            }
            else
            {
                Console.WriteLine($"Error: No state found for channel {channel}.");
            }
        }

        private static void SelectTcaChannel(I2cBus i2cBus, int tcaAddress, int channel)
        {
            Console.WriteLine($"[MagneticEncoderController] SelectTcaChannel for channel {channel}."); // DEBUG
            using (var tcaDevice = i2cBus.CreateDevice(tcaAddress))
            {
                byte channelMask = (byte)(1 << channel);
                tcaDevice.WriteByte(channelMask);
                System.Threading.Thread.Sleep(10); // Keep this small delay
            }
        }

        private static int ReadSensorValue(I2cBus i2cBus, int sensorAddress)
        {
            try
            {
                using (var sensorDevice = i2cBus.CreateDevice(sensorAddress))
                {
                    // Read MSB and LSB of raw angle
                    Span<byte> buffer = stackalloc byte[2];
                    sensorDevice.WriteByte(0x0C); // Start at register 0x0C
                    sensorDevice.Read(buffer);

                    //Console.WriteLine($"Read data: MSB=0x{buffer[0]:X2}, LSB=0x{buffer[1]:X2}");

                    // Combine bytes into 12-bit angle value
                    int rawAngle = ((buffer[0] << 8) | buffer[1]) & 0x0FFF;
                    Console.WriteLine($"ReadSensorValue - Raw Angle: {rawAngle}"); // DEBUG
                    return rawAngle;
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Failed to read sensor value: {ex.Message}");
                throw; // Re-throw to handle in the main loop
            }
        }
    }
}
