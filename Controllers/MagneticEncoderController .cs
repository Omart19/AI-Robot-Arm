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
        public List<string> ReadDataFromSensors()
        {
            int busId = 1; // Adjust for your system
            int tcaAddress = 0x70; // TCA9548A address
            var sensorAddresses = new Dictionary<int, int>
            {
                { 0, 0x36 }, // Channel 0
                { 1, 0x36 }, // Channel 1
                { 2, 0x36 }, // Channel 2
                { 3, 0x36 }  // Channel 3
            };

            var angles = new List<string>();
            var i2cBus = I2cBus.Create(busId);

            foreach (var channel in sensorAddresses.Keys)
            {
                try
                {
                    // Select the channel
                    SelectTcaChannel(i2cBus, tcaAddress, channel);

                    // Read sensor value
                    int angle = ReadSensorValue(i2cBus, sensorAddresses[channel]);

                    angles.Add(angle.ToString());
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"Error on channel {channel}: {ex.Message}");
                    angles.Add("Error");
                }
            }

            // Output all angles
            // Console.WriteLine("Final Angles:");
            // for (int i = 0; i < angles.Count; i++)
            // {
            //     Console.WriteLine($"Channel {i}: {angles[i]}");
            // }

            return angles;
        }

        private static void SelectTcaChannel(I2cBus i2cBus, int tcaAddress, int channel)
        {
            using (var tcaDevice = i2cBus.CreateDevice(tcaAddress))
            {
                byte channelMask = (byte)(1 << channel); // Enable desired channel
                tcaDevice.WriteByte(channelMask);

                // Optional: Add a small delay to ensure the channel is active
                System.Threading.Thread.Sleep(10);

                //Console.WriteLine($"Channel {channel} selected with mask: 0x{channelMask:X2}");
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
