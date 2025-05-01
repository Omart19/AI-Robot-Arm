
using Avalonia.Threading;
using System;
using System.Diagnostics;
using System.IO.Ports;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Collections.ObjectModel;
using System.Net.Sockets;

namespace RobotAIArm.Controllers
{
    public class ArduinoController : IDisposable
    {
        private SerialPort? _serialPort;
        private string _portName = "";
        private int _baudRate = 9600;
        public event EventHandler<string>? LogMessageAvailable;
        public string ConfiguredPortName => _portName;

        public bool IsConnected => _serialPort?.IsOpen ?? false;

        // Set the COM port manually (used by normal mode)
        public void SetPort(string portName)
        {
            if (IsConnected)
            {
                Console.WriteLine("Cannot change port while connected.");
                return;
            }
            _portName = portName;
            Console.WriteLine($"Port set to: {_portName}");
        }

        public void SetBaudRate(int baudRate)
        {
            if (IsConnected)
            {
                Console.WriteLine("Cannot change baud rate while connected.");
                return;
            }
            _baudRate = baudRate;
            Console.WriteLine($"Baud rate set to: {_baudRate}");
        }

        public async Task<bool> ConnectAsync()
        {
            if (AppSettings.Instance.IsRemoteMode)
            {
                Console.WriteLine("[INFO] Arduino connection is disabled in remote mode.");
                return true;
            }

            if (IsConnected)
            {
                Console.WriteLine("Already connected.");
                return true;
            }

            if (string.IsNullOrEmpty(_portName))
            {
                Console.WriteLine("Error: Port name not set.");
                return false;
            }

            bool success = false;

            await Task.Run(() =>
            {
                try
                {
                    _serialPort = new SerialPort(_portName, _baudRate)
                    {
                        Parity = Parity.None,
                        DataBits = 8,
                        StopBits = StopBits.One,
                        Handshake = Handshake.None,
                        ReadTimeout = 1000,
                        WriteTimeout = 1000
                    };

                    _serialPort.DataReceived += SerialPort_DataReceived;

                    Console.WriteLine($"Attempting to open port {_portName}...");
                    _serialPort.Open();
                    Console.WriteLine($"Port {_portName} opened successfully.");
                    Thread.Sleep(500);

                    success = true;
                }
                catch (Exception ex)
                {
                    RaiseLogEvent($"[ERROR] {ex.Message}");
                    success = false;
                }
            });

            if (success)
                RaiseLogEvent($"Connected to Arduino on {_portName}.");
            else
                RaiseLogEvent($"Failed to connect Arduino on {_portName}.");

            return success;
        }

        public void Disconnect()
        {
            if (AppSettings.Instance.IsRemoteMode)
            {
                Console.WriteLine("[INFO] Arduino disconnect skipped in remote mode.");
                return;
            }

            if (!IsConnected || _serialPort == null)
                return;

            try
            {
                _serialPort.DataReceived -= SerialPort_DataReceived;
                if (_serialPort.IsOpen)
                    _serialPort.Close();
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error during disconnect: {ex.Message}");
            }
            finally
            {
                _serialPort?.Dispose();
                _serialPort = null;
                Console.WriteLine("Arduino disconnected.");
            }
        }

        public async Task SendCommandAsync(string command)
        {
            if (AppSettings.Instance.IsRemoteMode)
            {
                try
                {
                    using var client = new TcpClient();
                    await client.ConnectAsync(AppSettings.Instance.RemoteIP, 23457);
                    using var stream = client.GetStream();
                    byte[] data = Encoding.UTF8.GetBytes(command + "\n");
                    await stream.WriteAsync(data, 0, data.Length);
                    Console.WriteLine($"[REMOTE] Sent Arduino command: {command}");
                }
                catch (Exception ex)
                {
                    Console.WriteLine("[REMOTE ERROR] Failed to send command: " + ex.Message);
                }
                return;
            }

            if (!IsConnected || _serialPort == null)
            {
                RaiseLogEvent("Error: Cannot send command - Not connected.");
                return;
            }

            await Task.Run(() =>
            {
                try
                {
                    RaiseLogEvent($"TX: {command}");
                    _serialPort.WriteLine(command);
                }
                catch (Exception ex)
                {
                    RaiseLogEvent($"Error sending: {ex.Message}");
                }
            });
        }


        private void SerialPort_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            if (_serialPort == null || !_serialPort.IsOpen) return;

            try
            {
                string receivedData = _serialPort.ReadLine();
                Dispatcher.UIThread.InvokeAsync(() =>
                {
                    Console.WriteLine($"Received: {receivedData.Trim()}");
                });
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error reading serial: {ex.Message}");
            }
        }

        public static string[] GetAvailablePorts()
        {
            return SerialPort.GetPortNames();
        }

        private void RaiseLogEvent(string message)
        {
            Dispatcher.UIThread.InvokeAsync(() =>
            {
                LogMessageAvailable?.Invoke(this, message);
            });
        }

        public void Dispose()
        {
            Disconnect();
            GC.SuppressFinalize(this);
        }
    }
}
