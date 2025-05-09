
using Avalonia.Threading;
using System;
using System.Diagnostics;
using System.IO.Ports;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Collections.ObjectModel;
using System.Net.Sockets;
using System.IO;
using System.Collections.Concurrent;
using System.Linq;

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
        private Thread _arduinoThread;
        private BlockingCollection<string> _arduinoCommandQueue = new BlockingCollection<string>();
        public bool _arduinoThreadRunning = false;
        private readonly string _arduinoServerIp = AppSettings.Instance.RemoteIP; // Cache IP
        private const int _arduinoServerPort = 23457;
        private CancellationTokenSource _arduinoCts; // For cancelling the Arduino thread
        private TcpClient? _commandClient;
        private NetworkStream? _commandStream;
        private StreamWriter? _commandWriter;
        private bool connected = false;


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
        public void StartArduinoThread()
        {
            if (_arduinoThread == null || !_arduinoThreadRunning)
            {
                _arduinoThread = new Thread(ArduinoThreadWorker);
                _arduinoThread.IsBackground = true; // Important: Allow app to exit
                _arduinoThread.Start(AppSettings.Instance.RemoteIP); // Pass the IP
                _arduinoThreadRunning = true;
                Console.WriteLine("[CameraController] Arduino thread started.");
            }
            else
            {
                Console.WriteLine("[CameraController] Arduino thread already running.");
            }
        }
        public void CleanupArduinoResources()
        {
            try { _commandWriter?.Dispose(); } catch (Exception ex) { Console.WriteLine($"[Cleanup Error] CommandWriter: {ex.Message}"); }
            _commandWriter = null;
            try { _commandStream?.Dispose(); } catch (Exception ex) { Console.WriteLine($"[Cleanup Error] CommandStream: {ex.Message}"); }
            _commandStream = null;
            try { _commandClient?.Dispose(); } catch (Exception ex) { Console.WriteLine($"[Cleanup Error] CommandClient: {ex.Message}"); }
            _commandClient = null;
        }

        public async Task SendArduinoCommandAsync(string command)
        {
            if (_commandWriter != null)
            {
                try
                {
                    await _commandWriter.WriteLineAsync(command); // Send command directly
                    //Console.WriteLine($"[COMMAND SENT] {command}"); // Optional log
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"[COMMAND ERROR] Failed to send command '{command}': {ex.Message}");
                    // Consider triggering a disconnect/reconnect?
                }
            }
            else
            {
                Console.WriteLine($"[COMMAND WARN] Cannot send '{command}', not connected.");
            }
        }
        public void StopArduinoThread()
        {
            // --- START ADDED LOGGING ---
            string callStack = Environment.StackTrace; // Get the call stack
            Console.WriteLine($"[DEBUG StopArduinoThread] CALLED. _arduinoThreadRunning was: {_arduinoThreadRunning}. Thread is {(_arduinoThread == null ? "null" : "not null")}.");
            Console.WriteLine($"[DEBUG StopArduinoThread] Call Stack:\n{callStack}");
            // --- END ADDED LOGGING ---

            if (_arduinoThread != null && _arduinoThreadRunning)
            {
                _arduinoCommandQueue.CompleteAdding();
                Console.WriteLine($"[DEBUG StopArduinoThread] Called CompleteAdding. Joining thread...");
                bool joined = _arduinoThread.Join(1000);
                Console.WriteLine($"[DEBUG StopArduinoThread] Thread join completed: {joined}. IsAlive: {_arduinoThread?.IsAlive}");

                if (!joined && _arduinoThread != null && _arduinoThread.IsAlive)
                {
                    // _arduinoThread.Interrupt(); // Generally avoid if possible
                    Console.WriteLine("[CameraController] Arduino thread join timed out but was still alive. Proceeding to mark as stopped.");
                }
                _arduinoThread = null;
                _arduinoThreadRunning = false; // Flag set to false
                CleanupArduinoResources();
                Console.WriteLine("[CameraController] Arduino thread stopped. _arduinoThreadRunning is now false.");
            }
            else
            {
                Console.WriteLine($"[CameraController] Arduino thread already not running or null when StopArduinoThread was called again. _arduinoThreadRunning: {_arduinoThreadRunning}");
            }
        }
        private async void ArduinoThreadWorker(object? obj) // Changed to async void
        {
            string serverIp = (string)obj;
            int serverPort = 23457;
            try
            {
                // --- Connect to Arduino Command Server (Persistent Connection) ---
                while (!connected) // Keep trying to connect until exit condition
                {
                    try
                    {
                        _commandClient = new TcpClient();
                        await _commandClient.ConnectAsync(serverIp, serverPort);
                        _commandStream = _commandClient.GetStream();
                        _commandWriter = new StreamWriter(_commandStream, Encoding.UTF8) { AutoFlush = true };
                        Console.WriteLine("[ArduinoThread] Connected to Arduino command server.");
                        connected = true;
                        break; // Connection successful, exit the connection loop
                    }
                    catch (SocketException sockEx)
                    {
                        Console.WriteLine($"[ArduinoThread] Socket Exception: {sockEx.Message}. Reconnecting in 2 seconds...");
                        CleanupArduinoResources(); // Clean up resources before retry
                        await Task.Delay(2000);    // Wait before retrying
                    }
                    catch (Exception ex)
                    {
                        Console.WriteLine($"[ArduinoThread] Error connecting: {ex.Message}");
                        CleanupArduinoResources();
                        break; // Exit on unexpected error
                    }
                }
                Console.WriteLine("[ArduinoThread] starting command loop.");

                try
                {
                    // --- Process Commands (Continuous Loop) ---
                    while (_arduinoThreadRunning) // Use the running flag as the main loop condition
                    {

                        string? command = null;
                        //try
                        //{
                        if (_arduinoCommandQueue.Count >= 1)
                        {
                            
                                command = _arduinoCommandQueue.Last();
                                _commandWriter.WriteLine(command); // Send command

                            

                        }
                        //catch (InvalidOperationException)
                        //{
                        //    // Thrown when CompleteAdding is called and queue is empty
                        //    Console.WriteLine("[ArduinoThread] Command queue is completing.");
                        //    break; // Exit the loop
                        //}

                        //if (!string.IsNullOrEmpty(command))
                        //{

                        //    _commandWriter.WriteLine(command); // Send command
                        //    Console.WriteLine($"[ArduinoThread] Sent command: {command}");


                        //}
                        
                        
                        //_arduinoCommandQueue = new BlockingCollection<string>();
                        ClearArduinoCommandQueue();
                        Thread.Sleep(1); // Small delay to prevent tight-looping

                    }
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"[ArduinoThread] Unexpected error starting command loop: {ex.Message}");
                }

            }
            finally
            {
                CleanupArduinoResources();
                _arduinoCommandQueue.CompleteAdding(); // Signal any waiting consumers
                _arduinoCommandQueue.Dispose();
                _arduinoThreadRunning = false;
                Console.WriteLine("[ArduinoThread] Arduino thread ended.");
            }
        }



        public void EnqueueArduinoCommand(string command)
        {
            try
            {
                _arduinoCommandQueue.Add(command);
                Console.WriteLine($"[CameraController] Enqueued command: {command}");
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[CameraController] command not able to send {ex}.");
            }
            //ClearArduinoCommandQueue();
        }
        public void ClearArduinoCommandQueue()
        {
            // Keep taking items from the queue until it's empty.
            // The 'out _' (discard) is used because we don't need the value of the items being removed.
            while (_arduinoCommandQueue.Count != 0)
            {
                _arduinoCommandQueue.Take();
                // No action needed here, just consuming and discarding items.
            }
            Console.WriteLine("Arduino command queue has been emptied.");
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
