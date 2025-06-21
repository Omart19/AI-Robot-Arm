
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

        //public ArduinoController()
        //{
        //    // ...
        //    _arduinoCts = new CancellationTokenSource(); // Initial CTS
        //}

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
            if (!_arduinoThreadRunning) // Use the flag to check
            {
                _arduinoCts?.Dispose(); // Dispose previous CTS if any
                _arduinoCts = new CancellationTokenSource();

                // Re-initialize BlockingCollection if it was completed/disposed
                if (_arduinoCommandQueue == null || _arduinoCommandQueue.IsAddingCompleted)
                {
                    _arduinoCommandQueue = new BlockingCollection<string>();
                }

                _arduinoThread = new Thread(ArduinoThreadWorker);
                _arduinoThread.IsBackground = true;
                _arduinoThread.Name = "ArduinoCommandThread";
                _arduinoThreadRunning = true; // Set flag before starting
                _arduinoThread.Start(AppSettings.Instance.RemoteIP);
                Console.WriteLine("[ArduinoController] Arduino command thread started.");
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
            Console.WriteLine($"[ArduinoController DEBUG StopArduinoThread] CALLED. _arduinoThreadRunning was: {_arduinoThreadRunning}.");
            if (_arduinoThreadRunning) // Check the flag
            {
                _arduinoThreadRunning = false; // Set flag immediately to signal loop to stop

                _arduinoCts?.Cancel(); // Signal cancellation to tasks/blocking calls

                // Signal the BlockingCollection that no more items will be added.
                // This will cause Take() to throw InvalidOperationException if the queue becomes empty.
                if (_arduinoCommandQueue != null && !_arduinoCommandQueue.IsAddingCompleted)
                {
                    _arduinoCommandQueue.CompleteAdding();
                    Console.WriteLine($"[ArduinoController DEBUG StopArduinoThread] Called CompleteAdding on queue.");
                }

                if (_arduinoThread != null && _arduinoThread.IsAlive)
                {
                    Console.WriteLine($"[ArduinoController DEBUG StopArduinoThread] Attempting to join Arduino thread...");
                    bool joined = _arduinoThread.Join(1500); // Increased timeout slightly
                    Console.WriteLine($"[ArduinoController DEBUG StopArduinoThread] Arduino thread join completed: {joined}. IsAlive after join: {_arduinoThread?.IsAlive}");
                    if (!joined && _arduinoThread.IsAlive)
                    {
                        Console.WriteLine("[ArduinoController WARN StopArduinoThread] Arduino thread did not join in time.");
                        // Avoid Thread.Abort() if possible. The CancellationToken and loop flag should handle it.
                    }
                }
                _arduinoThread = null; // Null out the thread object
            }
            else
            {
                Console.WriteLine($"[ArduinoController DEBUG StopArduinoThread] Arduino thread was already not running or null.");
            }
            // Cleanup resources regardless, as they might be in an inconsistent state if thread didn't stop cleanly
            CleanupArduinoResources();
            Console.WriteLine("[ArduinoController DEBUG StopArduinoThread] StopArduinoThread finished.");
        }

        private async void ArduinoThreadWorker(object? obj) // Keep async void if it's a top-level thread method
        {
            string serverIp = (string)obj!; // Assuming obj is never null here based on StartArduinoThread
            int serverPort = 23457;        // _arduinoServerPort

            while (_arduinoThreadRunning) // Outer loop for maintaining connection
            {
                if (!connected) // Check if we need to connect/reconnect
                {
                    try
                    {
                        Console.WriteLine($"[ArduinoThread] Attempting to connect to {serverIp}:{serverPort}...");
                        _commandClient = new TcpClient();
                        // Use a timeout for connection attempts
                        var connectTask = _commandClient.ConnectAsync(serverIp, serverPort);
                        if (await Task.WhenAny(connectTask, Task.Delay(3000, _arduinoCts?.Token ?? CancellationToken.None)) == connectTask && connectTask.IsCompletedSuccessfully)
                        {
                            _commandStream = _commandClient.GetStream();
                            _commandWriter = new StreamWriter(_commandStream, Encoding.UTF8) { AutoFlush = true };
                            Console.WriteLine("[ArduinoThread] Connected to Arduino command server.");
                            connected = true;
                        }
                        else
                        {
                            Console.WriteLine("[ArduinoThread] Connection attempt timed out or failed.");
                            _commandClient?.Dispose(); // Dispose if connect failed
                            _commandClient = null;
                            if (!_arduinoThreadRunning) break; // Exit if stop was requested during connect attempt
                            await Task.Delay(2000, _arduinoCts?.Token ?? CancellationToken.None); // Wait before retrying connection
                            continue; // Retry connection
                        }
                    }
                    catch (OperationCanceledException)
                    {
                        Console.WriteLine("[ArduinoThread] Connection attempt cancelled.");
                        _arduinoThreadRunning = false; // Ensure loop terminates
                        break;
                    }
                    catch (Exception ex)
                    {
                        Console.WriteLine($"[ArduinoThread] Error connecting: {ex.Message}");
                        CleanupArduinoResources(); // Clears _commandClient, _commandStream, _commandWriter
                        if (!_arduinoThreadRunning) break;
                        await Task.Delay(2000, _arduinoCts?.Token ?? CancellationToken.None); // Wait before retrying
                        continue; // Retry connection
                    }
                }

                // Process commands if connected
                if (connected && _commandWriter != null)
                {
                    string? commandToSend = null;
                    try
                    {
                        // Take one command. This will block if the queue is empty until a command is added
                        // or until CompleteAdding is called (then throws InvalidOperationException).
                        // Use CancellationToken for Take if _arduinoCts is available and used for stopping.
                        //commandToSend = _arduinoCommandQueue.Take(_arduinoCts?.Token ?? CancellationToken.None);
                        commandToSend = _arduinoCommandQueue.Take(_arduinoCts?.Token ?? CancellationToken.None);

                        if (!string.IsNullOrEmpty(commandToSend))
                        {
                            await _commandWriter.WriteLineAsync(commandToSend);
                            Console.WriteLine($"[ArduinoThread] Sent command: {commandToSend}");

                            // OPTIONAL: Add a small delay here to pace commands sent to the Pi/Arduino.
                            // This gives the Pi/Arduino time to process one command before the next.
                            // Adjust the delay as needed (e.g., 20-50ms).
                            await Task.Delay(1, _arduinoCts?.Token ?? CancellationToken.None); // e.g., 30ms delay
                        }
                    }
                    catch (InvalidOperationException) // Thrown by Take if CompleteAdding has been called and queue is empty.
                    {
                        Console.WriteLine("[ArduinoThread] Command queue completed, worker stopping.");
                        _arduinoThreadRunning = false; // Signal main loop to exit
                    }
                    catch (OperationCanceledException)
                    {
                        Console.WriteLine("[ArduinoThread] Take operation cancelled, worker stopping.");
                        _arduinoThreadRunning = false; // Signal main loop to exit
                    }
                    catch (IOException ioEx) // Covers network errors during WriteLineAsync
                    {
                        Console.WriteLine($"[ArduinoThread] IOException sending command '{commandToSend}': {ioEx.Message}. Assuming disconnect.");
                        connected = false; // Trigger reconnect logic in the outer loop
                        CleanupArduinoResources();
                        // Optionally, re-enqueue commandToSend if it's critical and hasn't been processed.
                    }
                    catch (Exception ex)
                    {
                        Console.WriteLine($"[ArduinoThread] Error sending command '{commandToSend}': {ex.Message}");
                        // Decide if this error means a disconnect or if we can continue.
                        // If it might be a disconnect:
                        // connected = false;
                        // CleanupArduinoResources();
                        if (_arduinoThreadRunning)
                        { // Avoid delay if stopping
                            try { await Task.Delay(100, _arduinoCts?.Token ?? CancellationToken.None); } catch { /* ignore cancellation on delay */ }
                        }
                    }
                }
                else if (connected && _commandWriter == null) // Should not happen if connected is true
                {
                    Console.WriteLine("[ArduinoThread] Error: Connected but _commandWriter is null. Forcing reconnect.");
                    connected = false;
                    CleanupArduinoResources();
                }
            } // End while (_arduinoThreadRunning)

            // Final cleanup when thread is exiting
            CleanupArduinoResources();
            if (_arduinoCommandQueue != null && !_arduinoCommandQueue.IsAddingCompleted)
            {
                _arduinoCommandQueue.CompleteAdding();
            }
            Console.WriteLine("[ArduinoThread] Arduino thread fully ended.");
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
