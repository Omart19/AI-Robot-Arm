using Avalonia.Threading; // For Dispatcher
using System;
using System.Diagnostics;
using System.IO.Ports;
using System.Text; // For StringBuilder
using System.Threading;
using System.Threading.Tasks;
using System.Collections.ObjectModel; // For ObservableCollection if logging messages

// Optional: If using MVVM pattern with CommunityToolkit.Mvvm
// using CommunityToolkit.Mvvm.ComponentModel;

// Make this class ObservableObject if using MVVM for property change notifications
// public class ArduinoController : ObservableObject
namespace RobotAIArm.Controllers
{
public class ArduinoController : IDisposable
{
    private SerialPort? _serialPort;
    private string _portName = "";
    private int _baudRate = 9600; // Default, can be changed

    // --- Optional Properties for UI Binding (using MVVM pattern) ---
    // Uncomment and use CommunityToolkit.Mvvm if needed
    // [ObservableProperty]
    // [NotifyPropertyChangedFor(nameof(IsNotConnected))]
    // private bool _isConnected;
    // public bool IsNotConnected => !_isConnected;
    public event EventHandler<string>? LogMessageAvailable; // Nullable EventHandler
    public string ConfiguredPortName => _portName;


    // // Example: Log received messages for display
    // public ObservableCollection<string> ReceivedMessages { get; } = new ObservableCollection<string>();
    // --- End Optional Properties ---

    // Simpler status flag if not using MVVM
    public bool IsConnected => _serialPort?.IsOpen ?? false;


    // --- Configuration ---
    public void SetPort(string portName)
    {
        if (IsConnected)
        {
            Console.WriteLine("Cannot change port while connected.");
            // Or throw new InvalidOperationException("Cannot change port while connected.");
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
            // Or throw new InvalidOperationException("Cannot change baud rate while connected.");
            return;
        }
        _baudRate = baudRate;
         Console.WriteLine($"Baud rate set to: {_baudRate}");
    }

    // --- Connection Management ---

    public async Task<bool> ConnectAsync()
    {
        if (IsConnected)
        {
            Console.WriteLine("Already connected.");
            return true;
        }

        if (string.IsNullOrEmpty(_portName))
        {
            Console.WriteLine("Error: Port name not set.");
            // Optionally update UI status via Dispatcher here
            return false;
        }

        bool success = false;
        // Use Task.Run to avoid blocking UI thread during port opening attempt
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
                    ReadTimeout = 1000, // Timeout for ReadLine in event handler
                    WriteTimeout = 1000
                };

                // Subscribe to the DataReceived event BEFORE opening the port
                _serialPort.DataReceived += SerialPort_DataReceived;

                Console.WriteLine($"Attempting to open port {_portName}...");
                _serialPort.Open();
                Console.WriteLine($"Port {_portName} opened successfully.");
                Thread.Sleep(500); // Wait 500ms


                // Update status property (use Dispatcher if needed for UI binding)
                // For MVVM: SetProperty(ref _isConnected, true);
                success = true;

                // Optional: Send an initial handshake/ready message?
                // SendCommandAsync("READY");
            }
            catch (UnauthorizedAccessException ex)
            {
                RaiseLogEvent($"Error: Access denied to port {_portName}. In use or permissions issue? {ex.Message}");
                Thread.Sleep(500); // Wait 500ms
                
                // Update UI via Dispatcher if needed
            }
            catch (System.IO.IOException ex)
            {
                RaiseLogEvent($"Error: Port {_portName} does not exist or is invalid. {ex.Message}");
                success = false; // Ensure success is false
                // Update UI via Dispatcher if needed
            }
            catch (Exception ex) // Catch other potential errors
            {
                Console.WriteLine($"An unexpected error occurred during connect: {ex.Message}");
                // Update UI via Dispatcher if needed
                RaiseLogEvent($"An unexpected error occurred during connect: {ex.Message}");
                success = false; // Ensure success is false
                if (_serialPort != null)
                {
                
                     // Clean up event handler if something went wrong after subscription but before success
                     _serialPort.DataReceived -= SerialPort_DataReceived;
                    _serialPort.Dispose();
                    _serialPort = null;
                }
            }
        });
if (success)
    {
        RaiseLogEvent($"Connection task completed successfully for {ConfiguredPortName}.");
    }
    else
    {
        RaiseLogEvent($"Connection task failed for {ConfiguredPortName}.");
    }

    return success;    }

    public void Disconnect()
    {
        if (!IsConnected || _serialPort == null)
        {
            Console.WriteLine("Not connected or already disconnected.");
            return;
        }

        Console.WriteLine("Disconnecting...");
        try
        {
            // Unsubscribe from event BEFORE closing
            _serialPort.DataReceived -= SerialPort_DataReceived;

            if (_serialPort.IsOpen)
            {
                _serialPort.Close();
            }
        }
        catch (Exception ex)
        {
            Console.WriteLine($"Error during port closing: {ex.Message}");
        }
        finally
        {
             // Dispose should be called to release resources
            _serialPort?.Dispose();
            _serialPort = null;
            Console.WriteLine("Port closed and disposed.");
            // Update status property (use Dispatcher if needed for UI binding)
            // For MVVM: SetProperty(ref _isConnected, false);
        }
    }

    // --- Sending Data ---

    public async Task SendCommandAsync(string command)
{
    if (!IsConnected || _serialPort == null)
    {
        // Use RaiseLogEvent for consistency, even for errors before sending
        RaiseLogEvent("Error: Cannot send command - Not connected.");
        return;
    }

    // Use Task.Run for the write operation to prevent blocking UI
    await Task.Run(() =>
    {
        try
        {
            // ** FIX: Use RaiseLogEvent to send log to UI **
            RaiseLogEvent($"TX: {command}"); // Log the command being sent via the event

            // Actually send the command
            _serialPort.WriteLine(command);
        }
        catch (TimeoutException ex)
        {
            RaiseLogEvent($"Error: Write Timeout: {ex.Message}");
        }
        catch (InvalidOperationException ex) // Port might have been closed unexpectedly
        {
            RaiseLogEvent($"Error: Send failed (port closed?): {ex.Message}");
            // Attempt to update state via Disconnect, which also raises events
            Disconnect(); // Attempt cleanup
        }
        catch (Exception ex)
        {
            RaiseLogEvent($"Error: Sending command: {ex.Message}");
        }
    });
}
    // --- Receiving Data ---

    private void SerialPort_DataReceived(object sender, SerialDataReceivedEventArgs e)
    {
        if (_serialPort == null || !_serialPort.IsOpen) return;

        try
        {
            // ReadLine() blocks until a newline is received or ReadTimeout occurs
            // It runs on a ThreadPool thread here, NOT the UI thread
            string receivedData = _serialPort.ReadLine();

            // Use Dispatcher to update UI elements safely from this background thread
            Dispatcher.UIThread.InvokeAsync(() =>
            {
                Console.WriteLine($"Received: [{receivedData.Trim()}]"); // Log to console

                // --- Update UI elements here ---
                // Example: Add to an ObservableCollection for a log display
                // ReceivedMessages.Add(receivedData.Trim());
                // if (ReceivedMessages.Count > 100) ReceivedMessages.RemoveAt(0); // Limit log size

                // Example: Update a status TextBlock
                // myStatusTextBlock.Text = $"Last Msg: {receivedData.Trim()}";

            });
        }
        catch (TimeoutException)
        {
            // Read timed out - often normal if Arduino doesn't always respond instantly
            // Can read remaining bytes with ReadExisting() if needed, but ReadLine is cleaner for line-based protocols
             Console.WriteLine("Read Timeout in DataReceived handler.");
        }
        catch (InvalidOperationException)
        {
             // Port might have been closed between check and read - handle gracefully
             Console.WriteLine("Port closed during read in DataReceived handler.");
        }
        catch (Exception ex)
        {
             Console.WriteLine($"Error in DataReceived handler: {ex.Message}");
             // Use Dispatcher to show error on UI if needed
             Dispatcher.UIThread.InvokeAsync(() => {
                 // Update some error status label on UI
             });
        }
    }

     // --- Utility ---

    public static string[] GetAvailablePorts()
    {
        return SerialPort.GetPortNames();
    }


    // --- Cleanup ---
    public void Dispose()
    {
        Disconnect(); // Ensure port is closed and disposed
        GC.SuppressFinalize(this);
    }

    // private void SerialPort_DataReceived(object sender, SerialDataReceivedEventArgs e)
    // {
    //     if (_serialPort == null || !_serialPort.IsOpen) return;
    //     string receivedData = "Error reading data"; // Default error message

    //     try
    //     {
    //         // ReadLine() runs on a ThreadPool thread
    //         receivedData = _serialPort.ReadLine().Trim(); // Read and trim whitespace

    //         // Use Dispatcher BEFORE raising the event to ensure subscribers get it on the UI thread
    //         Dispatcher.UIThread.InvokeAsync(() =>
    //         {
    //             // Raise the event with the received data
    //             LogMessageAvailable?.Invoke(this, $"RX: {receivedData}"); // Prefix with RX:
    //         });
    //     }
    //     catch (TimeoutException)
    //     {
    //         // Log timeouts if needed, but often they are expected if no data/newline arrives
    //          Dispatcher.UIThread.InvokeAsync(() => LogMessageAvailable?.Invoke(this, "Info: Read Timeout"));
    //     }
    //     catch (Exception ex)
    //     {
    //          // Log other errors
    //          receivedData = $"Error reading serial: {ex.Message}";
    //          Dispatcher.UIThread.InvokeAsync(() => LogMessageAvailable?.Invoke(this, $"Error: {receivedData}"));
    //     }
    // }

    // Helper method within ArduinoController to raise the log event from other places
    private void RaiseLogEvent(string message)
    {
         // Ensure it runs on UI thread for consistency
         Dispatcher.UIThread.InvokeAsync(() =>
         {
              LogMessageAvailable?.Invoke(this, message);
         });
    }

    // Modify ConnectAsync, Disconnect, SendCommandAsync to call RaiseLogEvent
    // Example modification in SendCommandAsync's Task.Run:
    // public async Task SendCommandAsync(string command)
    // {
    //     // ... (check if connected) ...
    //     await Task.Run(() =>
    //     {
    //         try
    //         {
    //             // Use RaiseLogEvent instead of Console.WriteLine
    //             RaiseLogEvent($"TX: {command}");
    //             _serialPort.WriteLine(command);
    //         }
    //         catch (TimeoutException ex) { RaiseLogEvent($"Error: Write Timeout: {ex.Message}"); }
    //         catch (InvalidOperationException ex) { RaiseLogEvent($"Error: Send failed (port closed?): {ex.Message}"); Disconnect(); }
    //         catch (Exception ex) { RaiseLogEvent($"Error: Sending command: {ex.Message}"); }
    //     });
    // }

    // Optional: Finalizer (if you have unmanaged resources not handled by SerialPort itself)
    // ~ArduinoController()
    // {
    //     Dispose();
    // }
}
}