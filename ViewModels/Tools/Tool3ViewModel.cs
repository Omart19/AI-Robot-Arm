using Avalonia.Threading;
using CommunityToolkit.Mvvm.ComponentModel; // NuGet Package: CommunityToolkit.Mvvm
using CommunityToolkit.Mvvm.Input;         // NuGet Package: CommunityToolkit.Mvvm
using RobotAIArm.Controllers;              // Your namespace for ArduinoController
using System;
using System.Collections.ObjectModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Dock.Model.Mvvm.Controls;



namespace RobotAIArm.ViewModels.Tools
{
    // Inherit from ObservableObject for INotifyPropertyChanged implementation
    public partial class Tool3ViewModel : Tool
    {
        private readonly ArduinoController _arduinoController;
        private readonly StringBuilder _logBuilder = new StringBuilder();

        // --- Observable Properties for UI Binding ---

        [ObservableProperty]
        private ObservableCollection<string> _availablePorts = new(); // Use ObservableCollection for dynamic updates

        [ObservableProperty]
        [NotifyCanExecuteChangedFor(nameof(ConnectCommand))] // Update command state when port changes
        private string? _selectedPort; // Nullable

        [ObservableProperty]
        [NotifyPropertyChangedFor(nameof(IsNotConnected))]
        private bool _isConnected;

        public bool IsNotConnected => !_isConnected;

        [ObservableProperty]
        private string _connectButtonText = "Connect";

        [ObservableProperty]
        private string _connectionStatus = "Disconnected";

        [ObservableProperty]
        private string _consoleOutputText = "";

        [ObservableProperty]
        [NotifyCanExecuteChangedFor(nameof(SendCommand))] // Update command state when text changes
        private string _commandToSend = "";

        // --- Constructor ---

        public Tool3ViewModel(ArduinoController arduinoController)
        {
            _arduinoController = arduinoController;
            _arduinoController.LogMessageAvailable += HandleArduinoLogMessage;

            LoadAvailablePorts();
            UpdateUiConnectionState(false); // Initial state
        }

        // --- Commands for UI Binding ---

        private bool CanConnect() => !string.IsNullOrEmpty(SelectedPort); // Can only connect if a port is selected

        [RelayCommand(CanExecute = nameof(CanConnect))]
        private async Task ConnectAsync()
        {
            // Button text/status updated via HandleArduinoLogMessage based on controller events
            if (IsConnected)
            {
                _arduinoController.Disconnect();
            }
            else
            {
                if (SelectedPort != null)
                {
                    _arduinoController.SetPort(SelectedPort);
                    // Optionally set baud rate if needed
                    // _arduinoController.SetBaudRate(115200);
                    await _arduinoController.ConnectAsync();
                    // Success/fail message will come via LogMessageAvailable event
                }
                else
                {
                    // Log via the handler
                    HandleArduinoLogMessage(this, "Error: No serial port selected.");
                }
            }
            // Re-evaluate CanExecute for the command itself
            ConnectCommand.NotifyCanExecuteChanged();
            SendCommand.NotifyCanExecuteChanged(); // Send command state might change
        }

        private bool CanSendCommand() => IsConnected && !string.IsNullOrWhiteSpace(CommandToSend);

        [RelayCommand(CanExecute = nameof(CanSendCommand))]
        private async Task SendAsync() // Renamed from SendCommand to avoid conflict with property name convention
        {
            if (CanSendCommand()) // Redundant check, but safe
            {
                string command = CommandToSend; // Copy command before clearing
                CommandToSend = ""; // Clear input box immediately (updates via ObservableProperty)
                await _arduinoController.SendCommandAsync(command);
            }
        }


        // --- Methods ---

        private void LoadAvailablePorts()
        {
            var ports = ArduinoController.GetAvailablePorts();
            AvailablePorts.Clear();
            foreach (var port in ports)
            {
                AvailablePorts.Add(port);
            }

            // Select the last port by default (often the Arduino on Linux/Mac)
            SelectedPort = AvailablePorts.LastOrDefault();

            if (!AvailablePorts.Any())
            {
                HandleArduinoLogMessage(this, "Warning: No serial ports found.");
            }
            // Ensure connect command state is updated after loading ports
            ConnectCommand.NotifyCanExecuteChanged();
        }

        // Handler for messages from ArduinoController
        private void HandleArduinoLogMessage(object? sender, string message)
        {
            // This should arrive on the UI thread because of the Dispatcher in ArduinoController
            AppendToLog(message);

            // Update connection state based on specific messages
            bool newConnectionState = _arduinoController.IsConnected; // Check current actual state
            if (message.Contains("opened successfully")) newConnectionState = true;
            if (message.Contains("closed") || message.Contains("Disconnecting") || (message.StartsWith("Error:") && message.Contains("port closed?")) || message.Contains("Failed to connect")) newConnectionState = false;

            if (IsConnected != newConnectionState)
            {
                UpdateUiConnectionState(newConnectionState);
            }
        }

        // Method to append messages to the log TextBlock property
        private void AppendToLog(string message)
        {
            // Assumes already on UI thread thanks to ArduinoController event handling
            string timestamp = DateTime.Now.ToString("HH:mm:ss.fff");
            _logBuilder.AppendLine($"[{timestamp}] {message}");

            // Optional: Limit log length
            const int maxLogLength = 10000;
            if (_logBuilder.Length > maxLogLength)
            {
                _logBuilder.Remove(0, _logBuilder.Length - maxLogLength);
                if (!_logBuilder.ToString().StartsWith("[... Trimmed ...]"))
                {
                    _logBuilder.Insert(0, "[... Trimmed ...]\n");
                }
            }
            // Update the bound property
            ConsoleOutputText = _logBuilder.ToString();

            // Scrolling needs to be handled in the View if possible,
            // as ViewModel shouldn't know about ScrollViewer.
            // Alternatively, raise an event the View subscribes to trigger scroll.
        }

        private void UpdateUiConnectionState(bool connectedState)
        {
            // Assumes already on UI thread
            IsConnected = connectedState; // This triggers PropertyChanged via [ObservableProperty]
            ConnectButtonText = IsConnected ? "Disconnect" : "Connect";
            ConnectionStatus = IsConnected ? $"Connected ({_arduinoController.ConfiguredPortName})" : "Disconnected";

            // Important: Manually notify dependent command states
            ConnectCommand.NotifyCanExecuteChanged();
            SendCommand.NotifyCanExecuteChanged();
        }


        // --- Cleanup ---
        public void Dispose()
        {
            if (_arduinoController != null)
            {
                _arduinoController.LogMessageAvailable -= HandleArduinoLogMessage; // Unsubscribe
                _arduinoController.Dispose();
            }
            GC.SuppressFinalize(this);
            Console.WriteLine("Tool3ViewModel disposed.");
        }
    }
}