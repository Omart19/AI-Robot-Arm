using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using Avalonia.Threading; // Needed for DispatcherTimer and Dispatcher
using Dock.Model.Mvvm.Controls;
using RobotAIArm.Controllers; // Assuming AppSettings and SignalController are accessible

namespace RobotAIArm.ViewModels.Tools
{
    public class Tool2ViewModel : Tool // Consider implementing IDisposable
    {
        // --- Fields ---
        private MagneticEncoderController? _sensorController; // For local mode

        private readonly List<string> angleLabels = new List<string>() {
            "Base Angle:",
            "Lower joint Angle:",
            "Middle joint Angle:",
            "Upper joint Angle:"
        };

       

        // --- Properties ---
        public ObservableCollection<string> EncoderAngles { get; } = new ObservableCollection<string>();

        // --- Constructor ---
        public Tool2ViewModel()
        {
            

            // Initial setup based on current mode
            UpdateSubscriptionBasedOnMode(AppSettings.Instance.IsRemoteMode);

            // Subscribe to future mode changes
            AppSettings.Instance.PropertyChanged += AppSettings_PropertyChanged;

            
        }

        // --- Event Handlers ---
        private void AppSettings_PropertyChanged(object? sender, System.ComponentModel.PropertyChangedEventArgs e)
        {
            if (e.PropertyName == nameof(AppSettings.IsRemoteMode))
            {
                Console.WriteLine($"[Tool2ViewModel] Detected mode change. New Remote Mode: {AppSettings.Instance.IsRemoteMode}");
                UpdateSubscriptionBasedOnMode(AppSettings.Instance.IsRemoteMode);
            }
        }

        // This now ONLY stores the latest data, doesn't touch UI directly. Runs on background thread.
        private async void OnEncodersReceived(int[] encoders) // Changed to async void
        {
            // No longer storing locally - update UI immediately via dispatcher
            // Runs on the background thread that raised the event.
            Console.WriteLine($"[Tool2ViewModel.OnEncodersReceived] Handler Executed with: {(encoders == null ? "NULL" : string.Join(",", encoders))}");

            try
            {
                // Marshal the UI update logic to the UI thread.
                

                    if (encoders.Length == angleLabels.Count)
                    {
                        // Optimization: Update existing items instead of Clear/Add if counts match
                        if (EncoderAngles.Count == encoders.Length)
                        {
                            for (int i = 0; i < encoders.Length; i++)
                            {
                                EncoderAngles[i] = $"{angleLabels[i]} {encoders[i]}";
                            }
                        }
                        else // If count differs (e.g., first time), clear and add
                        {
                            EncoderAngles.Clear();
                            for (int i = 0; i < encoders.Length; i++)
                            {
                                EncoderAngles.Add($"{angleLabels[i]} {encoders[i]}");
                            }
                        }
                    }
                    else
                    {
                        Console.WriteLine($"[Tool2ViewModel WARN] Direct Update: Mismatched encoder count ({encoders.Length}).");
                        EncoderAngles.Clear();
                        EncoderAngles.Add("Error: Invalid encoder data");
                    }
                
            }
            catch (Exception ex)
            {
                // Exceptions in async void are harder to handle gracefully
                Console.WriteLine($"[Tool2ViewModel] Error during direct UI update dispatch: {ex.Message}");
            }
        }


        // This runs periodically on the UI thread via the timer

        // --- Mode Change Logic ---
        private void UpdateSubscriptionBasedOnMode(bool isRemote)
        {
            Console.WriteLine($"[Tool2ViewModel] UpdateSubscriptionBasedOnMode called with isRemote={isRemote}.");

            // Unsubscribe from SignalController event FIRST
            SignalController.Instance.EncodersReceived -= OnEncodersReceived;
            Console.WriteLine("[Tool2ViewModel] Unsubscribed from EncodersReceived.");

            // Stop local reading loop if it was running
            StopLocalEncoderReading();

            // --- Timer stop logic REMOVED ---
            // _updateTimer.Stop();
            // Console.WriteLine("[Tool2ViewModel] UI Update Timer stopped.");

            // --- Local value clearing REMOVED (no longer storing locally) ---
            // lock (_lockObject) { _latestEncoderValues = null; }

            if (isRemote)
            {
                Console.WriteLine("[Tool2ViewModel] Setting up for remote mode (Direct UI Update).");
                // Subscribe to remote signals
                SignalController.Instance.EncodersReceived += OnEncodersReceived;
                Console.WriteLine("[Tool2ViewModel] SUBSCRIBED to EncodersReceived.");
                _sensorController = null;
                // --- Timer start logic REMOVED ---
                // _updateTimer.Start();
                // Console.WriteLine("[Tool2ViewModel] UI Update Timer started.");
            }
            else
            {
                Console.WriteLine("[Tool2ViewModel] Setting up for local mode.");
                StartLocalEncoderReading(); // Start local reading
            }

            // Clear UI on mode change & provide feedback
            EncoderAngles.Clear();
            EncoderAngles.Add($"Mode set to {(isRemote ? "Remote" : "Local")} - Waiting...");
        }
        // --- Local Mode Methods ---
        private CancellationTokenSource? _localReadCts;

        private void StartLocalEncoderReading()
        {
            StopLocalEncoderReading(); // Ensure any previous loop is stopped
            _localReadCts = new CancellationTokenSource();
            var token = _localReadCts.Token;

            _sensorController ??= new MagneticEncoderController(); // Create if null

            Task.Run(async () => { /* ... Same local loop as before ... */
                Console.WriteLine("[Tool2ViewModel] Local reading loop started.");
                while (!token.IsCancellationRequested)
                {
                    try
                    {
                        // Use the existing local read method, which should handle its own UI updates via Dispatcher
                        await ReadAndUpdateEncoderAnglesLocally(token);
                        await Task.Delay(100, token); // Read local sensors periodically
                    }
                    catch (OperationCanceledException)
                    {
                        Console.WriteLine("[Tool2ViewModel] Local reading loop cancelled.");
                        break;
                    }
                    catch (Exception ex)
                    {
                        Console.WriteLine($"[Tool2ViewModel] Error in local reading loop: {ex.Message}");
                        await Task.Delay(1000, token);
                    }
                }
                Console.WriteLine("[Tool2ViewModel] Local reading loop stopped.");
            }, token);
        }

        private void StopLocalEncoderReading()
        {
            if (_localReadCts != null)
            {
                Console.WriteLine("[Tool2ViewModel] Stopping local reading loop...");
                try
                {
                    _localReadCts.Cancel();
                    _localReadCts.Dispose();
                }
                catch (ObjectDisposedException) { /* Ignore if already disposed */ }
                _localReadCts = null;
            }
            // Optional: Dispose sensor controller if it implements IDisposable and is only for local mode
            // _sensorController?.Dispose();
            // _sensorController = null;
        }

        // Keep your existing local reading logic which updates UI via dispatcher
        private async Task ReadAndUpdateEncoderAnglesLocally(CancellationToken token)
        {
            if (_sensorController == null) return;

            List<string> anglesList = new List<string>();
            try
            {
                anglesList.AddRange(_sensorController.ReadDataFromSensors()); // Assuming sync read
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[Tool2ViewModel] Error reading local sensors: {ex.Message}");
                await Dispatcher.UIThread.InvokeAsync(() => {
                    EncoderAngles.Clear();
                    EncoderAngles.Add("Error reading sensors");
                }, DispatcherPriority.Normal, token);
                return;
            }

            // Update the ObservableCollection on the UI thread (already handled in local loop)
            await Dispatcher.UIThread.InvokeAsync(() =>
            {
                if (anglesList.Count == angleLabels.Count)
                {
                    // Apply same optimization here
                    if (EncoderAngles.Count == anglesList.Count)
                    {
                        for (int i = 0; i < anglesList.Count; i++)
                        {
                            EncoderAngles[i] = $"{angleLabels[i]} {anglesList[i]}"; // Assuming local returns values
                        }
                    }
                    else
                    {
                        EncoderAngles.Clear();
                        for (int i = 0; i < anglesList.Count; i++)
                        {
                            EncoderAngles.Add($"{angleLabels[i]} {anglesList[i]}");
                        }
                    }
                }
                else
                {
                    Console.WriteLine($"[Tool2ViewModel WARN] Local read returned {anglesList.Count} values, expected {angleLabels.Count}.");
                    EncoderAngles.Clear();
                    EncoderAngles.Add("Error: Mismatched local sensor data");
                }
            }, DispatcherPriority.Normal, token); // Pass token
        }


        // --- Cleanup ---
        public void Cleanup() // Or implement IDisposable.Dispose
        {
            Console.WriteLine("[Tool2ViewModel] Cleaning up...");
            
            AppSettings.Instance.PropertyChanged -= AppSettings_PropertyChanged;
            SignalController.Instance.EncodersReceived -= OnEncodersReceived;
            StopLocalEncoderReading();
        }
    }
}