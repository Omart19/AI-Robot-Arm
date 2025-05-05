using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Globalization;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using Avalonia.Threading; // Needed for DispatcherTimer and Dispatcher
using CommunityToolkit.Mvvm.ComponentModel;
using Dock.Model.Mvvm.Controls;
using RobotAIArm.Controllers; // Assuming AppSettings and SignalController are accessible

namespace RobotAIArm.ViewModels.Tools
{
    public partial class Tool2ViewModel : Tool // Consider implementing IDisposable
    {
        // --- Fields ---
        private MagneticEncoderController? _sensorController; // For local mode

        [ObservableProperty]
        private int? _baseAngle; // Initialize to null (will show as empty initially)

        [ObservableProperty]
        private int? _lowerJointAngle;

        [ObservableProperty]
        private int? _middleJointAngle;

        [ObservableProperty]
        private int? _upperJointAngle;



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
        private void OnEncodersReceived(int[]? encoders) // Make parameter nullable for check
        {
            // --- ADD THIS LOGGING ---
            Console.WriteLine($"[Tool2ViewModel.OnEncodersReceived] Handler Executed with: {(encoders == null ? "NULL" : string.Join(",", encoders))}");
            // -----------------------

            // --- Add Null Check ---
            //if (encoders == null)
            //{
            //    Console.WriteLine("[Tool2ViewModel.OnEncodersReceived] Received null encoders.");
            //    // Reset properties to null if desired
            //    BaseAngle = null;
            //    LowerJointAngle = null;
            //    MiddleJointAngle = null;
            //    UpperJointAngle = null;
            //    return;
            //}
            // ---------------------

            //try
            //{
                if (encoders.Length == 4) // Check if we have exactly 4 values
                {
                    // --- Update individual numeric properties ---
                    BaseAngle = encoders[0];
                    LowerJointAngle = encoders[1];
                    MiddleJointAngle = encoders[2];
                    UpperJointAngle = encoders[3];
                    // --- End Update ---
                }
                else
                {
                    Console.WriteLine($"[Tool2ViewModel WARN] Received mismatched encoder count ({encoders.Length}). Expected 4");
                    // Set properties to null or an error indicator if desired
                    BaseAngle = null;
                    LowerJointAngle = null;
                    MiddleJointAngle = null;
                    UpperJointAngle = null;
                }
            //}
            //catch (Exception ex)
            //{
            //    Console.WriteLine($"[Tool2ViewModel.OnEncodersReceived] Error updating properties: {ex.Message}");
            //    // Optionally set properties to null or an error state on exception
            //    BaseAngle = null;
            //    LowerJointAngle = null;
            //    MiddleJointAngle = null;
            //    UpperJointAngle = null;
            //}
        }

        // This runs periodically on the UI thread via the timer

        // --- Mode Change Logic ---
        private void UpdateSubscriptionBasedOnMode(bool isRemote)
        {
            Console.WriteLine($"[Tool2ViewModel] UpdateSubscriptionBasedOnMode called with isRemote={isRemote}.");

            SignalController.Instance.EncodersReceived -= OnEncodersReceived; // Unsubscribe first
            StopLocalEncoderReading(); // Stop local if running

            // Reset properties on mode change
            BaseAngle = null;
            LowerJointAngle = null;
            MiddleJointAngle = null;
            UpperJointAngle = null;

            if (isRemote)
            {
                Console.WriteLine("[Tool2ViewModel] Setting up for remote mode.");
                SignalController.Instance.EncodersReceived += OnEncodersReceived; // Subscribe
                Console.WriteLine("[Tool2ViewModel] SUBSCRIBED to EncodersReceived.");
                _sensorController = null;
                // Properties already reset above
            }
            else
            {
                Console.WriteLine("[Tool2ViewModel] Setting up for local mode.");
                // Properties already reset above
                StartLocalEncoderReading(); // Start local reading
            }
        }
        // --- Local Mode Methods ---
        private CancellationTokenSource? _localReadCts;

        private void StartLocalEncoderReading()
        {
            StopLocalEncoderReading();
            _localReadCts = new CancellationTokenSource();
            var token = _localReadCts.Token;
            // Ensure controller is created only if needed and not already disposed
            try
            {
                _sensorController ??= new MagneticEncoderController();
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[Tool2ViewModel] Failed to create MagneticEncoderController: {ex.Message}");
                // Set properties to an error state?
                BaseAngle = null; LowerJointAngle = null; MiddleJointAngle = null; UpperJointAngle = null;
                return; // Don't start the task if controller failed
            }


            Task.Run(async () => {
                Console.WriteLine("[Tool2ViewModel] Local reading loop started.");
                while (!token.IsCancellationRequested)
                {
                    try { await ReadAndUpdateEncoderAnglesLocally(token); await Task.Delay(100, token); }
                    catch (OperationCanceledException) { Console.WriteLine("[Tool2ViewModel] Local reading loop cancelled."); break; }
                    catch (Exception ex) { Console.WriteLine($"[Tool2ViewModel] Error in local reading loop: {ex.Message}"); await Task.Delay(1000, token); }
                }
                Console.WriteLine("[Tool2ViewModel] Local reading loop stopped.");
            }, token);
        }
        private void StopLocalEncoderReading()
        {
            if (_localReadCts != null)
            {
                Console.WriteLine("[Tool2ViewModel] Stopping local reading loop...");
                try { _localReadCts.Cancel(); _localReadCts.Dispose(); }
                catch (ObjectDisposedException) { /* Ignore */ }
                _localReadCts = null;
            }
            // Dispose sensor controller if needed and it implements IDisposable
            // (_sensorController as IDisposable)?.Dispose();
            // _sensorController = null; // Maybe keep if needed again? Depends on lifecycle.
        }

        // Keep your existing local reading logic which updates UI via dispatcher
        private async Task ReadAndUpdateEncoderAnglesLocally(CancellationToken token)
        {
            if (_sensorController == null) return;

            List<string> anglesListRaw = new List<string>();
            int?[] parsedValues = new int?[4]; // Array to hold parsed int? values

            try
            {
                // Assuming ReadDataFromSensors returns a List<string> of the numeric values
                anglesListRaw.AddRange(_sensorController.ReadDataFromSensors());

                if (anglesListRaw.Count == 4)
                {
                    for (int i = 0; i < 4; i++)
                    {
                        if (int.TryParse(anglesListRaw[i], NumberStyles.Any, CultureInfo.InvariantCulture, out int val))
                        {
                            parsedValues[i] = val;
                        }
                        else
                        {
                            parsedValues[i] = null; // Parsing failed
                            Console.WriteLine($"[Tool2ViewModel WARN] Local read parse failed for value: {anglesListRaw[i]}");
                        }
                    }
                }
                else
                {
                    Console.WriteLine($"[Tool2ViewModel WARN] Local read returned {anglesListRaw.Count} values, expected 4.");
                    // Leave parsedValues as nulls
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[Tool2ViewModel] Error reading/parsing local sensors: {ex.Message}");
                // Update properties to show error state on UI thread
                if (token.IsCancellationRequested) return;
                await Dispatcher.UIThread.InvokeAsync(() => {
                    BaseAngle = null; LowerJointAngle = null; MiddleJointAngle = null; UpperJointAngle = null;
                }, DispatcherPriority.Normal, token);
                return;
            }

            // Update properties on the UI thread
            if (token.IsCancellationRequested) return;

            await Dispatcher.UIThread.InvokeAsync(() =>
            {
                // Update individual properties using the parsed values
                BaseAngle = parsedValues[0];
                LowerJointAngle = parsedValues[1];
                MiddleJointAngle = parsedValues[2];
                UpperJointAngle = parsedValues[3];

            }, DispatcherPriority.Normal, token);
        }

        // --- Cleanup ---
        public void Cleanup() // Or implement IDisposable.Dispose
        {
            Console.WriteLine("[Tool2ViewModel] Cleaning up...");
            AppSettings.Instance.PropertyChanged -= AppSettings_PropertyChanged;
            SignalController.Instance.EncodersReceived -= OnEncodersReceived;
            StopLocalEncoderReading();
            // Dispose sensor controller if needed
            // (_sensorController as IDisposable)?.Dispose();
        }
    }
}