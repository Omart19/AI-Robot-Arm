using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text.Json;
using System.Threading;
using System.Threading.Tasks;
using Avalonia.Threading; // Needed for DispatcherTimer and Dispatcher
using CommunityToolkit.Mvvm.ComponentModel;
using Dock.Model.Mvvm.Controls;
using RobotAIArm.Controllers; // Assuming AppSettings and SignalController are accessible

namespace RobotAIArm.ViewModels.Tools
{
    public class EncoderState
    {
        public int PreviousRawAngle { get; set; } = -1;
        public int RotationCount { get; set; } = 0;
        public int CalibratedRawAt90Degrees { get; set; } = -1;
        public bool IsCalibrated { get; set; } = false;
    }
    public class EncoderCalibrationData
    {
        // Using a dictionary to map encoder index (0-3) to its raw calibrated value
        public Dictionary<int, int> CalibratedRawValues { get; set; } = new Dictionary<int, int>();
    }
    public partial class Tool2ViewModel : Tool // Consider implementing IDisposable
    {
        // --- Fields ---
        private MagneticEncoderController? _sensorController; // For local mode

        [ObservableProperty]
        private double? _baseAngle;

        [ObservableProperty]
        private double? _lowerJointAngle;

        [ObservableProperty]
        private double? _middleJointAngle;

        [ObservableProperty]
        private double? _upperJointAngle;

        private int[]? _lastReceivedRawEncoders = null; // Initialize to null or new int[4];
        private readonly object _rawEncoderLock = new object(); // For thread-safe access to _lastReceivedRawEncoders

        private Dictionary<int, EncoderState> _receivedEncoderStates = new Dictionary<int, EncoderState>();
        private const int MAX_RAW_VALUE = 4095;
        private const int HALF_RAW_VALUE = MAX_RAW_VALUE / 2;

        /// <summary>
        /// Indicates if calibration data was successfully loaded from the file at startup.
        /// </summary>
        public bool WasCalibrationLoadedFromFile { get; private set; } = false;
        private bool _initialCalibrationSaveDone = false; // Flag to save only once if multiple encoders calibrate initially

        private static readonly string CalibrationFilePath = Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.ApplicationData),
            "RobotAIArm", // Your application's folder
            "encoder_calibration.json");
        private bool _startupHomingRequestAttempted = false; // Prevents multiple requests

        /// <summary>
        /// Gets the current processed angle for a specific encoder index.
        /// Returns null if the angle is not available or the index is invalid.
        /// </summary>
        /// <param name="encoderIndex">The index of the encoder (0 for Base, 1 for Lower, 2 for Middle, 3 for Upper).</param>
        /// <returns>The processed angle in degrees, or null.</returns>
        public double? GetSpecificAngle(int encoderIndex)
        {
            return encoderIndex switch
            {
                0 => BaseAngle,
                1 => LowerJointAngle,
                2 => MiddleJointAngle,
                3 => UpperJointAngle,
                _ => null // Invalid index
            };
        }

        // --- Constructor ---
        public Tool2ViewModel()
        {
            Console.WriteLine("!!!!!!!!!!!!!!!!!!!! TOOL2VIEWMODEL CONSTRUCTOR IS RUNNING !!!!!!!!!!!!!!!!!!!!"); // <-- ADD THIS
                                                                                                                  // ... rest of your existing constructor code
            Console.WriteLine("*** [Tool2ViewModel] CONSTRUCTOR CALLED ***"); // Keep your existing detailed log too
            for (int i = 0; i < 4; i++)
            {
                _receivedEncoderStates[i] = new EncoderState();
            }
            LoadCalibrationData();

            SignalController.Instance.RobotControllerReadyForHomingCheck += OnRobotControllerReady;

            if (WasCalibrationLoadedFromFile)
            {
                Console.WriteLine("[Tool2ViewModel] Constructor: Calibration was loaded from file. Requesting startup homing.");
                // Use a small delay or Dispatcher to ensure other components might be ready
                // For simplicity here, we raise it directly. If RobotController is not yet subscribed, this might be missed.
                // A slightly more robust way is to ensure RobotController is constructed and subscribed first,
                // or use a small Task.Delay before raising.
                // Let's try direct first.
                Task.Run(async () => {
                    await Task.Delay(500); // Give other components a moment to subscribe
                    Console.WriteLine("[Tool2ViewModel] Delayed Homing Request: Raising StartupHomingRequestedAsync.");
                    SignalController.Instance.RaiseStartupHomingRequested(this);
                });
            }
            else
            {
                Console.WriteLine("[Tool2ViewModel] Constructor: Calibration NOT loaded from file. Initial dynamic calibration will occur.");
                // The existing logic in OnEncodersReceived will handle setting CalibratedRawAt90Degrees
                // from the first encoder values if WasCalibrationLoadedFromFile is false (state.IsCalibrated will be false).
            }

            Console.WriteLine($"*** [Tool2ViewModel] Initial AppSettings.Instance.IsRemoteMode = {AppSettings.Instance.IsRemoteMode} ***");
            UpdateSubscriptionBasedOnMode(AppSettings.Instance.IsRemoteMode);
            AppSettings.Instance.PropertyChanged += AppSettings_PropertyChanged;
            Console.WriteLine("*** [Tool2ViewModel] CONSTRUCTOR FINISHED ***");
        }
        public void CalibrateReceivedEncoderTo90Degrees(int encoderIndex, int currentRawValue)
        {
            if (_receivedEncoderStates.TryGetValue(encoderIndex, out EncoderState? state))
            {
                state.CalibratedRawAt90Degrees = currentRawValue;
                state.RotationCount = 0;
                state.PreviousRawAngle = currentRawValue; // Important for first unwrapping after calibration
                state.IsCalibrated = true;
                Console.WriteLine($"[Tool2ViewModel] Received Encoder {encoderIndex} calibrated: Raw value {currentRawValue} is now 90-deg point.");
            }
        }
        // --- Event Handlers ---
        private void AppSettings_PropertyChanged(object? sender, System.ComponentModel.PropertyChangedEventArgs e)
        {
            if (e.PropertyName == nameof(AppSettings.IsRemoteMode))
            {
                Console.WriteLine($"*** [Tool2ViewModel] AppSettings_PropertyChanged: IsRemoteMode changed to {AppSettings.Instance.IsRemoteMode} ***");
                UpdateSubscriptionBasedOnMode(AppSettings.Instance.IsRemoteMode);
            }
        }

        // This now ONLY stores the latest data, doesn't touch UI directly. Runs on background thread.
        private void OnEncodersReceived(int[]? encoders)
        {
            Console.WriteLine($"!!!!!!!!!!!!!!!!!!!! TOOL2VIEWMODEL OnEncodersReceived CALLED! Encoders: {(encoders == null ? "NULL" : string.Join(",", encoders))} !!!!!!!!!!!!!!!!!!!!");
            bool needsSaveForFirstTimeDynamicCalibration = false;

            if (encoders != null && encoders.Length == 4)
            {
                lock (_rawEncoderLock)
                {
                    if (_lastReceivedRawEncoders == null || _lastReceivedRawEncoders.Length != 4)
                    {
                        _lastReceivedRawEncoders = new int[4];
                    }
                    Array.Copy(encoders, _lastReceivedRawEncoders, 4);
                }

                for (int i = 0; i < 4; i++)
                {
                    if (_receivedEncoderStates.TryGetValue(i, out EncoderState? state))
                    {
                        // If WasCalibrationLoadedFromFile is false, state.IsCalibrated would also be false here initially.
                        // This block then sets the *very first* encoder reading as the calibration point.
                        if (!state.IsCalibrated)
                        {
                            Console.WriteLine($"[Tool2ViewModel] OnEncodersReceived: Encoder {i} NOT calibrated. Setting first value ({encoders[i]}) as CalibratedRawAt90Degrees.");
                            state.CalibratedRawAt90Degrees = encoders[i];
                            state.IsCalibrated = true; // Now it's calibrated (dynamically)
                            state.PreviousRawAngle = encoders[i];
                            state.RotationCount = 0;
                            needsSaveForFirstTimeDynamicCalibration = true;
                        }
                    }
                }

                // If a new dynamic calibration happened AND we haven't done an initial save yet (either from file or this dynamic one)
                if (needsSaveForFirstTimeDynamicCalibration && !_initialCalibrationSaveDone)
                {
                    Console.WriteLine("[Tool2ViewModel] OnEncodersReceived: Saving newly established dynamic calibration data.");
                    SaveCalibrationData();
                    _initialCalibrationSaveDone = true; // Mark that initial calibration (dynamic or from file) is now done and saved if new.
                }

                double pBase = ProcessSingleReceivedEncoder(0, encoders[0]);
                double pLower = ProcessSingleReceivedEncoder(1, encoders[1]);
                double pMiddle = ProcessSingleReceivedEncoder(2, encoders[2]);
                double pUpper = ProcessSingleReceivedEncoder(3, encoders[3]);

                Dispatcher.UIThread.Post(() =>
                {
                    BaseAngle = pBase;
                    LowerJointAngle = pLower;
                    MiddleJointAngle = pMiddle;
                    UpperJointAngle = pUpper;
                    Console.WriteLine($"[Tool2ViewModel UI UPDATE] Base: {BaseAngle:F2}, Lower: {LowerJointAngle:F2}, Middle: {MiddleJointAngle:F2}, Upper: {UpperJointAngle:F2}");
                });
            }
            else
            {
                Console.WriteLine($"[Tool2ViewModel WARN OnEncodersReceived] Encoders array null or invalid length. IsNull: {encoders == null}, Length: {(encoders?.Length.ToString() ?? "N/A")}");
                Dispatcher.UIThread.Post(() => { BaseAngle = null; LowerJointAngle = null; MiddleJointAngle = null; UpperJointAngle = null; });
            }
        }

        private void OnRobotControllerReady()
        {
            Console.WriteLine("[Tool2ViewModel] Received RobotControllerReadyForHomingCheck event.");
            // Unsubscribe immediately to prevent multiple calls if RobotController were to raise its event more than once (it shouldn't).
            SignalController.Instance.RobotControllerReadyForHomingCheck -= OnRobotControllerReady;

            if (WasCalibrationLoadedFromFile && !_startupHomingRequestAttempted)
            {
                _startupHomingRequestAttempted = true; // Ensure we only try this once
                Console.WriteLine("[Tool2ViewModel] RobotController is ready AND calibration was loaded. Requesting startup homing NOW.");
                SignalController.Instance.RaiseStartupHomingRequested(this);
            }
            else if (!WasCalibrationLoadedFromFile)
            {
                Console.WriteLine("[Tool2ViewModel] RobotController is ready, but no calibration was loaded from file. Homing still skipped.");
            }
            else if (_startupHomingRequestAttempted)
            {
                Console.WriteLine("[Tool2ViewModel] RobotController is ready, but startup homing request was already attempted/made.");
            }
        }
        // This runs periodically on the UI thread via the timer
        private void LoadCalibrationData()
        {
            WasCalibrationLoadedFromFile = false;
            try
            {
                if (File.Exists(CalibrationFilePath))
                {
                    Console.WriteLine($"[Tool2ViewModel] LoadCalibrationData: Attempting to load from: {CalibrationFilePath}");
                    string json = File.ReadAllText(CalibrationFilePath);
                    EncoderCalibrationData? loadedData = JsonSerializer.Deserialize<EncoderCalibrationData>(json);

                    if (loadedData?.CalibratedRawValues != null && loadedData.CalibratedRawValues.Any())
                    {
                        int successfullyAppliedValues = 0;
                        foreach (var pair in loadedData.CalibratedRawValues)
                        {
                            if (pair.Key >= 0 && pair.Key < 4 && _receivedEncoderStates.TryGetValue(pair.Key, out EncoderState? state))
                            {
                                state.CalibratedRawAt90Degrees = pair.Value;
                                state.IsCalibrated = true;
                                state.PreviousRawAngle = -1;
                                state.RotationCount = 0;
                                Console.WriteLine($"[Tool2ViewModel] LoadCalibrationData: Loaded & Applied for Encoder {pair.Key}: RawAt90 = {pair.Value}");
                                successfullyAppliedValues++;
                            }
                            else
                            {
                                Console.WriteLine($"[Tool2ViewModel] LoadCalibrationData: Warning - Invalid key {pair.Key} in calibration file or state not found.");
                            }
                        }

                        if (successfullyAppliedValues > 0 && successfullyAppliedValues == loadedData.CalibratedRawValues.Count && successfullyAppliedValues == 4) // Expect all 4 encoders
                        {
                            Console.WriteLine($"[Tool2ViewModel] LoadCalibrationData: ALL {successfullyAppliedValues} calibration values successfully loaded and applied from file.");
                            WasCalibrationLoadedFromFile = true; // CRITICAL: Set the flag
                            _initialCalibrationSaveDone = true;
                        }
                        else
                        {
                            Console.WriteLine($"[Tool2ViewModel] LoadCalibrationData: Calibration file parsed, but not all values were applied (Applied: {successfullyAppliedValues}, InFile: {loadedData.CalibratedRawValues.Count}, Expected 4). Treating as not fully calibrated from file.");
                        }
                    }
                    else
                    {
                        Console.WriteLine("[Tool2ViewModel] LoadCalibrationData: Calibration file was present but loadedData or CalibratedRawValues was null/empty.");
                    }
                }
                else
                {
                    Console.WriteLine($"[Tool2ViewModel] LoadCalibrationData: Calibration file not found: {CalibrationFilePath}.");
                }
            }
            catch (JsonException jsonEx)
            {
                Console.WriteLine($"[Tool2ViewModel] LoadCalibrationData: JSON Error loading calibration data: {jsonEx.Message}. File might be corrupt.");
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[Tool2ViewModel] LoadCalibrationData: General Error loading calibration data: {ex.Message}");
            }
            Console.WriteLine($"[Tool2ViewModel] LoadCalibrationData: Final status WasCalibrationLoadedFromFile = {WasCalibrationLoadedFromFile}");
        }
        private void SaveCalibrationData()
        {
            try
            {
                EncoderCalibrationData dataToSave = new EncoderCalibrationData();
                bool hasDataToSave = false;
                for (int i = 0; i < 4; i++) // Assuming 4 encoders
                {
                    if (_receivedEncoderStates.TryGetValue(i, out EncoderState? state) && state.IsCalibrated)
                    {
                        dataToSave.CalibratedRawValues[i] = state.CalibratedRawAt90Degrees;
                        hasDataToSave = true;
                    }
                }

                if (hasDataToSave)
                {
                    string directoryPath = Path.GetDirectoryName(CalibrationFilePath) ?? "";
                    if (!string.IsNullOrEmpty(directoryPath) && !Directory.Exists(directoryPath))
                    {
                        Directory.CreateDirectory(directoryPath);
                        Console.WriteLine($"[Tool2ViewModel] Created directory for calibration file: {directoryPath}");
                    }

                    string json = JsonSerializer.Serialize(dataToSave, new JsonSerializerOptions { WriteIndented = true });
                    File.WriteAllText(CalibrationFilePath, json);
                    Console.WriteLine($"[Tool2ViewModel] Calibration data saved to: {CalibrationFilePath}");
                }
                else
                {
                    Console.WriteLine("[Tool2ViewModel] No calibrated encoder data to save.");
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[Tool2ViewModel] Error saving calibration data: {ex.Message}");
            }
        }

        // --- Mode Change Logic ---
        private void UpdateSubscriptionBasedOnMode(bool isRemote)
        {
            Console.WriteLine($"*** [Tool2ViewModel] UpdateSubscriptionBasedOnMode called with isRemote={isRemote}. ***");
            SignalController.Instance.EncodersReceived -= OnEncodersReceived;
            StopLocalEncoderReading(); // Handles local mode logic

            // Reset properties when mode changes, especially if switching away from a mode that was providing data
            Dispatcher.UIThread.Post(() => {
                BaseAngle = null; LowerJointAngle = null; MiddleJointAngle = null; UpperJointAngle = null;
            });


            if (isRemote)
            {
                Console.WriteLine("*** [Tool2ViewModel] Setting up for REMOTE mode. ***");
                SignalController.Instance.EncodersReceived += OnEncodersReceived;
            }
            else
            {
                // LOCAL MODE (currently affected by I2C TargetFramework issue on Windows)
                Console.WriteLine("*** [Tool2ViewModel] Setting up for LOCAL mode. Attempting StartLocalEncoderReading(). ***");
                StartLocalEncoderReading();
            }
        }
        // --- Local Mode Methods ---
        private CancellationTokenSource? _localReadCts;

        private void StartLocalEncoderReading()
        {
            Console.WriteLine("*** [Tool2ViewModel] StartLocalEncoderReading() CALLED. ***");
            // ... (existing logic to create _sensorController, but it will fail on I2C.Create currently) ...
            // If it were to work, the Task.Run loop would call a method that gets raw values,
            // then calls ProcessSingleReceivedEncoder for each, similar to OnEncodersReceived.
            StopLocalEncoderReading();
            _localReadCts = new CancellationTokenSource();
            var token = _localReadCts.Token;

            try
            {
                Console.WriteLine("*** [Tool2ViewModel] Attempting to create MagneticEncoderController instance... ***");
                _sensorController ??= new MagneticEncoderController(); // This will likely log an I2C error due to target framework
                Console.WriteLine("*** [Tool2ViewModel] MagneticEncoderController instance potentially created. Null? " + (_sensorController == null) + " ***");
            }
            catch (Exception ex)
            {
                Console.WriteLine($"*** [Tool2ViewModel] FAILED to create MagneticEncoderController: {ex.ToString()} ***");
                return; // Don't start the task if controller failed
            }

            if (_sensorController == null)
            {
                Console.WriteLine("*** [Tool2ViewModel] _sensorController is NULL after creation attempt. Cannot start local reading task. ***");
                return;
            }

            Task.Run(async () => {
                Console.WriteLine("*** [Tool2ViewModel] LOCAL READING TASK STARTED. ***");
                while (!token.IsCancellationRequested)
                {
                    // Console.WriteLine("*** [Tool2ViewModel] LOCAL READING TASK: Top of while loop. ***");
                    try
                    {
                        // This is where you would call a method in _sensorController
                        // that returns raw int[] or List<int>, then process them.
                        // For now, this part is effectively non-functional due to the I2C issue.
                        // Example if _sensorController.ReadRawIntEncoders() existed:
                        // int[] rawVals = _sensorController.ReadRawIntEncoders();
                        // double pLocalBase = ProcessSingleReceivedEncoder(0, rawVals[0]); ...
                        // Dispatcher.UIThread.Post(() => { BaseAngle = pLocalBase; ... });
                        await Task.Delay(100, token); // Placeholder
                    }
                    catch (OperationCanceledException) { Console.WriteLine("*** [Tool2ViewModel] LOCAL READING TASK: OperationCanceledException. ***"); break; }
                    catch (Exception ex) { Console.WriteLine($"*** [Tool2ViewModel] LOCAL READING TASK: Error in loop: {ex.ToString()} ***"); await Task.Delay(1000, token); }
                }
                Console.WriteLine("*** [Tool2ViewModel] LOCAL READING TASK STOPPED. ***");
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
        }


        // Keep your existing local reading logic which updates UI via dispatcher
        private async Task ReadAndUpdateEncoderAnglesLocally(CancellationToken token)
        {
            if (_sensorController == null)
            {
                Console.WriteLine("[Tool2ViewModel] Sensor controller is null in ReadAndUpdateEncoderAnglesLocally.");
                // Optionally set angles to null/error state on UI thread
                await Dispatcher.UIThread.InvokeAsync(() => {
                    BaseAngle = null; LowerJointAngle = null; MiddleJointAngle = null; UpperJointAngle = null;
                }, DispatcherPriority.Normal, token);
                return;
            }
            Console.WriteLine("[Tool2ViewModel] Sensor controller is NOT NULL."); // DEBUG

            List<double> processedAngles; // This will now be List<double>

            try
            {
                Console.WriteLine("[Tool2ViewModel] Attempting to call _sensorController.ReadDataFromSensors()."); // DEBUG

                // Call the method that returns List<double>
                processedAngles = _sensorController.ReadDataFromSensors();
                Console.WriteLine($"[Tool2ViewModel LocalRead] Processed Angles: {string.Join(", ", processedAngles)}"); // DEBUG

                // Ensure you still handle the UI update on the UI thread
                if (token.IsCancellationRequested)
                {
                    Console.WriteLine("[Tool2ViewModel] Cancellation requested after reading sensors."); // DEBUG
                    return;
                }
                Console.WriteLine("[Tool2ViewModel] Attempting to dispatch UI update."); // DEBUG

                await Dispatcher.UIThread.InvokeAsync(() =>
                {
                    if (processedAngles.Count == 4)
                    {
                        BaseAngle = processedAngles[0];
                        LowerJointAngle = processedAngles[1];
                        MiddleJointAngle = processedAngles[2];
                        UpperJointAngle = processedAngles[3];
                    }
                    else
                    {
                        Console.WriteLine($"[Tool2ViewModel WARN] Local read returned {processedAngles.Count} double values, expected 4.");
                        BaseAngle = null;
                        LowerJointAngle = null;
                        MiddleJointAngle = null;
                        UpperJointAngle = null;
                    }
                }, DispatcherPriority.Normal, token);
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[Tool2ViewModel] CRITICAL Error in ReadAndUpdateEncoderAnglesLocally's try block: {ex.ToString()}"); // DEBUG - ToString for more detail
                if (token.IsCancellationRequested) return;
                try
                {
                    await Dispatcher.UIThread.InvokeAsync(() => {
                        BaseAngle = null; LowerJointAngle = null; MiddleJointAngle = null; UpperJointAngle = null;
                    }, DispatcherPriority.Normal, token);
                }
                catch (Exception dispatchEx)
                {
                    Console.WriteLine($"[Tool2ViewModel] CRITICAL Error dispatching nulls after main error: {dispatchEx.ToString()}"); // DEBUG
                }
            }
        }

        public Dictionary<int, EncoderState> GetEncoderStatesForHoming()
        {
            // Return a new dictionary containing the same references.
            // If RobotController is only reading these states, this is fine.
            // If there's a risk of modification, a more robust deep copy or ReadOnlyDictionary might be considered.
            lock (_receivedEncoderStates) // Assuming _receivedEncoderStates might be modified elsewhere, though less likely for the dictionary structure itself
            {
                return new Dictionary<int, EncoderState>(_receivedEncoderStates);
            }
        }

        /// <summary>
        /// Gets a copy of the last received raw encoder values.
        /// Returns null if no values have been received yet.
        /// </summary>
        public int[]? GetLastRawEncoderValues()
        {
            lock (_rawEncoderLock)
            {
                if (_lastReceivedRawEncoders == null)
                {
                    return null;
                }
                // Return a copy to prevent external modification of the internal array
                return (int[])_lastReceivedRawEncoders.Clone();
            }
        }

        private double ProcessSingleReceivedEncoder(int encoderIndex, int newRawAngle)
        {
            if (!_receivedEncoderStates.TryGetValue(encoderIndex, out EncoderState? state))
            {
                Console.WriteLine($"[Tool2ViewModel] CRITICAL: No state for received encoder index {encoderIndex} in ProcessSingle.");
                return (newRawAngle / (double)MAX_RAW_VALUE) * 360.0;
            }

            // Console.WriteLine($"[Tool2ViewModel DEBUG ProcessSingle] Encoder {encoderIndex}: RawIn={newRawAngle}, Calibrated={state.IsCalibrated}, PrevRaw={state.PreviousRawAngle}, RotCount={state.RotationCount}, CalibRawAt90={state.CalibratedRawAt90Degrees}");

            if (!state.IsCalibrated || state.CalibratedRawAt90Degrees == -1) // Ensure it has a valid calibration value
            {
                // Console.WriteLine($"[Tool2ViewModel] Info: Received Encoder {encoderIndex} not truly calibrated (CalibratedRawAt90Degrees is -1 or IsCalibrated is false). Returning raw 0-360deg conversion.");
                return (newRawAngle / (double)MAX_RAW_VALUE) * 360.0;
            }

            if (state.PreviousRawAngle == -1) // Initialize PreviousRawAngle if it's the first run after calibration value is set
            {
                state.PreviousRawAngle = newRawAngle;
            }

            int deltaRaw = newRawAngle - state.PreviousRawAngle;
            if (state.PreviousRawAngle != -1) // Avoid unwrap logic on very first data point if prevAngle was truly -1
            {
                if (deltaRaw > HALF_RAW_VALUE) { state.RotationCount--; }
                else if (deltaRaw < -HALF_RAW_VALUE) { state.RotationCount++; }
            }
            state.PreviousRawAngle = newRawAngle;

            long continuousRawValue = (long)newRawAngle + ((long)state.RotationCount * (MAX_RAW_VALUE + 1));
            double calibratedPointAsDegrees = (state.CalibratedRawAt90Degrees / (double)MAX_RAW_VALUE) * 360.0;
            double currentContinuousDegrees = (continuousRawValue / (double)MAX_RAW_VALUE) * 360.0;
            double finalAngle = currentContinuousDegrees - calibratedPointAsDegrees + 90.0;

            // Console.WriteLine($"[Tool2ViewModel DEBUG ProcessSingle] Encoder {encoderIndex}: FinalAngle={finalAngle:F2}");
            return finalAngle;
        }
        // --- Cleanup ---
        public void Cleanup()
        {
            Console.WriteLine("[Tool2ViewModel] Cleaning up...");
            AppSettings.Instance.PropertyChanged -= AppSettings_PropertyChanged;
            SignalController.Instance.EncodersReceived -= OnEncodersReceived;
            StopLocalEncoderReading();
            // Potentially save calibration on graceful shutdown too, though auto-save on first calib is main request
            // SaveCalibrationData(); 
        }
    }
}