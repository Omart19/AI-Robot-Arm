using Avalonia.Controls;
using Avalonia.Media.Imaging;
using Avalonia.Threading;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing.Imaging;
using System.IO;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Reflection.Emit;
using System.Text;
using System.Text.Json;
using System.Text.Json.Serialization;
using System.Threading;
using System.Threading.Channels;
using System.Threading.Tasks;

namespace RobotAIArm.Controllers
{
    // Implement IAsyncDisposable for proper async cleanup
    internal record FrameDataPacket(byte[] JpegBytes, int[]? EncoderValues, int? TofHand, int? TofCam);
    // --- NEW: Structure for a single detection result ---
    public class DetectionResult
    {
        [JsonPropertyName("label")] // Maps to Python's "label"
        public string Label { get; set; } = "";

        [JsonPropertyName("confidence")] // Maps to Python's "confidence"
        public float Confidence { get; set; }

        [JsonPropertyName("box")] // Maps to Python's "box"
        public List<int> Box { get; set; } = new List<int>(); // x1, y1, x2, y2
    }

    // --- Updated: ProcessedData to include detections ---
    internal record ProcessedData(
        Bitmap? ProcessedBitmap,
        int[]? EncoderValues,
        List<DetectionResult>? Detections,
    int? TofHandValue, // Added
    int? TofCamValue
    );

    public class CameraController : IDisposable, IAsyncDisposable
    {
        // --- UI and Core Logic Fields ---
        private readonly Image _cameraImage; // Reference to the UI Image control
        private readonly ArduinoController _arduino;
        private Process? _visionProcess;
        private CancellationTokenSource? _cts;
        private bool _disposed = false;

        
        



        // --- Network Fields ---
        private TcpClient? _client;
        private NetworkStream? _networkStream;
        private TcpClient? _visionClient;
        private NetworkStream? visionStream;
        
        

        // --- Channel for decoupling Receive from Processing ---
        private readonly Channel<FrameDataPacket> _processingChannel = Channel.CreateBounded<FrameDataPacket>(new BoundedChannelOptions(5) // Buffer size
        {
            FullMode = BoundedChannelFullMode.DropOldest
        });

        // --- Fields for Throttled UI Update ---
        private readonly object _latestResultLock = new object();
        private ProcessedData? _latestProcessedData = null; // Stores combined result
        private bool _newResultAvailable = false;          // Flag for the timer
        private DispatcherTimer? _uiUpdateTimer;           // Timer for UI updates
                                                           // In CameraController class fields:
        private Bitmap? _bitmapCurrentlyDisplayed = null; // Track bitmap assigned to UI
        private RobotController? _robotController;

        public CameraController(Image cameraImage, ArduinoController arduino)
        {
            _cameraImage = cameraImage;
            _arduino = arduino;
            new RobotController(arduino);
        }

        
        private async Task StopCameraFeedAsync()
        {
            // Only proceed if not already disposed
            if (_disposed) return;

            Console.WriteLine("[CameraController] Stopping camera feed (Full Stop)...");
            try
            {
                // 1. Signal cancellation to loops (if CTS exists and not already requested)
                if (_cts != null && !_cts.IsCancellationRequested)
                {
                    try
                    {
                        _cts.Cancel();
                        Console.WriteLine("[CameraController] Cancellation token signaled.");
                    }
                    catch (ObjectDisposedException)
                    {
                        Console.WriteLine("[CameraController] Cancellation token source already disposed.");
                    }
                }
                // Don't dispose _cts here yet, Dispose/DisposeAsync handles that

                // 2. Close network streams and clients using the helper
                CleanupNetworkResources();

                // 3. Terminate Python process (Keep existing logic with correction)
                if (_visionProcess != null && !_visionProcess.HasExited)
                {
                    // ... (Keep the corrected process termination logic from previous answer) ...
                    try
                    {
                        Console.WriteLine($"[CameraController] Killing Vision process (PID: {_visionProcess.Id})...");
                        _visionProcess.Kill(true);
                        Console.WriteLine("[CameraController] Waiting for vision process exit (max 5s)...");
                        using var exitCts = new CancellationTokenSource(TimeSpan.FromSeconds(5));
                        try { await _visionProcess.WaitForExitAsync(exitCts.Token); Console.WriteLine("[CameraController] Vision process exited gracefully after kill signal."); }
                        catch (OperationCanceledException) { Console.WriteLine("[CameraController] Timeout expired waiting for vision process exit."); }
                    }
                    catch (InvalidOperationException ioEx) { Console.WriteLine($"[CameraController] Info: Error managing vision process (may have already exited): {ioEx.Message}"); }
                    catch (Exception procEx) { Console.WriteLine($"[CameraController] Error killing/waiting for vision process: {procEx.Message}"); }
                    finally { try { _visionProcess.Dispose(); } catch { } _visionProcess = null; } // Ensure dispose happens
                }
                else
                {
                    try { _visionProcess?.Dispose(); } catch { } // Dispose if handle exists but process exited
                    _visionProcess = null;
                }

                // 4. Clear the processing channel (Optional, helps if producer finished mid-write)
                while (_processingChannel.Reader.TryRead(out _)) { }

                Console.WriteLine("[CameraController] Camera feed stopped (Full Stop).");
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[CameraController] Error during full stop: {ex.Message}");
            }
            //_arduino.StopArduinoThread(); // Ensure Arduino thread is stopped

        }

        

        public async Task SetModeAsync(bool isRemote)
        {
            Console.WriteLine($"[CameraController] SetModeAsync called. Remote={isRemote}");

            await StopCameraFeedAsync(); // Stop previous state first (this will also stop the timer)

            if (isRemote)
            {
                Console.WriteLine("[CameraController] SetModeAsync: Setting up Remote mode...");
                StartVisionPipelineRemote();
                _arduino.StartArduinoThread(); // Start Arduino thread if not already running
                _ = ConnectToPiServerAsync(); // Fire-and-forget connection loop task
            }
            else
            {
                Console.WriteLine("[CameraController] SetModeAsync: Setting up Local mode...");
                StartVisionPipelineLocal(); // Placeholder
                // StopCameraFeedAsync already cleaned up remote connections and timer
            }
        }

        

        private void StartVisionPipelineLocal()
        {
            Console.WriteLine("[INFO] Local camera mode not implemented.");
            // If you have a local camera implementation, start it here.
        }

        private void StartVisionPipelineRemote()
        {
            Console.WriteLine("[StartVisionPipelineRemote] Method Entered."); // ADD THIS

            // Ensure any previous process is stopped before starting a new one
            if (_visionProcess != null && !_visionProcess.HasExited)
            {
                Console.WriteLine("[StartVisionPipelineRemote] Found existing vision process. Attempting to kill...");
                try
                {
                    _visionProcess.Kill(true);
                    _visionProcess.WaitForExit(1000); // Brief sync wait
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"[StartVisionPipelineRemote] Error killing existing process: {ex.Message}");
                }
                finally
                {
                    _visionProcess.Dispose();
                    _visionProcess = null;
                }
            }
            else
            {
                _visionProcess?.Dispose(); // Dispose if handle exists but process exited
                _visionProcess = null;
            }

            Console.WriteLine("[StartVisionPipelineRemote] Starting local Visionprocessor.py...");
            try
            {
                // Consider making the Python path more robust (e.g., relative to executable or configurable)
                var psi = new ProcessStartInfo
                {
                    FileName = "python", // Ensure python is in PATH
                    Arguments = $"\"Visionprocessor.py\"", // Quote arguments
                    UseShellExecute = false,
                    RedirectStandardOutput = true, // Important for potential debugging
                    RedirectStandardError = true,  // Important for seeing errors
                    CreateNoWindow = true,
                    WorkingDirectory = Path.GetFullPath("..\\..\\..\\PythonScripts") // Set working directory
                };

                _visionProcess = Process.Start(psi);

                if (_visionProcess != null)
                {
                    // Optionally, read output/error asynchronously for debugging
                    _visionProcess.OutputDataReceived += (sender, args) => { if (args.Data != null) Console.WriteLine($"[Vision.py OUT] {args.Data}"); };
                    _visionProcess.ErrorDataReceived += (sender, args) => { if (args.Data != null) Console.WriteLine($"[Vision.py ERR] {args.Data}"); };
                    _visionProcess.BeginOutputReadLine();
                    _visionProcess.BeginErrorReadLine();

                    Console.WriteLine($"[INFO] Started Vision.py on Windows (PID: {_visionProcess.Id}).");
                }
                else
                {
                    Console.WriteLine("[ERROR] Vision pipeline process failed to start (Process.Start returned null).");
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[ERROR] Vision pipeline start failed: {ex.Message}");
                _visionProcess = null; // Ensure it's null on failure
            }
        }

        // Core connection logic, starts loops, handles retries
        private async Task ConnectToPiServerAsync()
        {
            Console.WriteLine("[ConnectToPiServerAsync] Method Entered.");
            string serverIp = AppSettings.Instance.RemoteIP;
            if (string.IsNullOrEmpty(serverIp)) { /* ... error handling ... */ return; }
            string pythonIp = "127.0.0.1";
            int pythonPort = 34567;
            int cameraPort = 23456;
            int commandPort = 23457;

            // Dispose previous CTS if any and create a new one for this connection lifecycle
            _cts?.Dispose();
            _cts = new CancellationTokenSource();
            var token = _cts.Token;

            Console.WriteLine($"[Connect] Starting connection attempts cycle...");

            while (!token.IsCancellationRequested)
            {
                bool connectionAttemptSuccess = false; // Track if current attempt connected fully
                try
                {
                    // --- Connect to Vision.py, Pi Camera, Pi Command ---
                    Console.WriteLine($"[Connect] Attempting -> Vision.py ..."); await ConnectVisionAsync(pythonIp, 34567, token);
                    Console.WriteLine($"[Connect] Attempting -> Pi Camera ..."); await ConnectPiCameraAsync(serverIp, 23456, token);
                    //Console.WriteLine($"[Connect] Attempting -> Pi Command ..."); await ConnectPiCommandAsync(serverIp, 23457, token);
                    connectionAttemptSuccess = true;

                    // --- Start Background Loops ---
                    Console.WriteLine("[Connect] Starting Receive and Processing loops...");
                    Task receiveTask = ReceiveLoopAsync(token);       // Task 1 (Producer)
                    Task processingTask = ProcessingLoopAsync(token); // Task 2 & 3 Orchestrator (Consumer)

                    // --- Start the UI Update Timer (Task 4 driver) ---
                    StartUiUpdateTimer(26.0); // Target 26 FPS

                    if (_networkStream != null && _networkStream.CanRead) // Check if the stream is viable
                    {
                        Console.WriteLine("[CameraController ConnectToPiServerAsync] Pi Camera stream established. Signaling RemoteSystemReadyForActions.");
                        SignalController.Instance.RaiseRemoteSystemReadyForActions();
                    }
                    else
                    {
                        Console.WriteLine("[CameraController ConnectToPiServerAsync WARN] Pi Camera stream is null or not readable after connect attempt. Cannot signal ready.");
                    }

                    // Wait for EITHER loop task to complete
                    await Task.WhenAny(receiveTask, processingTask);

                    Console.WriteLine("[Connect] A loop task has completed. Session ending.");
                    StopUiUpdateTimer(); // Stop timer when loops end
                }
                catch (OperationCanceledException)
                {
                    Console.WriteLine("[Connect] Operation cancelled during connection or loops.");
                    // The while loop condition will terminate it.
                }
                catch (Exception ex)
                {
                    // This catches errors during the *connection attempt phase*
                    Console.WriteLine($"[Connect] Connection attempt failed: {ex.GetType().Name} - {ex.Message}");
                    // Cleanup is handled by the finally block below.
                    // If not cancelled, wait before retrying
                    if (!token.IsCancellationRequested)
                    {
                        Console.WriteLine("[Connect] Waiting 3 seconds before retry...");
                        try { await Task.Delay(3000, token); }
                        catch (OperationCanceledException) { /* Allow cancellation during delay */ }
                    }
                }
                finally
                {
                    Console.WriteLine("[Connect] Cleaning up network resources for this attempt...");
                    // Ensure network resources are cleaned up regardless of success or failure within the try block
                    // This allows the loop to retry connecting.
                    CleanupNetworkResources(); // Use the helper method

                    // If the connection was successful but loops ended (not due to cancellation),
                    // add a small delay before the automatic retry by the while loop.
                    if (connectionAttemptSuccess && !token.IsCancellationRequested)
                    {
                        Console.WriteLine("[Connect] Connection dropped. Waiting 3 seconds before automatic retry...");
                        try { await Task.Delay(3000, token); } catch (OperationCanceledException) { }
                    }
                }
            } // End while (!token.IsCancellationRequested)

            Console.WriteLine("[Connect] Connection loop exited permanently (likely cancelled).");
            // Final full cleanup is handled by DisposeAsync/Dispose
        }

        private async Task ConnectVisionAsync(string ip, int port, CancellationToken token) { /*...*/ _visionClient = new TcpClient(); await _visionClient.ConnectAsync(ip, port, token); visionStream = _visionClient.GetStream(); Console.WriteLine("[Connect] -> Vision.py Connected!"); }
        private async Task ConnectPiCameraAsync(string ip, int port, CancellationToken token) { /*...*/ _client = new TcpClient(); await _client.ConnectAsync(ip, port, token); _networkStream = _client.GetStream(); Console.WriteLine("[Connect] -> Pi Camera Connected!"); }
        //private async Task ConnectPiCommandAsync(string ip, int port, CancellationToken token) { /*...*/ _commandClient = new TcpClient(); await _commandClient.ConnectAsync(ip, port, token); _commandStream = _commandClient.GetStream(); _commandWriter = new StreamWriter(_commandStream, Encoding.UTF8) { AutoFlush = true }; Console.WriteLine("[Connect] -> Pi Command Connected!"); }

        private void StartUiUpdateTimer(double targetFps)
        {
            StopUiUpdateTimer(); // Ensure no duplicates

            if (targetFps <= 0) targetFps = 30.0; // Default/fallback FPS
            double intervalMs = 1000.0 / targetFps;

            _uiUpdateTimer = new DispatcherTimer(DispatcherPriority.Background); // Use Background priority for UI updates
            _uiUpdateTimer.Interval = TimeSpan.FromMilliseconds(intervalMs);
            _uiUpdateTimer.Tick += UiUpdateTimer_Tick;
            _uiUpdateTimer.Start();
            Console.WriteLine($"[CameraController] UI Update Timer started (Interval: {intervalMs:F2}ms, Target FPS: {targetFps}).");
        }

        private void UiUpdateTimer_Tick(object? sender, EventArgs e)
        {
            ProcessedData? dataToShow = null;
            bool hadNewResult;

            lock (_latestResultLock)
            {
                hadNewResult = _newResultAvailable;
                if (hadNewResult)
                {
                    dataToShow = _latestProcessedData;
                    _newResultAvailable = false; // Consume the new result
                }
            }

            if (hadNewResult && dataToShow != null) // Ensure dataToShow itself is not null
            {
                // --- Update Camera Image ---
                if (_cameraImage != null) // Check if the UI element is available
                {
                    Bitmap? newBitmap = dataToShow.ProcessedBitmap;
                    Bitmap? oldBitmap = _cameraImage.Source as Bitmap;

                    if (newBitmap != null)
                    {
                        // We have a new, valid bitmap to display
                        _cameraImage.Source = newBitmap;

                        // If there was an old bitmap and it's different from the new one, dispose the old one.
                        if (oldBitmap != null && !ReferenceEquals(oldBitmap, newBitmap))
                        {
                            // Console.WriteLine("[CameraController UI Tick] Disposing old bitmap.");
                            oldBitmap.Dispose();
                        }
                    }
                    else
                    {
                        // The new processed bitmap is null.
                        // Decide what to do:
                        // Option 1: Clear the image display
                        // _cameraImage.Source = null; // This might be acceptable if your UI can handle it without crashing.
                        // Option 2: Keep the last valid image (do nothing to _cameraImage.Source)
                        Console.WriteLine("[CameraController UI Tick] New ProcessedBitmap is null. Keeping the previous image displayed (if any).");
                        // Option 3: Set a placeholder "no signal" image if you have one
                        // _cameraImage.Source = _noSignalPlaceholderBitmap;
                    }
                }

                // --- Raise event for Encoders ---
                if (dataToShow.EncoderValues != null)
                {
                    // It's good practice to add the log here that you had before, to confirm what's being sent:
                    Console.WriteLine($"*** [CameraController UI Tick] Raising EncodersReceived with: {(dataToShow.EncoderValues == null ? "NULL" : string.Join(",", dataToShow.EncoderValues))} ***");
                    SignalController.Instance.RaiseEncodersReceived(dataToShow.EncoderValues);
                }
                else
                {
                    // This case should also be logged if it happens, as it means encoder values were unexpectedly null
                    Console.WriteLine("*** [CameraController UI Tick] dataToShow.EncoderValues is NULL. Not raising EncodersReceived. ***");
                }

                if (dataToShow.TofHandValue.HasValue || dataToShow.TofCamValue.HasValue)
                {
                    Console.WriteLine($"*** [CameraController UI Tick] Raising TofSensorsReceived with Hand: {dataToShow.TofHandValue?.ToString() ?? "N/A"}, Cam: {dataToShow.TofCamValue?.ToString() ?? "N/A"} ***");
                    SignalController.Instance.RaiseTofSensorsReceived(dataToShow.TofHandValue, dataToShow.TofCamValue);
                }

                // --- Raise event for Detections ---
                if (dataToShow.Detections != null)
                {
                    SignalController.Instance.RaiseDetectionsReceived(dataToShow.Detections);
                }
            }
        }

        private void StopUiUpdateTimer()
        {
            if (_uiUpdateTimer != null)
            {
                _uiUpdateTimer.Stop();
                _uiUpdateTimer.Tick -= UiUpdateTimer_Tick;
                _uiUpdateTimer = null;
                Console.WriteLine("[CameraController] UI Update Timer stopped.");
                // Reset availability flag when timer stops
                lock (_latestResultLock) { _newResultAvailable = false; }
            }
        }


        // --- NEW: Separate Image Processing Task ---
        private async Task<(Bitmap? ProcessedBitmap, List<DetectionResult>? Detections)> ProcessImageTaskAsync(
            byte[] jpegBytes, CancellationToken cancellationToken)
        {
            Bitmap? bitmap = null;
            List<DetectionResult>? detections = null;
            byte[]? processedJpeg = null;

            var currentVisionStream = visionStream;
            if (currentVisionStream == null || !currentVisionStream.CanWrite || !currentVisionStream.CanRead)
            {
                Console.WriteLine("[ProcessImageTaskAsync WARN] visionStream not available.");
                return (null, null);
            }

            try
            {
                // 1. Send Raw JPEG
                byte[] sizeBytes = BitConverter.GetBytes(IPAddress.HostToNetworkOrder(jpegBytes.Length));
                await currentVisionStream.WriteAsync(sizeBytes, 0, 4, cancellationToken);
                await currentVisionStream.WriteAsync(jpegBytes, 0, jpegBytes.Length, cancellationToken);

                // 2. Read JSON length
                byte[] jsonLengthBytes = new byte[4];
                int read = await currentVisionStream.ReadAsync(jsonLengthBytes, 0, 4, cancellationToken);
                if (read != 4) throw new IOException("Failed to read JSON length fully.");
                int jsonLength = IPAddress.NetworkToHostOrder(BitConverter.ToInt32(jsonLengthBytes, 0));
                if (jsonLength < 0 || jsonLength > 1_000_000) throw new IOException($"Invalid JSON data length: {jsonLength}");

                if (jsonLength > 0)
                {
                    byte[] jsonBytes = new byte[jsonLength];
                    int totalJsonRead = 0;
                    while (totalJsonRead < jsonLength)
                    {
                        int chunk = await currentVisionStream.ReadAsync(jsonBytes, totalJsonRead, jsonLength - totalJsonRead, cancellationToken);
                        if (chunk == 0) throw new IOException("visionStream closed during JSON data read.");
                        totalJsonRead += chunk;
                    }
                    string jsonString = Encoding.UTF8.GetString(jsonBytes);
                    detections = JsonSerializer.Deserialize<List<DetectionResult>>(jsonString);
                }
                else { detections = new List<DetectionResult>(); }

                // 3. Read image length
                byte[] imageLengthBytes = new byte[4];
                read = await currentVisionStream.ReadAsync(imageLengthBytes, 0, 4, cancellationToken);
                if (read != 4) throw new IOException("Failed to read processed image length fully.");
                int processedImageLength = IPAddress.NetworkToHostOrder(BitConverter.ToInt32(imageLengthBytes, 0));
                if (processedImageLength < 0 || processedImageLength > 10_000_000) throw new IOException($"Invalid processed image length: {processedImageLength}");

                if (processedImageLength > 0)
                {
                    processedJpeg = new byte[processedImageLength];
                    int totalImageRead = 0;
                    while (totalImageRead < processedImageLength)
                    {
                        int chunk = await currentVisionStream.ReadAsync(processedJpeg, totalImageRead, processedImageLength - totalImageRead, cancellationToken);
                        if (chunk == 0) throw new IOException("visionStream closed during processed image read.");
                        totalImageRead += chunk;
                    }
                    using var ms = new MemoryStream(processedJpeg);
                    bitmap = new Bitmap(ms);
                }
                else { bitmap = null; }

                return (bitmap, detections);
            }
            catch (OperationCanceledException) { Console.WriteLine("[ProcessImageTaskAsync] Cancelled."); return (null, null); }
            catch (JsonException jsonEx) { Console.WriteLine($"[ProcessImageTaskAsync JSON ERROR] {jsonEx.Message}"); return (null, detections); }
            catch (Exception ex) { Console.WriteLine($"[ProcessImageTaskAsync ERROR] {ex.GetType().Name}: {ex.Message}"); return (null, null); }
        }

        // --- ReceiveLoopAsync (Producer) ---
        private async Task ReceiveLoopAsync(CancellationToken token)
        {
            if (_networkStream == null || _cts == null) { Console.WriteLine("[ReceiveLoop ERR] Stream or CTS is null."); return; }
            Console.WriteLine("[ReceiveLoop] Starting (with TOF parsing)...");
            try
            {
                while (!token.IsCancellationRequested)
                {
                    //byte[] packetBuffer = null;
                    byte[]? jpegBytes = null;
                    int[]? encoderValues = null;
                    int? tofHand = null;
                    int? tofCam = null;

                    try
                    {
                        byte[] sizeBuffer = new byte[4];
                        int read = await _networkStream.ReadAsync(sizeBuffer, 0, 4, token);
                        if (read == 0) { Console.WriteLine("[ReceiveLoop] Server closed connection (read size 0)."); break; }
                        if (read != 4)
                        {
                            Console.WriteLine($"[ReceiveLoop WARN] Failed to read packet size header fully (read {read}/4 bytes). Skipping packet.");
                            await Task.Delay(10, token);
                            byte[] discardBuffer = new byte[1024];
                            while (_networkStream.DataAvailable) { await _networkStream.ReadAsync(discardBuffer, 0, discardBuffer.Length, token); }
                            continue;
                        }
                        int packetSize = BitConverter.ToInt32(sizeBuffer.Reverse().ToArray(), 0);
                        if (packetSize <= 0 || packetSize > 50_000_000) { Console.WriteLine($"[ReceiveLoop WARN] Invalid packet size: {packetSize}. Skipping."); continue; }


                        byte[] packetBuffer = new byte[packetSize];
                        int bytesRead = 0;
                        while (bytesRead < packetSize)
                        {
                            int chunk = await _networkStream.ReadAsync(packetBuffer, bytesRead, packetSize - bytesRead, token);
                            if (chunk == 0) throw new IOException("Socket closed during packet read.");
                            bytesRead += chunk;
                        }

                        int splitIndex = Array.IndexOf(packetBuffer, (byte)'\n');
                        if (splitIndex == -1)
                        {
                            Console.WriteLine("[ReceiveLoop WARN] Invalid packet format (no newline). Skipping packet.");
                            continue;
                        }
                        string labelAndSensorData = Encoding.UTF8.GetString(packetBuffer, 0, splitIndex);
                        jpegBytes = packetBuffer.AsSpan(splitIndex + 1).ToArray();
                        Console.WriteLine($"[ReceiveLoop] Full Label String: {labelAndSensorData}");

                        string[] allParts = labelAndSensorData.Split(':');
                        // Example: "ENCODERS:val1,val2,val3,val4:TOF_HAND_I2C:tof_h_val:TOF_CAM_UART:tof_c_val"

                        if (allParts.Length > 1 && allParts[0].Trim().ToUpper() == "ENCODERS")
                        {
                            string[] encoderStrValues = allParts[1].Split(',');
                            if (encoderStrValues.Length == 4)
                            {
                                encoderValues = new int[4];
                                bool encodersParsedSuccessfully = true;
                                for (int i = 0; i < 4; i++)
                                {
                                    if (!int.TryParse(encoderStrValues[i].Trim(), out encoderValues[i]))
                                    {
                                        Console.WriteLine($"[ReceiveLoop WARN] Failed to parse encoder value: {encoderStrValues[i]}");
                                        encodersParsedSuccessfully = false;
                                        break;
                                    }
                                }
                                if (!encodersParsedSuccessfully) encoderValues = null;
                            }
                            else
                            {
                                Console.WriteLine($"[ReceiveLoop WARN] Incorrect number of encoder values. Expected 4, got {encoderStrValues.Length}. Data: {allParts[1]}");
                            }
                        }
                        else
                        {
                            Console.WriteLine($"[ReceiveLoop WARN] 'ENCODERS:' keyword not found or malformed at the beginning of: {labelAndSensorData}");
                        }

                        // Search for TOF keys and their values
                        for (int i = 0; i < allParts.Length - 1; i++) // Iterate up to second to last element
                        {
                            if (allParts[i].Trim().ToUpper() == "TOF_HAND_I2C")
                            {
                                if (int.TryParse(allParts[i + 1].Trim(), out int thVal))
                                    tofHand = thVal;
                                else
                                    Console.WriteLine($"[ReceiveLoop WARN] Failed to parse TOF_HAND_I2C value: '{allParts[i + 1]}'");
                                i++; // Skip the value part in next iteration
                            }
                            else if (allParts[i].Trim().ToUpper() == "TOF_CAM_UART")
                            {
                                if (int.TryParse(allParts[i + 1].Trim(), out int tcVal))
                                    tofCam = tcVal;
                                else
                                    Console.WriteLine($"[ReceiveLoop WARN] Failed to parse TOF_CAM_UART value: '{allParts[i + 1]}'");
                                i++; // Skip the value part
                            }
                        }
                        Console.WriteLine($"[ReceiveLoop] Parsed: Encoders={(encoderValues == null ? "N/A" : string.Join(",", encoderValues))}, TOF_Hand={tofHand?.ToString() ?? "N/A"}, TOF_Cam={tofCam?.ToString() ?? "N/A"}");


                        if (jpegBytes != null) // Should always be true if splitIndex was found
                        {
                            var packet = new FrameDataPacket(jpegBytes, encoderValues, tofHand, tofCam);
                            await _processingChannel.Writer.WriteAsync(packet, token);
                        }
                    }
                    catch (OperationCanceledException) { break; }
                    catch (IOException ioEx) { Console.WriteLine($"[ReceiveLoop IO ERROR] {ioEx.Message}"); break; }
                    catch (Exception ex) { Console.WriteLine($"[ReceiveLoop PACKET ERROR] {ex.Message}"); continue; }
                }
            }
            finally
            {
                Console.WriteLine("[ReceiveLoop] Exiting loop.");
            }
            Console.WriteLine("[ReceiveLoop] Ended.");
        }

        // Update ProcessingLoopAsync to handle new FrameDataPacket structure
        private async Task ProcessingLoopAsync(CancellationToken cancellationToken)
        {
            Console.WriteLine("[ProcessingLoop] Starting (passing TOF through)...");
            try
            {
                await foreach (FrameDataPacket packet in _processingChannel.Reader.ReadAllAsync(cancellationToken))
                {
                    if (cancellationToken.IsCancellationRequested) break;

                    var imageProcessingResult = await ProcessImageTaskAsync(packet.JpegBytes, cancellationToken);

                    // Pass all data from the packet along
                    var newData = new ProcessedData(
                        imageProcessingResult.ProcessedBitmap,
                        packet.EncoderValues,
                        imageProcessingResult.Detections,
                        packet.TofHand, // Pass TOF Hand
                        packet.TofCam   // Pass TOF Cam
                    );

                    lock (_latestResultLock)
                    {
                        _latestProcessedData?.ProcessedBitmap?.Dispose(); // Dispose previous bitmap if any
                        _latestProcessedData = newData;
                        _newResultAvailable = true;
                    }
                }
            }
            catch (OperationCanceledException) { Console.WriteLine("[ProcessingLoop] Cancelled."); }
            catch (ChannelClosedException) { Console.WriteLine("[ProcessingLoop] Channel closed."); }
            catch (Exception ex) { Console.WriteLine($"[ProcessingLoop ERROR] {ex.GetType().Name}: {ex.Message}"); }
            finally
            {
                Console.WriteLine("[ProcessingLoop] Exiting loop.");
                lock (_latestResultLock)
                {
                    _latestProcessedData?.ProcessedBitmap?.Dispose();
                    _latestProcessedData = null;
                }
            }
            Console.WriteLine("[ProcessingLoop] Ended.");
        }

        //// --- Send Command ---
        private async Task<int[]?> ProcessEncoderTaskAsync(int[]? encoderValues, CancellationToken cancellationToken)
        {
            // If there was CPU-intensive work needed on encoders, it would go here
            // await Task.Delay(1, cancellationToken); // Simulate tiny work if needed
            return await Task.FromResult(encoderValues); // Efficiently return existing value
        }
        

        // --- IDisposable and IAsyncDisposable Implementation ---

        // Call this to cleanup resources
        public async ValueTask DisposeAsync()
        {
            await DisposeAsyncCore();
            Dispose(disposing: false); // Suppress finalization check
            GC.SuppressFinalize(this);
        }

        protected virtual async ValueTask DisposeAsyncCore()
        {
            if (!_disposed)
            {
                Console.WriteLine("[DisposeAsyncCore] Starting async cleanup...");
                // Await the stop method which handles cancellation and resource cleanup
                await StopCameraFeedAsync();
                _arduino.StopArduinoThread(); // Ensure thread is stopped


                // Unsubscribe from events
                //if (SignalController.Instance != null)
                //{ // Check if instance exists
                //    SignalController.Instance.ModeChanged -= SetModeAsync();
                //}

                Console.WriteLine("[DisposeAsyncCore] Async cleanup finished.");
            }
        }



        private void CleanupNetworkResources()
        {
            Console.WriteLine("[CleanupNetworkResources] Closing network streams and clients...");
            // Use try-catch for each disposal to prevent one failure stopping others
           _arduino.CleanupArduinoResources();

            try { _networkStream?.Dispose(); } catch (Exception ex) { Console.WriteLine($"[Cleanup Error] NetworkStream (Pi Camera): {ex.Message}"); }
            _networkStream = null;
            try { _client?.Dispose(); } catch (Exception ex) { Console.WriteLine($"[Cleanup Error] Client (Pi Camera): {ex.Message}"); }
            _client = null;

            try { visionStream?.Dispose(); } catch (Exception ex) { Console.WriteLine($"[Cleanup Error] VisionStream: {ex.Message}"); }
            visionStream = null;
            try { _visionClient?.Dispose(); } catch (Exception ex) { Console.WriteLine($"[Cleanup Error] VisionClient: {ex.Message}"); }
            _visionClient = null;
            Console.WriteLine("[CleanupNetworkResources] Network cleanup finished.");
        }

        // Standard Dispose pattern (sync)
        public void Dispose()
        {
            Dispose(disposing: true);
            GC.SuppressFinalize(this);
        }

        protected virtual void Dispose(bool disposing)
        {
            if (!_disposed)
            {
                if (disposing)
                {
                    Console.WriteLine("[Dispose] Disposing managed resources...");
                    // Minimal synchronous cleanup here.
                    // Ensure CTS is cancelled and disposed if not already.
                    if (_cts != null && !_cts.IsCancellationRequested)
                    {
                        try { _cts.Cancel(); } catch { } // Best effort cancel
                    }
                    _cts?.Dispose(); // Dispose CTS handle

                    // Attempt to complete channel writer if not already done
                    _processingChannel.Writer.TryComplete();
                    //_arduino.StopArduinoThread(); // Ensure thread is stopped


                    // Other strictly synchronous managed resource cleanup can go here,
                    // but most cleanup is handled by StopCameraFeedAsync called via DisposeAsyncCore.
                    Console.WriteLine("[Dispose] Managed resource disposal attempted.");
                }

                lock (_latestResultLock)
                {
                    _latestProcessedData?.ProcessedBitmap?.Dispose();
                    _latestProcessedData = null;
                    _bitmapCurrentlyDisplayed?.Dispose();
                    _bitmapCurrentlyDisplayed = null;
                }
                Console.WriteLine("[Dispose] Final bitmaps disposed (sync path).");

                // Cleanup unmanaged resources here if any (none in this class directly)

                _disposed = true;
                Console.WriteLine("[Dispose] Completed.");
            }
        }

        // Finalizer (optional, only if you have unmanaged resources directly in this class)
        // ~CameraController()
        // {
        //     Dispose(disposing: false);
        // }
    }
}