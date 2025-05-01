using Avalonia.Controls;
using Avalonia.Media.Imaging;
using Avalonia.Threading;
using System;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Channels;
using System.Threading.Tasks;

namespace RobotAIArm.Controllers
{
    // Implement IAsyncDisposable for proper async cleanup
    public class CameraController : IDisposable, IAsyncDisposable
    {
        private readonly Image _cameraImage;
        private readonly ArduinoController _arduino;
        private Process? _visionProcess;

        // Network resources - make nullable
        private TcpClient? _client;           // Connection to Pi Camera Port (23456)
        private NetworkStream? _networkStream;
        private TcpClient? _visionClient;     // Connection to local Vision.py (34567)
        private NetworkStream? visionStream;
        private TcpClient? _commandClient;    // Connection to Pi Command Port (23457)
        private NetworkStream? _commandStream;
        private StreamWriter? _commandWriter; // Writer for commands

        private CancellationTokenSource? _cts;
        private bool _disposed = false; // To detect redundant calls

        // Channel for decoupling processing
        private readonly Channel<byte[]> _processingChannel = Channel.CreateBounded<byte[]>(new BoundedChannelOptions(5)
        {
            FullMode = BoundedChannelFullMode.DropOldest
        });

        public CameraController(Image cameraImage, ArduinoController arduino)
        {
            _cameraImage = cameraImage;
            _arduino = arduino; // Assuming ArduinoController is used elsewhere

            // Subscribe to mode changes
            //SignalController.Instance.ModeChanged += OnModeChanged;
            // Note: OnFrameReceived might be obsolete if ProcessingLoop updates UI
        }

        // This method might not be needed if ProcessingLoop handles UI updates
        // based on the *processed* image. Keep if you need raw image display too.
        /*
        private void OnFrameReceived(byte[] frameBytes)
        {
            // ... (implementation if needed) ...
        }
        */
        private async Task StopCameraFeedAsync()
        {
            Console.WriteLine("[CameraController] Stopping camera feed...");
            try
            {
                // 1. Signal cancellation to loops
                if (_cts != null && !_cts.IsCancellationRequested)
                {
                    _cts.Cancel();
                    Console.WriteLine("[CameraController] Cancellation token signaled.");
                }
                _cts?.Dispose(); // Dispose the CTS itself
                _cts = null;

                // 2. Close network streams and clients
                _networkStream?.Dispose(); // Dispose Pi stream
                _networkStream = null;
                _client?.Dispose(); // Dispose Pi client
                _client = null;

                visionStream?.Dispose(); // Dispose vision stream
                visionStream = null;
                _visionClient?.Dispose(); // Dispose vision client
                _visionClient = null;
                Console.WriteLine("[CameraController] Network connections closed.");


                // 3. Terminate Python process
                if (_visionProcess != null && !_visionProcess.HasExited) // Check if already exited
                {
                    try
                    {
                        Console.WriteLine($"[CameraController] Killing Vision process (PID: {_visionProcess.Id})...");
                        _visionProcess.Kill(true); // Kill process and its children

                        // --- Correct way to await exit with timeout ---
                        Console.WriteLine("[CameraController] Waiting for vision process exit (max 5s)...");
                        // Create a CancellationTokenSource that cancels after 5 seconds
                        using var exitCts = new CancellationTokenSource(TimeSpan.FromSeconds(5));
                        try
                        {
                            // Wait for the process to exit OR the timeout token to be cancelled
                            await _visionProcess.WaitForExitAsync(exitCts.Token);
                            Console.WriteLine("[CameraController] Vision process exited gracefully after kill signal.");
                        }
                        catch (OperationCanceledException) // Thrown if the timeout (exitCts.Token) expires before exit
                        {
                            Console.WriteLine("[CameraController] Timeout expired waiting for vision process exit.");
                            // Process might still be running, but we stop waiting.
                        }
                        // --- End corrected wait ---
                    }
                    catch (InvalidOperationException ioEx)
                    {
                        // Process may have already exited between check and Kill/WaitForExitAsync
                        Console.WriteLine($"[CameraController] Info: Error managing vision process (may have already exited): {ioEx.Message}");
                    }
                    catch (Exception procEx)
                    {
                        Console.WriteLine($"[CameraController] Error killing/waiting for vision process: {procEx.Message}");
                    }
                    finally
                    {
                        // Ensure dispose happens even if errors occurred
                        _visionProcess.Dispose();
                        _visionProcess = null;
                    }
                }
                else
                {
                    // If process handle exists but process already exited
                    _visionProcess?.Dispose();
                    _visionProcess = null;
                }


                // 4. Clear the processing channel (optional, good practice)
                // Try reading remaining items to allow writer completion if stuck
                while (_processingChannel.Reader.TryRead(out _)) { }


                Console.WriteLine("[CameraController] Camera feed stopped.");
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[CameraController] Error stopping camera feed: {ex.Message}");
            }
        }

        //private async void OnModeChanged(bool isRemote)
        //{
        //    Console.WriteLine($"[CameraController] Mode changed handler invoked. Remote={isRemote}");

        //    // --- Stop Existing Feed ---
        //    // DisposeAsyncCore (called via StopCameraFeedAsync -> DisposeAsync) handles cancellation and cleanup
        //    await StopCameraFeedAsync(); // Ensure previous state is cleaned up

        //    if (isRemote)
        //    {
        //        Console.WriteLine("[CameraController] Mode changed to Remote. Starting setup...");
        //        // --- Start Vision Script ---
        //        StartVisionPipelineRemote(); // Start the local Python script first

        //        // --- Attempt Connection (which now includes starting loops) ---
        //        // Run the connection attempts in the background.
        //        // The ConnectToPiServerAsync loop handles retries internally.
        //        _ = ConnectToPiServerAsync(); // Fire and forget the connection loop task
        //    }
        //    else
        //    {
        //        Console.WriteLine("[CameraController] Mode changed to Local. Ensuring remote connections are stopped.");
        //        StartVisionPipelineLocal(); // Placeholder for local mode logic if any
        //        // StopCameraFeedAsync already cleaned up remote connections
        //    }
        //}
        public async Task SetModeAsync(bool isRemote) // Changed from private async void OnModeChanged
        {
            Console.WriteLine($"[CameraController] SetModeAsync called. Remote={isRemote}");

            // --- Stop Existing Feed ---
            await StopCameraFeedAsync(); // Stop previous state first

            if (isRemote)
            {
                Console.WriteLine("[CameraController] SetModeAsync: Setting up Remote mode...");
                // --- Start Vision Script ---
                StartVisionPipelineRemote();

                // --- Attempt Connection ---
                // Run connection attempts in the background. Handles retries internally.
                _ = ConnectToPiServerAsync();
            }
            else
            {
                Console.WriteLine("[CameraController] SetModeAsync: Setting up Local mode...");
                StartVisionPipelineLocal();
                // StopCameraFeedAsync already cleaned up remote connections
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
            // Ensure settings are loaded correctly
            Console.WriteLine("[ConnectToPiServerAsync] Method Entered.");

            string serverIp = AppSettings.Instance.RemoteIP;
            if (string.IsNullOrEmpty(serverIp))
            {
                Console.WriteLine("[Connect Error] Remote IP is not set in AppSettings.");
                return;
            }
            string pythonIp = "127.0.0.1";
            int pythonPort = 34567;
            int cameraPort = 23456;
            int commandPort = 23457;

            // Dispose previous CTS if any and create a new one for this connection lifecycle
            _cts?.Dispose();
            _cts = new CancellationTokenSource();
            var token = _cts.Token;

            Console.WriteLine($"[Connect] Starting connection attempts to Pi ({serverIp}) and Vision ({pythonIp}:{pythonPort})...");

            while (!token.IsCancellationRequested)
            {
                TcpClient? tempVisionClient = null;
                TcpClient? tempPiClient = null;
                TcpClient? tempCommandClient = null;
                bool connectedSuccessfully = false;

                try
                {
                    // --- Connect to Local Vision Script ---
                    Console.WriteLine($"[Connect] Attempting -> Vision.py ({pythonIp}:{pythonPort})...");
                    tempVisionClient = new TcpClient();
                    await tempVisionClient.ConnectAsync(pythonIp, pythonPort, token);
                    _visionClient = tempVisionClient; // Assign to class field on success
                    visionStream = _visionClient.GetStream();
                    Console.WriteLine("[Connect] -> Vision.py Connected!");

                    // --- Connect to Raspberry Pi Camera Port ---
                    Console.WriteLine($"[Connect] Attempting -> Pi Camera ({serverIp}:{cameraPort})...");
                    tempPiClient = new TcpClient();
                    await tempPiClient.ConnectAsync(serverIp, cameraPort, token);
                    _client = tempPiClient; // Assign to class field on success
                    _networkStream = _client.GetStream();
                    Console.WriteLine("[Connect] -> Pi Camera Connected!");

                    // --- Connect to Raspberry Pi Command Port ---
                    Console.WriteLine($"[Connect] Attempting -> Pi Command ({serverIp}:{commandPort})...");
                    tempCommandClient = new TcpClient();
                    await tempCommandClient.ConnectAsync(serverIp, commandPort, token);
                    _commandClient = tempCommandClient; // Assign to class field on success
                    _commandStream = _commandClient.GetStream();
                    _commandWriter = new StreamWriter(_commandStream, Encoding.UTF8) { AutoFlush = true };
                    Console.WriteLine("[Connect] -> Pi Command Connected!");

                    connectedSuccessfully = true; // All connections established

                    // --- Start Producer and Consumer Loops Concurrently ---
                    Console.WriteLine("[Connect] Starting Receive and Processing loops...");
                    Task receiveTask = ReceiveLoopAsync();      // Producer
                    Task processingTask = ProcessingLoopAsync(token); // Consumer

                    // Wait for EITHER task to complete (indicates disconnection, error, or cancellation)
                    await Task.WhenAny(receiveTask, processingTask);

                    Console.WriteLine("[Connect] A loop task has completed (disconnect/error/cancel).");
                    // Exiting WhenAny means the session is over for this attempt.
                    // StopCameraFeedAsync will be called below or by external cancellation.
                    break; // Exit the while loop after a session ends

                }
                catch (OperationCanceledException)
                {
                    Console.WriteLine("[Connect] Operation cancelled during connection or loops.");
                    break; // Exit while loop cleanly
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"[Connect] Connection attempt failed: {ex.GetType().Name} - {ex.Message}");
                    // Clean up temporary clients from this failed attempt
                    tempVisionClient?.Dispose();
                    tempPiClient?.Dispose();
                    tempCommandClient?.Dispose();

                    // If not cancelled, wait before retrying
                    if (!token.IsCancellationRequested)
                    {
                        Console.WriteLine("[Connect] Waiting 3 seconds before retry...");
                        try { await Task.Delay(3000, token); } catch (OperationCanceledException) { break; } // Allow delay to be cancelled
                    }
                }
                finally
                {
                    // If connection was successful but WhenAny exited, ensure cleanup via StopCameraFeedAsync
                    if (connectedSuccessfully)
                    {
                        Console.WriteLine("[Connect] Session ended. Triggering cleanup...");
                        await StopCameraFeedAsync(); // Ensure cleanup after loops finish/fail
                    }
                }
            } // End while connection loop

            Console.WriteLine("[Connect] Connection loop exited.");
            // Final cleanup check, StopCameraFeedAsync should handle most cases
            await StopCameraFeedAsync(); // Call stop explicitly if loop ends for any reason other than ongoing cancellation
        }


        // --- ReceiveLoopAsync (Producer) ---
        private async Task ReceiveLoopAsync()
        {
            if (_networkStream == null || _cts == null)
            {
                Console.WriteLine("[ReceiveLoop Error] Stream or CancellationTokenSource is null.");
                return;
            }
            var token = _cts.Token; // Get token at start
            Console.WriteLine("[ReceiveLoop] Starting...");
            try
            {
                while (!token.IsCancellationRequested)
                {
                    byte[]? jpegBytes = null;
                    try
                    {
                        // Step 1: Read frame from the source (Pi Camera)
                        byte[] sizeBuffer = new byte[4];
                        int read = await _networkStream.ReadAsync(sizeBuffer, 0, 4, token);
                        if (read == 0) { Console.WriteLine("[ReceiveLoop] Server closed connection (read size 0)."); break; }
                        if (read != 4) throw new IOException("Failed to read packet size header fully.");

                        int packetSize = BitConverter.ToInt32(sizeBuffer.Reverse().ToArray(), 0);
                        if (packetSize <= 0 || packetSize > 50_000_000) throw new IOException($"Invalid packet size: {packetSize}");

                        byte[] packetBuffer = new byte[packetSize];
                        int bytesRead = 0;
                        while (bytesRead < packetSize)
                        {
                            int chunk = await _networkStream.ReadAsync(packetBuffer, bytesRead, packetSize - bytesRead, token);
                            if (chunk == 0) throw new IOException("Socket closed during packet read.");
                            bytesRead += chunk;
                        }

                        // Step 2: Extract label and raw JPEG
                        int splitIndex = Array.IndexOf(packetBuffer, (byte)'\n');
                        if (splitIndex == -1) throw new FormatException("Invalid packet format (no newline)");

                        string label = Encoding.UTF8.GetString(packetBuffer, 0, splitIndex);
                        Console.WriteLine($"[ENCODERS] {label}"); // Log received label

                        jpegBytes = packetBuffer.AsSpan(splitIndex + 1).ToArray(); // Use AsSpan for efficiency
                    }
                    catch (OperationCanceledException) { Console.WriteLine("[ReceiveLoop] Cancellation requested during read."); break; }
                    catch (IOException ioEx) { Console.WriteLine($"[ReceiveLoop IO ERROR] {ioEx.Message}"); break; } // Exit loop on socket read errors
                    catch (FormatException formatEx) { Console.WriteLine($"[ReceiveLoop PACKET FORMAT ERROR] {formatEx.Message}"); continue; } // Log format error, try next packet
                    catch (Exception ex) { Console.WriteLine($"[ReceiveLoop PACKET ERROR] {ex.GetType().Name}: {ex.Message}"); continue; } // Log other errors, try next packet

                    // Step 2.5: Add successfully extracted JPEG data to the processing channel
                    if (jpegBytes != null)
                    {
                        bool success = _processingChannel.Writer.TryWrite(jpegBytes);
                        if (!success)
                        {
                            Console.WriteLine("[ReceiveLoop WARNING] Processing channel full, frame dropped.");
                        }
                    }
                } // End while loop
            }
            finally
            {
                Console.WriteLine("[ReceiveLoop] Exiting loop.");
                // Don't close streams here, let StopCameraFeedAsync handle it
                _processingChannel.Writer.TryComplete(); // Signal that this producer is done
            }
            Console.WriteLine("[ReceiveLoop] Ended.");
        }

        // --- ProcessingLoopAsync (Consumer) ---
        private async Task ProcessingLoopAsync(CancellationToken cancellationToken)
        {
            if (visionStream == null)
            {
                Console.WriteLine("[ProcessingLoop Error] visionStream is null.");
                return; // Cannot proceed without the vision stream
            }
            Console.WriteLine("[ProcessingLoop] Starting...");
            try
            {
                await foreach (byte[] jpegBytes in _processingChannel.Reader.ReadAllAsync(cancellationToken))
                {
                    if (visionStream == null || !visionStream.CanWrite || !visionStream.CanRead)
                    {
                        Console.WriteLine("[ProcessingLoop WARN] visionStream is not available or closed. Skipping frame.");
                        continue; // Skip if vision stream disconnected
                    }

                    try // Process each frame individually
                    {
                        //Console.WriteLine($"[ProcessingLoop] Processing frame (size: {jpegBytes.Length}).");

                        // Step 3: Send JPEG to Vision.py via visionStream
                        byte[] sizeBytes = BitConverter.GetBytes(IPAddress.HostToNetworkOrder(jpegBytes.Length));
                        await visionStream.WriteAsync(sizeBytes, 0, 4, cancellationToken);
                        await visionStream.WriteAsync(jpegBytes, 0, jpegBytes.Length, cancellationToken);

                        // Step 4: Receive processed JPEG size from visionStream
                        byte[] responseSize = new byte[4];
                        int r = await visionStream.ReadAsync(responseSize, 0, 4, cancellationToken);
                        if (r == 0) throw new IOException("visionStream closed while reading size");
                        if (r != 4) throw new IOException("Failed to read processed JPEG size fully");

                        int processedSize = IPAddress.NetworkToHostOrder(BitConverter.ToInt32(responseSize, 0));
                        if (processedSize <= 0 || processedSize > 10_000_000) throw new IOException($"Invalid processed JPEG size: {processedSize}");

                        // Step 5: Read processed JPEG bytes from visionStream
                        byte[] processedJpeg = new byte[processedSize];
                        int totalRead = 0;
                        while (totalRead < processedSize)
                        {
                            int chunk = await visionStream.ReadAsync(processedJpeg, totalRead, processedSize - totalRead, cancellationToken);
                            if (chunk == 0) throw new IOException("visionStream closed during processed JPEG read");
                            totalRead += chunk;
                        }

                        // Step 6: Display the processed JPEG
                        using var ms = new MemoryStream(processedJpeg);
                        var bitmap = new Bitmap(ms); // Decode JPEG

                        // Update UI on the UI thread
                        await Dispatcher.UIThread.InvokeAsync(() =>
                        {
                            _cameraImage.Source = bitmap;
                        });
                    }
                    catch (OperationCanceledException) { Console.WriteLine("[ProcessingLoop] Cancellation requested during frame processing."); break; }
                    catch (IOException ioEx) { Console.WriteLine($"[ProcessingLoop VISION STREAM ERROR] {ioEx.Message}"); break; } // Assume visionStream connection is lost, exit loop
                    catch (Exception ex) { Console.WriteLine($"[ProcessingLoop FRAME ERROR] {ex.GetType().Name}: {ex.Message}"); } // Log other errors, continue loop
                } // End foreach loop
            }
            catch (OperationCanceledException) { Console.WriteLine("[ProcessingLoop] Cancellation requested while waiting for channel items."); }
            catch (ChannelClosedException) { Console.WriteLine("[ProcessingLoop] Processing channel was closed."); } // Handle channel closing gracefully
            catch (Exception ex) { Console.WriteLine($"[ProcessingLoop UNEXPECTED ERROR] {ex.Message}"); } // Catch unexpected errors in the await foreach
            finally { Console.WriteLine("[ProcessingLoop] Exiting loop."); }
            Console.WriteLine("[ProcessingLoop] Ended.");
        }

        // --- Send Command ---
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

                // Unsubscribe from events
                //if (SignalController.Instance != null)
                //{ // Check if instance exists
                //    SignalController.Instance.ModeChanged -= SetModeAsync();
                //}

                Console.WriteLine("[DisposeAsyncCore] Async cleanup finished.");
            }
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

                    // Other strictly synchronous managed resource cleanup can go here,
                    // but most cleanup is handled by StopCameraFeedAsync called via DisposeAsyncCore.
                    Console.WriteLine("[Dispose] Managed resource disposal attempted.");
                }

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