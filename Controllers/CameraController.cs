using Avalonia.Controls;
using Avalonia.Media.Imaging;
using Avalonia.Threading;
using Dock.Model.Avalonia.Core;
using Dock.Model.Core;
using System;
using System.Diagnostics;
using System.IO;
using System.Threading;
using System.Threading.Tasks;

namespace RobotAIArm.Controllers
{
    public class CameraController : DockWindow
    {
        private Image _cameraImage;
        private Process? _cameraProcess;
        private CancellationTokenSource? _cancellationTokenSource;

        public CameraController(Image cameraImage)
        {
            _cameraImage = cameraImage;
            _cancellationTokenSource = new CancellationTokenSource();
        }

        public async Task StartCameraFeed()
        {
            // Offload to a background thread
            await Task.Run(async () => 
            {
                try
                {
                    var startInfo = new ProcessStartInfo
                    {
                        FileName = "libcamera-vid",
                        Arguments = "-t 0 --codec mjpeg -o - --width 1536 --height 864 --framerate 60 --nopreview",
                        RedirectStandardOutput = true,
                        UseShellExecute = false,
                        CreateNoWindow = true
                    };

                    _cameraProcess = new Process();
                    _cameraProcess.StartInfo = startInfo;
                    _cameraProcess.Start();

                    await ProcessCameraStream(_cameraProcess.StandardOutput.BaseStream, _cancellationTokenSource.Token);
                }
                catch (Exception ex)
                {
                    // Handle exceptions (e.g., log, display error message)
                    Console.WriteLine($"Error starting camera feed: {ex.Message}");
                    // Consider using Dispatcher.UIThread.InvokeAsync to update the UI with an error message
                }
            });
        }

        private async Task ProcessCameraStream(Stream stream, CancellationToken cancellationToken)
        {
            try
            {
                var buffer = new byte[2];

                while (!cancellationToken.IsCancellationRequested)
                {
                    var ms = new MemoryStream();
                    bool frameStartFound = false;

                    // Read until frame start is found
                    while (!frameStartFound && !cancellationToken.IsCancellationRequested)
                    {
                        var bytesRead = await stream.ReadAsync(buffer, 0, buffer.Length, cancellationToken);
                        if (bytesRead == 0)
                        {
                            await Task.Delay(10, cancellationToken);
                            continue;
                        }

                        for (int i = 0; i < bytesRead - 1; i++)
                        {
                            if (buffer[i] == 0xFF && buffer[i + 1] == 0xD8) // JPEG start marker
                            {
                                ms.Write(buffer, i, bytesRead - i);
                                frameStartFound = true;
                                break;
                            }
                        }
                    }

                    // Read until frame end is found
                    while (frameStartFound && !cancellationToken.IsCancellationRequested)
                    {
                        var bytesRead = await stream.ReadAsync(buffer, 0, buffer.Length, cancellationToken);
                        if (bytesRead == 0)
                        {
                            await Task.Delay(10, cancellationToken);
                            continue;
                        }

                        ms.Write(buffer, 0, bytesRead);

                        for (int i = 0; i < bytesRead - 1; i++)
                        {
                            if (buffer[i] == 0xFF && buffer[i + 1] == 0xD9) // JPEG end marker
                            {
                                ms.Position = 0;
                                var bitmap = new Bitmap(ms);

                                // Dispatch to UI thread
                                await Dispatcher.UIThread.InvokeAsync(() =>
                                {
                                    if (!cancellationToken.IsCancellationRequested)
                                    {
                                        _cameraImage.Source = bitmap;
                                    }
                                }, DispatcherPriority.Render);

                                frameStartFound = false;
                                break;
                            }
                        }
                    }
                }
            }
            catch (OperationCanceledException)
            {
                Console.WriteLine("Camera stream processing was cancelled.");
            }
            catch (Exception ex)
            {
                // Handle exceptions (e.g., log, display error message)
                Console.WriteLine($"Error processing camera stream: {ex.Message}");
                // Consider using Dispatcher.UIThread.InvokeAsync to update the UI with an error message
            }
            finally
            {
                stream.Close();
            }
        }

        public void StopCameraFeed()
        {
            _cancellationTokenSource?.Cancel();

            if (_cameraProcess != null && !_cameraProcess.HasExited)
            {
                try
                {
                    _cameraProcess.Kill();
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"Error killing camera process: {ex.Message}");
                }
            }

            _cameraProcess?.Dispose();
            _cancellationTokenSource?.Dispose();
            _cancellationTokenSource = new CancellationTokenSource(); // Reset cancellation token source for potential reuse
        }
    }
}