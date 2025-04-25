using Avalonia.Controls;
using Avalonia.Media.Imaging;
using Avalonia.Threading;
using Dock.Model.Avalonia.Core;
using Dock.Model.Core;
using System;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace RobotAIArm.Controllers
{
    public class CameraController : DockWindow
    {
        private readonly Image _cameraImage;
        private readonly ArduinoController _arduino;
        private Process? _visionProcess;
        private CancellationTokenSource? _cancellationTokenSource;

        private DateTime _lastWaveTime = DateTime.MinValue;
        private DateTime _lastGrabTime = DateTime.MinValue;
        private readonly TimeSpan _waveCooldown = TimeSpan.FromSeconds(120);
        private readonly TimeSpan _grabCooldown = TimeSpan.FromSeconds(60);

        public CameraController(Image cameraImage, ArduinoController arduino)
        {
            _cameraImage = cameraImage;
            _arduino = arduino;
            _cancellationTokenSource = new CancellationTokenSource();
        }

        public async Task StartCameraFeed()
        {
            StartVisionPipelineInline();
            StartImageSocketListener();
        }

        private void StartVisionPipelineInline()
        {
            try
            {
                var psi = new ProcessStartInfo
                {
                    FileName = "bash",
                    Arguments = "-c \"libcamera-vid -t 0 --codec mjpeg -o - --width 1280 --height 720 --framerate 26 --nopreview --mode 2304:1296:SRGGB10 | python3 /home/ergy/Shared/RobotAIArm9/PythonScripts/Vision.py\"",
                    UseShellExecute = false,
                    RedirectStandardOutput = true,
                    RedirectStandardError = true,
                    CreateNoWindow = true
                };

                _visionProcess = new Process { StartInfo = psi };
                _visionProcess.OutputDataReceived += (s, e) => Console.WriteLine("[PIPELINE] " + e.Data);
                _visionProcess.ErrorDataReceived += (s, e) => Console.WriteLine("[PIPELINE ERR] " + e.Data);
                _visionProcess.Start();
                _visionProcess.BeginOutputReadLine();
                _visionProcess.BeginErrorReadLine();

                Console.WriteLine("[INFO] Vision pipeline started.");
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[ERROR] Failed to start vision pipeline: {ex.Message}");
            }
        }

        private void StartImageSocketListener()
        {
            _ = Task.Run(() =>
            {
                try
                {
                    var client = new TcpClient();
                    Thread.Sleep(10000);
                    client.Connect(IPAddress.Loopback, 23456);
                    Console.WriteLine("[SOCKET] Connected to Python vision server.");
                    using var stream = client.GetStream();
                    var reader = new StreamReader(stream, Encoding.UTF8);

                    while (!_cancellationTokenSource!.IsCancellationRequested)
                    {
                        string? labelLine = reader.ReadLine();
                        if (labelLine == null || !labelLine.StartsWith("LABEL:"))
                            continue;

                        var parts = labelLine.Split(':');
                        if (parts.Length >= 2)
                        {
                            var label = parts[1].Trim();
                            HandleDetectedLabel(label);
                        }

                        var sizeBytes = new byte[4];
                        if (stream.Read(sizeBytes, 0, 4) != 4) continue;
                        int jpegSize = BitConverter.ToInt32(sizeBytes.Reverse().ToArray(), 0);
                        if (jpegSize <= 0 || jpegSize > 10_000_000) continue;

                        var jpegBytes = new byte[jpegSize];
                        int bytesRead = 0;
                        while (bytesRead < jpegSize)
                        {
                            int chunk = stream.Read(jpegBytes, bytesRead, jpegSize - bytesRead);
                            if (chunk == 0) break;
                            bytesRead += chunk;
                        }

                        if (bytesRead < jpegSize) continue;

                        try
                        {
                            using var ms = new MemoryStream(jpegBytes);
                            var bitmap = new Bitmap(ms);
                            Dispatcher.UIThread.InvokeAsync(() =>
                            {
                                _cameraImage.Source = bitmap;
                                _cameraImage.InvalidateVisual();
                            });
                        }
                        catch (Exception ex)
                        {
                            Console.WriteLine($"[ERROR] Bitmap render failed: {ex.Message}");
                        }
                    }
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"[SOCKET ERROR] {ex.Message}");
                }
            });
        }

        private void HandleDetectedLabel(string label)
{
    _ = Task.Run(async () =>  // <-- super important: run it separately
    {
        int actionDelay = 1000; // milliseconds between servo commands

        if (label == "person")
        {
            if (DateTime.Now - _lastWaveTime > _waveCooldown)
            {
                _lastWaveTime = DateTime.Now;
                Console.WriteLine("[ACTION] Waving to person...");

                await _arduino.SendCommandAsync("servo145");
                await Task.Delay(actionDelay);
                await _arduino.SendCommandAsync("servo10");
            }
        }
        else if (label == "bottle" || label == "cup" || label == "pottedplant" || label == "chair")
        {
            if (DateTime.Now - _lastGrabTime > _grabCooldown)
            {
                _lastGrabTime = DateTime.Now;
                Console.WriteLine("[ACTION] Attempting grab...");

                await _arduino.SendCommandAsync("moveX1000");
                await Task.Delay(actionDelay);
                await _arduino.SendCommandAsync("moveY500");
                await Task.Delay(actionDelay);
                await _arduino.SendCommandAsync("servo1180");
            }
        }
    });
}

        public void StopCameraFeed()
        {
            _cancellationTokenSource?.Cancel();

            if (_visionProcess != null && !_visionProcess.HasExited)
                _visionProcess.Kill();

            _visionProcess?.Dispose();
            _cancellationTokenSource?.Dispose();
            _cancellationTokenSource = new CancellationTokenSource();
        }
    }
}
