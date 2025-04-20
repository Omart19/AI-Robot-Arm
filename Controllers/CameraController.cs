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
using System.Threading;
using System.Threading.Tasks;

namespace RobotAIArm.Controllers
{
    public class CameraController : DockWindow
    {
        private Image _cameraImage;
        private Process? _visionProcess;
        private CancellationTokenSource? _cancellationTokenSource;

        public CameraController(Image cameraImage)
        {
            _cameraImage = cameraImage;
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
            Arguments = "-c \"libcamera-vid -t 0 --codec mjpeg -o - --width 1280 --height 720 --framerate 30 --nopreview | python3 /home/ergy/Shared/RobotAIArm9/PythonScripts/Vision.py\"",
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
                    Thread.Sleep(10000); // Wait for pipeline startup

                    client.Connect(IPAddress.Loopback, 23456);
                    Console.WriteLine("[SOCKET] Connected to Python vision server.");
                    using var stream = client.GetStream();

                    while (!_cancellationTokenSource!.IsCancellationRequested)
                    {
                        var sizeBytes = new byte[4];
                        int read = stream.Read(sizeBytes, 0, 4);
                        if (read != 4) continue;

                        int jpegSize = BitConverter.ToInt32(sizeBytes.Reverse().ToArray(), 0);
                        if (jpegSize <= 0 || jpegSize > 10_000_000)
                        {
                            Console.WriteLine($"[WARNING] Invalid JPEG size received: {jpegSize}");
                            continue;
                        }

                        var jpegBytes = new byte[jpegSize];
                        int bytesRead = 0;
                        while (bytesRead < jpegSize)
                        {
                            int chunk = stream.Read(jpegBytes, bytesRead, jpegSize - bytesRead);
                            if (chunk == 0) break;
                            bytesRead += chunk;
                        }

                        if (bytesRead < jpegSize)
                        {
                            Console.WriteLine("[ERROR] Incomplete JPEG frame received.");
                            continue;
                        }

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
                        catch (Exception imgEx)
                        {
                            Console.WriteLine($"[ERROR] Bitmap creation failed: {imgEx.Message}");
                        }
                    }
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"[SOCKET ERROR] {ex.Message}");
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
