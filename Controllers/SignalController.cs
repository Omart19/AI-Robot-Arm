// SignalController.cs

using System;
using System.IO;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using Tmds.DBus.Protocol;

namespace RobotAIArm.Controllers
{
    public class SignalController : IDisposable
    {
        private static SignalController? _instance;
        public static SignalController Instance => _instance ??= new SignalController();

        private TcpClient? _client;
        private NetworkStream? _stream;
        private CancellationTokenSource? _cts;

        private TcpClient? _visionClient;
        private NetworkStream? _visionStream;
        private Socket? _visionSocket;

        public event Action<byte[]>? FrameReceived;
        public event Action<int[]>? EncodersReceived;

        private string _serverIp => "127.0.0.1";

        public event Action<bool>? ModeChanged;

        public void RaiseFrameReceived(byte[] frameBytes)
        {
            FrameReceived?.Invoke(frameBytes);
        }

        public void RaiseModeChanged(bool isRemote)
        {
            ModeChanged?.Invoke(isRemote);
        }
        private SignalController() { }

        //public void SetServerIp(string ip)
        //{
        //    _serverIp = ip;
        //}


        public async Task StartAsync()
        {
            _cts = new CancellationTokenSource();
            await ConnectAsync();
        }

        private async Task ConnectAsync()
        {
            while (!_cts!.IsCancellationRequested)
            {
                try
                {
                    Console.WriteLine($"[SignalController] Connecting to {AppSettings.Instance.RemoteIP}:23456...");
                    _client = new TcpClient();
                    await _client.ConnectAsync(IPAddress.Parse(AppSettings.Instance.RemoteIP), 23456);
                    _stream = _client.GetStream();
                    Console.WriteLine("[SignalController] Connected!");

                    await ReceiveLoopAsync();
                }
                catch (Exception ex)
                {
                    Console.WriteLine("[SignalController] Connect Error: " + ex.Message);
                    await Task.Delay(2000);
                }
            }
        }
        
        public async Task<byte[]?> SendFrameToVisionAndGetProcessed(byte[] originalFrame)
        {
            try
            {
                if (_visionClient == null || !_visionClient.Connected)
                {
                    Console.WriteLine("[SignalController] Connecting to Vision.py...");
                    _visionClient = new TcpClient();
                    await _visionClient.ConnectAsync("127.0.0.1", 34567);  // Vision.py should listen on localhost:34567
                    _visionStream = _visionClient.GetStream();
                    Console.WriteLine("[SignalController] Connected to Vision.py");
                }

                // --- Send frame ---
                byte[] sizeBytes = BitConverter.GetBytes(originalFrame.Length);
                if (BitConverter.IsLittleEndian)
                    Array.Reverse(sizeBytes); // Send size in big-endian

                await _visionStream!.WriteAsync(sizeBytes, 0, 4);
                await _visionStream!.WriteAsync(originalFrame, 0, originalFrame.Length);

                // --- Receive processed frame ---
                byte[] responseSizeBytes = new byte[4];
                int read = await _visionStream.ReadAsync(responseSizeBytes, 0, 4);
                if (read != 4)
                    throw new Exception("Failed to read processed frame size");

                if (BitConverter.IsLittleEndian)
                    Array.Reverse(responseSizeBytes);

                int processedSize = BitConverter.ToInt32(responseSizeBytes, 0);
                if (processedSize <= 0 || processedSize > 50_000_000)
                    throw new Exception($"Invalid processed frame size: {processedSize}");

                byte[] processedFrame = new byte[processedSize];
                int totalRead = 0;
                while (totalRead < processedSize)
                {
                    int chunk = await _visionStream.ReadAsync(processedFrame, totalRead, processedSize - totalRead);
                    if (chunk == 0)
                        throw new Exception("Connection closed while receiving processed frame");
                    totalRead += chunk;
                }

                return processedFrame;
            }
            catch (Exception ex)
            {
                Console.WriteLine("[SignalController] Error: " + ex.Message);
                return null;
            }
        }

        private async Task EnsureVisionConnectionAsync()
        {
            if (_visionClient != null && _visionClient.Connected)
                return;

            Console.WriteLine("[SignalController] Connecting to Vision.py...");

            _visionClient = new TcpClient();
            int retries = 0;

            while (!_visionClient.Connected && retries < 30)
            {
                try
                {
                    await _visionClient.ConnectAsync(IPAddress.Loopback, 34567);
                }
                catch (SocketException)
                {
                    retries++;
                    Console.WriteLine($"[SignalController] Vision.py not ready yet. Retrying... ({retries})");
                    await Task.Delay(500);
                }
            }

            if (!_visionClient.Connected)
            {
                Console.WriteLine("[SignalController] Failed to connect to Vision.py after retries.");
            }
            else
            {
                _visionStream = _visionClient.GetStream();
                Console.WriteLine("[SignalController] Connected to Vision.py!");

                // Wait for "READY\n"
                using var reader = new StreamReader(_visionStream, Encoding.UTF8, leaveOpen: true);
                var readyLine = await reader.ReadLineAsync();
                if (readyLine != "READY")
                {
                    throw new Exception("[SignalController] Vision.py did not send READY.");
                }
                Console.WriteLine("[SignalController] Vision.py is ready to process frames.");
            }
        }


        private async Task ReceiveLoopAsync()
        {
            var buffer = new byte[4];
            while (!_cts!.IsCancellationRequested)
            {
                try
                {
                    int read = await _stream!.ReadAsync(buffer, 0, 4);
                    if (read != 4)
                        throw new Exception("Failed to read packet size.");

                    int packetSize = BitConverter.ToInt32(buffer.Reverse().ToArray(), 0);
                    if (packetSize <= 0 || packetSize > 50_000_000)
                        throw new Exception("Invalid packet size: " + packetSize);

                    byte[] packetBuffer = new byte[packetSize];
                    int bytesRead = 0;
                    while (bytesRead < packetSize)
                    {
                        int chunk = await _stream.ReadAsync(packetBuffer, bytesRead, packetSize - bytesRead);
                        if (chunk == 0)
                            throw new Exception("Socket closed during packet read.");
                        bytesRead += chunk;
                    }

                    // Packet = EncoderData\n + JPEG
                    int split = Array.IndexOf(packetBuffer, (byte)'\n');
                    if (split == -1)
                        throw new Exception("Invalid packet format.");

                    string encoderText = Encoding.UTF8.GetString(packetBuffer, 0, split);
                    byte[] jpegBytes = packetBuffer.Skip(split + 1).ToArray();

                    ParseAndRaiseEvents(encoderText, jpegBytes);
                }
                catch (Exception ex)
                {
                    Console.WriteLine("[SignalController] Receive error: " + ex.Message);
                    break;
                }
            }
        }


        private void ParseAndRaiseEvents(string encoderText, byte[] jpegBytes)
        {
            if (encoderText.StartsWith("ENCODERS:"))
            {
                var parts = encoderText.Substring(9).Split(',');
                if (parts.Length == 4 && int.TryParse(parts[0], out int e0))
                {
                    int[] encoders = parts.Select(p =>
                    {
                        if (int.TryParse(p, out int v)) return v;
                        else return -1;
                    }).ToArray();
                    EncodersReceived?.Invoke(encoders);
                }
            }

            FrameReceived?.Invoke(jpegBytes);
        }

        public void Dispose()
        {
            _cts?.Cancel();
            _stream?.Dispose();
            _client?.Dispose();
        }
    }
}
