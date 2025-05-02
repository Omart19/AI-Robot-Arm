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
    public sealed class SignalController // Make sealed if only one instance intended
    {
        // Singleton Pattern
        private static readonly Lazy<SignalController> lazy =
            new Lazy<SignalController>(() => new SignalController());
        public static SignalController Instance => lazy.Value;
        private SignalController() { } // Private constructor

        // --- Define Events ---
        public event Action<bool>? ModeChanged;
        public event Action<int[]>? EncodersReceived;
        public event Action<byte[]>? RawFrameReceived;       // Optional: If needed elsewhere
        public event Action<byte[]>? ProcessedFrameReceived; // Optional: If needed elsewhere

        // --- Methods to Raise Events ---
        public void RaiseModeChanged(bool isRemote) => ModeChanged?.Invoke(isRemote);

        // Inside SignalController.cs
        public void RaiseEncodersReceived(int[] encoderValues)
        {
            var handler = EncodersReceived; // Capture locally for thread safety
            if (handler != null)
            {
                // ---> ADD LOGGING HERE <---
                Console.WriteLine($"[SignalController] Invoking EncodersReceived for {handler.GetInvocationList().Length} subscribers.");
                handler.Invoke(encoderValues);
            }
            else
            {
                // ---> ADD LOGGING HERE <---
                Console.WriteLine("[SignalController] No subscribers for EncodersReceived.");
            }
        }
        public void RaiseRawFrameReceived(byte[] frameBytes) => RawFrameReceived?.Invoke(frameBytes);

        public void RaiseProcessedFrameReceived(byte[] frameBytes) => ProcessedFrameReceived?.Invoke(frameBytes);

        // No network clients, streams, loops, or processing logic here!
        // No IDisposable needed unless events themselves hold resources indirectly.
    }
}