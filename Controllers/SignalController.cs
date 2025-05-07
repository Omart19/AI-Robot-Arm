// SignalController.cs

using System;
using System.Collections.Generic;
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
        public event Action<List<DetectionResult>?>? DetectionsReceived; // <<< NEW EVENT >>>


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

        public void RaiseDetectionsReceived(List<DetectionResult>? detections)
        {
            // 1. Notify external subscribers
            var handler = DetectionsReceived;
            if (handler != null)
            {
                Console.WriteLine($"[SignalController] Invoking DetectionsReceived for {handler.GetInvocationList().Length} external subscribers with {detections?.Count ?? 0} detections.");
                try
                {
                    handler.Invoke(detections);
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"[SignalController] Error invoking external DetectionsReceived subscriber: {ex.Message}");
                    // Decide if you want to continue processing or re-throw
                }
            }
            else
            {
                Console.WriteLine("[SignalController] No external subscribers for DetectionsReceived.");
            }

            // --- 2. Perform Integrated Detection Processing ---
            // This logic is now moved from DetectionProcessorService
            // It will run on the same thread that called RaiseDetectionsReceived
            // (which is the UI thread via CameraController's timer).
            if (detections == null || !detections.Any())
            {
                // Console.WriteLine("[SignalController-IntegratedProcessing] Received no detections or null list for internal processing.");
                return;
            }

            Console.WriteLine($"[SignalController-IntegratedProcessing] Processing {detections.Count} detections internally:");
            try
            {
                foreach (var detection in detections)
                {
                    Console.WriteLine($"  - Label: {detection.Label}, Conf: {detection.Confidence:P1}, Box: [{string.Join(",", detection.Box)}]");

                    // Example integrated logic:
                    if (detection.Label == "person" && detection.Confidence > 0.75)
                    {
                        Console.WriteLine($"    -> Person detected with high confidence! Box: {string.Join(",", detection.Box)} (Processed by SignalController)");
                        // TriggerAnotherActionInternally(detection.Box);
                    }
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[SignalController-IntegratedProcessing] Error during internal detection processing: {ex.Message}");
            }
        }
        public void RaiseRawFrameReceived(byte[] frameBytes) => RawFrameReceived?.Invoke(frameBytes);

        public void RaiseProcessedFrameReceived(byte[] frameBytes) => ProcessedFrameReceived?.Invoke(frameBytes);

        
        // No network clients, streams, loops, or processing logic here!
        // No IDisposable needed unless events themselves hold resources indirectly.
    }
}