using RobotAIArm.Controllers; // For SignalController, ArduinoController, DetectionResult
using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;

namespace RobotAIArm.Controllers
{
    public class RobotController : IDisposable
    {
        private readonly ArduinoController _arduinoController;
        private bool _isDisposed = false;

        // --- Configuration (Adjust as needed) ---
        private const int FrameWidth = 1280; // Match camera/processing resolution
        private const int FrameHeight = 720;
        private const double CenterThresholdFactor = 0.01; // Tolerance zone (8% of width/height from center)
        private const double ConfidenceThreshold = 0.6; // Minimum confidence to react
        private const int PanServoID = 2;
        private const int TiltServoID = 2;
        private const int CenterAngle = 90;
        private const int MinAngle = 0;
        private const int MaxAngle = 180;
        private const int PanStepDegrees = 2;  // How many degrees to move per update cycle (horizontal)
        private const int TiltStepDegrees = 2; // How many degrees to move per update cycle (vertical)

        // --- State Tracking ---
        private int _currentPanAngle = CenterAngle;
        private int _currentTiltAngle = CenterAngle;
        private bool _isMoving = false; // Optional: To track if actively moving
        private DateTime _lastPanCommandSentTime = DateTime.MinValue;
        private DateTime _lastTiltCommandSentTime = DateTime.MinValue;
        private readonly TimeSpan _servoCommandInterval = TimeSpan.FromSeconds(1.0); // Send commands at most once per second for each servo

        public RobotController(ArduinoController arduinoController)
        {
            _arduinoController = arduinoController ?? throw new ArgumentNullException(nameof(arduinoController));

            // Subscribe to detection results
            SignalController.Instance.DetectionsReceived += OnDetectionsReceived;
            Console.WriteLine("[RobotController] Initialized and subscribed to DetectionsReceived.");
            // Initialize servos to center?
            // _ = InitializeServos(); // Optional: Fire-and-forget centering
        }

        // Optional: Method to center servos on startup
        private async Task InitializeServos()
        {
            await Task.Delay(500); // Give time for connection
            Console.WriteLine("[RobotController] Centering Servos...");
            await SendServoCommand(PanServoID, CenterAngle);
            await Task.Delay(50); // Small delay between commands
            await SendServoCommand(TiltServoID, CenterAngle);
            _currentPanAngle = CenterAngle;
            _currentTiltAngle = CenterAngle;
        }


        private void OnDetectionsReceived(List<DetectionResult>? detections)
        {
            if (_isDisposed) return;

            DetectionResult? target = SelectTarget(detections);

            if (target != null)
            {
                // Console.WriteLine($"[RobotController] Target acquired: {target.Label} (Conf: {target.Confidence:P1})");
                MoveTowardsTarget(target);
                _isMoving = true; // Mark as moving since we have a target
            }
            else
            {
                 // Console.WriteLine("[RobotController] No priority target found.");
                 // Optionally stop movement if it was moving before and now has no target
                 if (_isMoving)
                 {
                    Console.WriteLine("[RobotController] Target lost, stopping movement.");
                    // No explicit "STOP" command needed if we just stop sending new angle commands
                    _isMoving = false;
                 }
            }
        }

        private DetectionResult? SelectTarget(List<DetectionResult>? detections)
        {
             if (detections == null || !detections.Any()) return null;

             // Prioritize "person"
             var personTarget = detections.FirstOrDefault(d => d.Label == "person" && d.Confidence >= ConfidenceThreshold);
             if (personTarget != null) return personTarget;

             // If no person, look for "bottle"
             var bottleTarget = detections.FirstOrDefault(d => d.Label == "bottle" && d.Confidence >= ConfidenceThreshold);
             if (bottleTarget != null) return bottleTarget;

             // Add other object priorities here...

             return null; // No desired target found
        }


        private void MoveTowardsTarget(DetectionResult target) // If you add awaits for delays, this becomes async Task
        {
            if (target.Box == null || target.Box.Count != 4) return;

            // Calculate center of the bounding box
            int x1 = target.Box[0]; int y1 = target.Box[1];
            int x2 = target.Box[2]; int y2 = target.Box[3];
            double boxCenterX = x1 + (x2 - x1) / 2.0;
            double boxCenterY = y1 + (y2 - y1) / 2.0;

            // Calculate image center
            double imageCenterX = FrameWidth / 2.0;
            double imageCenterY = FrameHeight / 2.0;
            const double pixelDeadZone = 0.75; // Or your preferred value

            // --- Calculate Desired Pan Angle ---
            // This part calculates where we WANT the servo to go based on the current frame
            int calculatedTargetPanAngle = _currentPanAngle; // Start with current physical angle as basis for calculation
            double horizontalDifference = boxCenterX - imageCenterX;

            if (horizontalDifference < -pixelDeadZone)
            {
                calculatedTargetPanAngle = _currentPanAngle + PanStepDegrees;
            }
            else if (horizontalDifference > pixelDeadZone)
            {
                calculatedTargetPanAngle = _currentPanAngle - PanStepDegrees;
            }
            // Clamp the calculated desired angle immediately
            calculatedTargetPanAngle = Math.Clamp(calculatedTargetPanAngle, MinAngle, MaxAngle);

            // --- Calculate Desired Tilt Angle ---
            int calculatedTargetTiltAngle = _currentTiltAngle; // Start with current physical angle
            double verticalDifference = boxCenterY - imageCenterY;

            if (verticalDifference < -pixelDeadZone)
            {
                calculatedTargetTiltAngle = _currentTiltAngle + TiltStepDegrees;
            }
            else if (verticalDifference > pixelDeadZone)
            {
                calculatedTargetTiltAngle = _currentTiltAngle - TiltStepDegrees;
            }
            // Clamp the calculated desired angle immediately
            calculatedTargetTiltAngle = Math.Clamp(calculatedTargetTiltAngle, MinAngle, MaxAngle);

            // --- Send Commands if Angle Changed AND Interval Passed ---
            bool commandActuallySentThisFrame = false;

            // Pan command logic
            // Only consider sending if the calculated target is different from the last commanded position
            if (calculatedTargetPanAngle != _currentPanAngle)
            {
                if (DateTime.UtcNow - _lastPanCommandSentTime >= _servoCommandInterval)
                {
                    Console.WriteLine($"[RobotController] Adjusting Pan from {_currentPanAngle} to {calculatedTargetPanAngle} (HDiff: {horizontalDifference:F2})");
                    _ = SendServoCommand(PanServoID, calculatedTargetPanAngle); // Enqueues the command
                    _currentPanAngle = calculatedTargetPanAngle;         // Update current state to the new commanded angle
                    _lastPanCommandSentTime = DateTime.UtcNow;           // Record time of this command
                    commandActuallySentThisFrame = true;
                }
                // else: An adjustment is desired, but we're waiting for the interval.
                // _currentPanAngle still reflects the last angle we actually commanded.
            }

            // Tilt command logic
            // Only consider sending if the calculated target is different from the last commanded position
            if (calculatedTargetTiltAngle != _currentTiltAngle)
            {
                if (DateTime.UtcNow - _lastTiltCommandSentTime >= _servoCommandInterval)
                {
                    // If your Arduino/ESP struggles with commands sent very close together (even from different SendServoCommand calls
                    // that get rapidly enqueued), you *might* consider a tiny delay here if both pan and tilt are sent.
                    // However, your ArduinoController's queue with its Thread.Sleep(10) might already handle this.
                    // Example: if (commandActuallySentThisFrame) { await Task.Delay(50); } // This would make MoveTowardsTarget async Task

                    Console.WriteLine($"[RobotController] Adjusting Tilt from {_currentTiltAngle} to {calculatedTargetTiltAngle} (VDiff: {verticalDifference:F2})");
                    _ = SendServoCommand(TiltServoID, calculatedTargetTiltAngle); // Enqueues the command
                    _currentTiltAngle = calculatedTargetTiltAngle;          // Update current state
                    _lastTiltCommandSentTime = DateTime.UtcNow;            // Record time of this command
                    commandActuallySentThisFrame = true;
                }
                // else: An adjustment is desired, but we're waiting for the interval.
            }

            // --- Logging Current State ---
            // The target is "DEAD CENTER" if the calculated desired angles (after clamping) are the same as the current servo angles,
            // AND no command was sent (which means it was already at that desired angle, or at a limit preventing change).
            // OR, more simply, if the differences are within the dead zone.
            bool isHorizontallyAligned = Math.Abs(horizontalDifference) <= pixelDeadZone;
            bool isVerticallyAligned = Math.Abs(verticalDifference) <= pixelDeadZone;

            if (isHorizontallyAligned && isVerticallyAligned)
            {
                // This condition means the target's center IS within the pixelDeadZone for both axes.
                // It also implies calculatedTargetPanAngle == _currentPanAngle and calculatedTargetTiltAngle == _currentTiltAngle
                // (unless a command was just sent in this same frame to achieve this alignment).
                Console.WriteLine($"[RobotController] Target ({target.Label}) is ALIGNED (DEAD CENTER). (HDiff: {horizontalDifference:F2}, VDiff: {verticalDifference:F2})");
            }
            else if (calculatedTargetPanAngle == _currentPanAngle && calculatedTargetTiltAngle == _currentTiltAngle && !commandActuallySentThisFrame)
            {
                // This means the desired angles (after clamping) are the same as current angles,
                // implying it's at a limit and cannot achieve better alignment.
                Console.WriteLine($"[RobotController] Target ({target.Label}) AT LIMIT, cannot improve alignment. (HDiff: {horizontalDifference:F2}, VDiff: {verticalDifference:F2})");
            }
            else if (!commandActuallySentThisFrame) // An adjustment is desired based on HDiff/VDiff, but was throttled
            {
                Console.WriteLine($"[RobotController] Target ({target.Label}) requires adjustment (Desired P:{calculatedTargetPanAngle}, T:{calculatedTargetTiltAngle}), waiting for command interval.");
            }
            // If commandActuallySentThisFrame is true, the specific adjustment messages are already printed.
        }
        // Helper to format and send servo command
        private async Task SendServoCommand(int servoId, int angle)
        {
            if (_isDisposed) return;
            string command = $"servo{servoId}{angle}";
            Console.WriteLine($"[RobotController] Sending command: {command}"); // Optional detailed log
            _arduinoController.EnqueueArduinoCommand(command);
        }

        public void Dispose()
        {
            if (!_isDisposed)
            {
                Console.WriteLine("[RobotController] Disposing...");
                _isDisposed = true;
                SignalController.Instance.DetectionsReceived -= OnDetectionsReceived;
                Console.WriteLine("[RobotController] Unsubscribed from DetectionsReceived.");
                // Optionally send commands to reset servos to a safe position?
                // _ = SendServoCommand(PanServoID, CenterAngle);
                // _ = SendServoCommand(TiltServoID, CenterAngle);
            }
        }
    }
}

