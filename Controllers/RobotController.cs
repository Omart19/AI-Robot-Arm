// RobotAIArm9/Controllers/RobotController.cs

using Avalonia.Controls; // Note: This using might not be needed in RobotController.cs
using Avalonia.Media.Imaging; // Note: This using might not be needed in RobotController.cs
using Avalonia.Threading; // Note: This using might not be needed in RobotController.cs
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
        private const int FrameWidth = 1280;
        private const int FrameHeight = 720;
        // private const double CenterThresholdFactor = 0.01; // Not used if pixelDeadZone is used directly
        private const double ConfidenceThreshold = 0.6;
        private const int PanServoID = 2;
        private const int TiltServoID = 3;
        private const int CenterAngle = 90;
        private const int MinAngle = 0;
        private const int MaxAngle = 180;
        private const int PanStepDegrees = 1;
        private const int TiltStepDegrees = 1;
        private enum TrackingFocus { None, Person, Object }
        private enum ControllerState
        {
            SearchingForPerson,
            CenteringPerson,
            MonitoringCenteredPerson,
            SearchingForObject,
            CenteringObject,
            ObjectIsCentered
        }

        private ControllerState _currentState = ControllerState.SearchingForPerson;
        private TrackingFocus _currentFocus = TrackingFocus.Person;

        private DateTime _personCenteredTimestamp = DateTime.MinValue;
        // private readonly TimeSpan _personCenteredDurationRequirement = TimeSpan.FromSeconds(5.0); // Replaced by _currentMonitoringDuration
        private DetectionResult? _lastTrackedPerson = null;
        private DetectionResult? _lastTrackedObject = null;
        private readonly string _priorityObjectName = "bottle";

        private Random _rng = new Random();
        private TimeSpan _currentMonitoringDuration = TimeSpan.FromSeconds(5);

        private const double pixelDeadZone = 10;

        // --- State Tracking ---
        private int _currentPanAngle = CenterAngle;
        private int _currentTiltAngle = CenterAngle;
        private bool _isMoving = false;

        private DateTime _lastRobotCommandBatchTime = DateTime.MinValue;
        private readonly TimeSpan _robotCommandCycleInterval = TimeSpan.FromSeconds(0.1);

        public RobotController(ArduinoController arduinoController)
        {
            _arduinoController = arduinoController ?? throw new ArgumentNullException(nameof(arduinoController));
            SignalController.Instance.DetectionsReceived += OnDetectionsReceived;
            Console.WriteLine("[RobotController] Initialized and subscribed to DetectionsReceived.");
        }

        private async Task InitializeServos()
        {
            await Task.Delay(500);
            Console.WriteLine("[RobotController] Centering Servos...");
            // Assuming SendServoCommand now returns Task, if not, remove await or adjust
            await SendServoCommand(PanServoID, CenterAngle);
            await Task.Delay(50);
            await SendServoCommand(TiltServoID, CenterAngle);
            _currentPanAngle = CenterAngle;
            _currentTiltAngle = CenterAngle;
        }

        private bool IsTargetEffectivelyCentered(DetectionResult? target)
        {
            if (target?.Box == null || target.Box.Count != 4) return false;

            int x1 = target.Box[0]; int y1 = target.Box[1];
            int x2 = target.Box[2]; int y2 = target.Box[3];
            double boxCenterX = x1 + (x2 - x1) / 2.0;
            double boxCenterY = y1 + (y2 - y1) / 2.0;

            double imageCenterX = FrameWidth / 2.0;
            double imageCenterY = FrameHeight / 2.0;

            double horizontalDifference = boxCenterX - imageCenterX;
            double verticalDifference = boxCenterY - imageCenterY;

            return Math.Abs(horizontalDifference) <= pixelDeadZone &&
                   Math.Abs(verticalDifference) <= pixelDeadZone;
        }

        private void OnDetectionsReceived(List<DetectionResult>? detections)
        {
            if (_isDisposed) return;

            DetectionResult? currentPerson = detections?.FirstOrDefault(d => d.Label == "person" && d.Confidence >= ConfidenceThreshold);
            DetectionResult? currentObject = detections?.FirstOrDefault(d => d.Label == _priorityObjectName && d.Confidence >= ConfidenceThreshold);

            if (currentPerson != null) _lastTrackedPerson = currentPerson;
            if (currentObject != null) _lastTrackedObject = currentObject;

            bool isPersonCurrentlyCentered = IsTargetEffectivelyCentered(currentPerson);
            bool isObjectCurrentlyCentered = IsTargetEffectivelyCentered(currentObject);

            switch (_currentState)
            {
                case ControllerState.SearchingForPerson:
                    _currentFocus = TrackingFocus.Person;
                    if (currentPerson != null)
                    {
                        Console.WriteLine($"[RobotController SM] Person detected. State: CenteringPerson. Target: {currentPerson.Label}");
                        _currentState = ControllerState.CenteringPerson;
                        MoveTowardsTarget(currentPerson);
                    }
                    else if (currentObject != null) // MODIFICATION: If no person, look for object
                    {
                        Console.WriteLine($"[RobotController SM] No person found, object '{_priorityObjectName}' detected. State: CenteringObject.");
                        _currentState = ControllerState.CenteringObject;
                        MoveTowardsTarget(currentObject);
                    }
                    else
                    {
                        Console.WriteLine("[RobotController SM] SearchingForPerson: No person or priority object found.");
                        _isMoving = false;
                    }
                    break;

                case ControllerState.CenteringPerson:
                    _currentFocus = TrackingFocus.Person;
                    if (currentPerson != null)
                    {
                        MoveTowardsTarget(currentPerson);
                        if (isPersonCurrentlyCentered)
                        {
                            _personCenteredTimestamp = DateTime.UtcNow;
                            _currentMonitoringDuration = TimeSpan.FromSeconds(_rng.Next(5, 10)); // Random 5-9 seconds
                            Console.WriteLine($"[RobotController SM] Person centered. State: MonitoringCenteredPerson for {_currentMonitoringDuration.TotalSeconds}s.");
                            _currentState = ControllerState.MonitoringCenteredPerson;
                        }
                        _isMoving = true;
                    }
                    else
                    {
                        Console.WriteLine("[RobotController SM] Person lost while CenteringPerson. State: SearchingForPerson.");
                        _currentState = ControllerState.SearchingForPerson;
                        _isMoving = false;
                    }
                    break;

                case ControllerState.MonitoringCenteredPerson:
                    _currentFocus = TrackingFocus.Person;
                    if (currentPerson != null) // Person still detected
                    {
                        if (isPersonCurrentlyCentered) // Is person STILL centered?
                        {
                            // Actively try to keep the person centered during monitoring
                            MoveTowardsTarget(currentPerson);
                            _isMoving = true; // Set based on whether MoveTowardsTarget implies movement

                            if (DateTime.UtcNow - _personCenteredTimestamp >= _currentMonitoringDuration)
                            {
                                Console.WriteLine($"[RobotController SM] Person monitored as centered for {_currentMonitoringDuration.TotalSeconds}s. State: SearchingForObject.");
                                _currentState = ControllerState.SearchingForObject;
                                // Attempt to look for object in the same cycle if visible
                                if (currentObject != null)
                                {
                                    Console.WriteLine($"[RobotController SM] Object '{_priorityObjectName}' detected. State: CenteringObject.");
                                    _currentState = ControllerState.CenteringObject;
                                    MoveTowardsTarget(currentObject);
                                }
                                else
                                {
                                    _isMoving = false; // No object to move to yet
                                }
                            }
                            else
                            {
                                Console.WriteLine($"[RobotController SM] MonitoringCenteredPerson: Person still centered. Waiting for timer: {(DateTime.UtcNow - _personCenteredTimestamp).TotalSeconds:F1}s / {_currentMonitoringDuration.TotalSeconds:F1}s");
                            }
                        }
                        else // Person moved from center
                        {
                            Console.WriteLine("[RobotController SM] Person moved from center during Monitoring. State: CenteringPerson.");
                            _currentState = ControllerState.CenteringPerson; // Re-center the person
                            MoveTowardsTarget(currentPerson);
                            _isMoving = true;
                        }
                    }
                    else // Person lost entirely
                    {
                        Console.WriteLine("[RobotController SM] Person lost during Monitoring. State: SearchingForPerson.");
                        _currentState = ControllerState.SearchingForPerson;
                        _isMoving = false;
                    }
                    break;

                case ControllerState.SearchingForObject:
                    _currentFocus = TrackingFocus.Object;
                    if (currentPerson != null) // Person always takes priority if seen
                    {
                        Console.WriteLine($"[RobotController SM] Person detected while SearchingForObject. Prioritizing. State: CenteringPerson.");
                        _currentState = ControllerState.CenteringPerson;
                        MoveTowardsTarget(currentPerson);
                        _isMoving = true;
                        break;
                    }

                    if (currentObject != null)
                    {
                        Console.WriteLine($"[RobotController SM] Object '{_priorityObjectName}' detected. State: CenteringObject. Target: {currentObject.Label}");
                        _currentState = ControllerState.CenteringObject;
                        MoveTowardsTarget(currentObject);
                        _isMoving = true;
                    }
                    else
                    {
                        Console.WriteLine($"[RobotController SM] SearchingForObject: No '{_priorityObjectName}' found. Returning to SearchingForPerson.");
                        _currentState = ControllerState.SearchingForPerson; // Default back to person search if no object
                        _isMoving = false;
                    }
                    break;

                case ControllerState.CenteringObject:
                    _currentFocus = TrackingFocus.Object;
                    if (currentPerson != null) // Person always takes priority
                    {
                        Console.WriteLine($"[RobotController SM] Person detected while CenteringObject. Prioritizing. State: CenteringPerson.");
                        _currentState = ControllerState.CenteringPerson;
                        MoveTowardsTarget(currentPerson);
                        _isMoving = true;
                        break;
                    }

                    if (currentObject != null)
                    {
                        MoveTowardsTarget(currentObject);
                        if (isObjectCurrentlyCentered)
                        {
                            Console.WriteLine($"[RobotController SM] Object '{_priorityObjectName}' centered. State: ObjectIsCentered.");
                            _currentState = ControllerState.ObjectIsCentered;
                        }
                        _isMoving = true;
                    }
                    else
                    {
                        Console.WriteLine($"[RobotController SM] Object '{_priorityObjectName}' lost while CenteringObject. State: SearchingForObject.");
                        _currentState = ControllerState.SearchingForObject;
                        _isMoving = false;
                    }
                    break;

                case ControllerState.ObjectIsCentered:
                    _currentFocus = TrackingFocus.Object;
                    Console.WriteLine($"[RobotController SM] Object '{_priorityObjectName}' is centered.");
                    if (currentPerson != null)
                    {
                        Console.WriteLine($"[RobotController SM] Person appeared while object centered. Switching to person.");
                        _currentState = ControllerState.CenteringPerson;
                        MoveTowardsTarget(currentPerson);
                        _isMoving = true;
                    }
                    else if (currentObject == null || !isObjectCurrentlyCentered)
                    {
                        Console.WriteLine($"[RobotController SM] Object lost or moved from center. Switching to SearchObject.");
                        _currentState = ControllerState.SearchingForObject;
                        _isMoving = false;
                    }
                    else
                    {
                        // Object still there and centered, keep monitoring it or decide to switch back to person search.
                        // For now, let's make it try to find a person again after a brief pause or next cycle.
                        // This could also be a timed "hold object" state.
                        Console.WriteLine($"[RobotController SM] Object still centered. Reverting to SearchingForPerson to check for person again.");
                        _currentState = ControllerState.SearchingForPerson;
                        _isMoving = false;
                    }
                    break;
            }
        }

        private void MoveTowardsTarget(DetectionResult target)
        {
            if (target.Box == null || target.Box.Count != 4) return;

            int x1 = target.Box[0]; int y1 = target.Box[1];
            int x2 = target.Box[2]; int y2 = target.Box[3];
            double boxCenterX = x1 + (x2 - x1) / 2.0;
            double boxCenterY = y1 + (y2 - y1) / 2.0;
            double imageCenterX = FrameWidth / 2.0;
            double imageCenterY = FrameHeight / 2.0;
            double horizontalDifference = boxCenterX - imageCenterX;
            double verticalDifference = boxCenterY - imageCenterY;

            int calculatedTargetPanAngle = _currentPanAngle;
            if (horizontalDifference < -pixelDeadZone) { calculatedTargetPanAngle = _currentPanAngle + PanStepDegrees; }
            else if (horizontalDifference > pixelDeadZone) { calculatedTargetPanAngle = _currentPanAngle - PanStepDegrees; }
            calculatedTargetPanAngle = Math.Clamp(calculatedTargetPanAngle, MinAngle, MaxAngle);

            int calculatedTargetTiltAngle = _currentTiltAngle;
            if (verticalDifference < -pixelDeadZone) { calculatedTargetTiltAngle = _currentTiltAngle + TiltStepDegrees; }
            else if (verticalDifference > pixelDeadZone) { calculatedTargetTiltAngle = _currentTiltAngle - TiltStepDegrees; }
            calculatedTargetTiltAngle = Math.Clamp(calculatedTargetTiltAngle, MinAngle, MaxAngle);

            bool panAdjustmentCalculated = (calculatedTargetPanAngle != _currentPanAngle);
            bool tiltAdjustmentCalculated = (calculatedTargetTiltAngle != _currentTiltAngle);
            bool anyMovementCalculatedAsNecessary = panAdjustmentCalculated || tiltAdjustmentCalculated;

            bool commandActuallySentThisCycle = false;
            bool waitingForGlobalInterval = false;

            if (anyMovementCalculatedAsNecessary)
            {
                if (DateTime.UtcNow - _lastRobotCommandBatchTime >= _robotCommandCycleInterval)
                {
                    if (panAdjustmentCalculated)
                    {
                        Console.WriteLine($"[RobotController] Adjusting Pan from {_currentPanAngle} to {calculatedTargetPanAngle} (HDiff: {horizontalDifference:F2})");
                        _ = SendServoCommand(PanServoID, calculatedTargetPanAngle);
                        _currentPanAngle = calculatedTargetPanAngle;
                        commandActuallySentThisCycle = true;
                    }
                    if (tiltAdjustmentCalculated)
                    {
                        Console.WriteLine($"[RobotController] Adjusting Tilt from {_currentTiltAngle} to {calculatedTargetTiltAngle} (VDiff: {verticalDifference:F2})");
                        _ = SendServoCommand(TiltServoID, calculatedTargetTiltAngle);
                        _currentTiltAngle = calculatedTargetTiltAngle;
                        commandActuallySentThisCycle = true;
                    }

                    if (commandActuallySentThisCycle)
                    {
                        _lastRobotCommandBatchTime = DateTime.UtcNow;
                    }
                }
                else
                {
                    waitingForGlobalInterval = true;
                }
            }

            LogAlignmentStatus(target, horizontalDifference, verticalDifference,
                               calculatedTargetPanAngle, calculatedTargetTiltAngle,
                               commandActuallySentThisCycle,
                               anyMovementCalculatedAsNecessary,
                               waitingForGlobalInterval);
        }

        private void LogAlignmentStatus(DetectionResult target, double hDiff, double vDiff,
                                        int calcPan, int calcTilt,
                                        bool commandSentThisCycle,
                                        bool anyMovementCalculated,
                                        bool isWaitingForGlobalInterval)
        {
            bool isPhysicallyAlignedNow = Math.Abs(hDiff) <= pixelDeadZone && Math.Abs(vDiff) <= pixelDeadZone;

            if (isPhysicallyAlignedNow)
            {
                Console.WriteLine($"[RobotController] Target ({target.Label}) is ALIGNED (DEAD CENTER). (HDiff: {hDiff:F2}, VDiff: {vDiff:F2})");
            }
            else if (anyMovementCalculated && calcPan == _currentPanAngle && calcTilt == _currentTiltAngle && !commandSentThisCycle && !isWaitingForGlobalInterval)
            {
                Console.WriteLine($"[RobotController] Target ({target.Label}) AT LIMIT, cannot improve alignment. (HDiff: {hDiff:F2}, VDiff: {vDiff:F2})");
            }
            else if (isWaitingForGlobalInterval)
            {
                Console.WriteLine($"[RobotController] Target ({target.Label}) requires adjustment (Desired P:{calcPan}, T:{calcTilt}), waiting for GLOBAL command interval.");
            }
        }

        // Marked SendServoCommand as returning Task as it calls an async method, though it's fire-and-forget

        private Task SendServoCommand(int servoId, int angle)
        {
            if (_isDisposed) return Task.CompletedTask;
            string command = $"servo{servoId}{angle}";
            Console.WriteLine($"[RobotController] Sending command: {command}");
            _arduinoController.ClearArduinoCommandQueue();
            _arduinoController.EnqueueArduinoCommand(command); // This should be Task or void
            //_arduinoController.ClearArduinoCommandQueue();
            return Task.CompletedTask; // Return a completed task if EnqueueArduinoCommand is void
                                       // Or return the Task from EnqueueArduinoCommand if it's async Task
        }
        //private Task SendServoCommand(int servoId, int angle)
        //{
        //    _arduinoController.ClearArduinoCommandQueue();
        //    if (_isDisposed) return Task.CompletedTask;
        //    string command = $"servo{servoId}{angle}";
        //    Console.WriteLine($"[RobotController] Sending command: {command}");
        //    _arduinoController.EnqueueArduinoCommand(command); // This should be Task or void
        //    return Task.CompletedTask; // Return a completed task if EnqueueArduinoCommand is void
        //                               // Or return the Task from EnqueueArduinoCommand if it's async Task
        //}

        public void Dispose()
        {
            if (!_isDisposed)
            {
                Console.WriteLine("[RobotController] Disposing...");
                _isDisposed = true;
                SignalController.Instance.DetectionsReceived -= OnDetectionsReceived;
                Console.WriteLine("[RobotController] Unsubscribed from DetectionsReceived.");
            }
        }
    }
}