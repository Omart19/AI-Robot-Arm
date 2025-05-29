using RobotAIArm.ViewModels.Tools;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Threading.Tasks;

// Assuming DetectionResult, SignalController, ArduinoController are accessible

namespace RobotAIArm.Controllers
{
    public class TrackedObjectMemory
    {
        public string Label { get; private set; }
        public DateTime LastSeenTimestamp { get; set; }
        public float CumulativeConfidence { get; set; } = 0f;
        public int SightingCount { get; set; } = 0;
        public float CumulativeSize { get; set; } = 0f;
        public int PanAngleAtLastCenter { get; set; } = -1;
        public int TiltAngleAtLastCenter { get; set; } = -1;
        public DateTime TimeOfLastCentering { get; set; } = DateTime.MinValue;
        public bool WasEverCentered { get; set; } = false;

        public float AverageConfidence => SightingCount > 0 ? CumulativeConfidence / SightingCount : 0f;
        public float AverageSize => SightingCount > 0 ? CumulativeSize / SightingCount : 0f;

        public TrackedObjectMemory(string label)
        {
            Label = label;
        }

        public void UpdateSighting(DetectionResult detection)
        {
            LastSeenTimestamp = DateTime.UtcNow;
            CumulativeConfidence += detection.Confidence;
            if (detection.Box != null && detection.Box.Count == 4)
            {
                if (detection.Box[2] >= detection.Box[0] && detection.Box[3] >= detection.Box[1])
                {
                    float width = detection.Box[2] - detection.Box[0];
                    float height = detection.Box[3] - detection.Box[1];
                    CumulativeSize += (width * height);
                }
            }
            SightingCount++;
        }

        public void UpdateCenteredLocation(int pan, int tilt)
        {
            PanAngleAtLastCenter = pan;
            TiltAngleAtLastCenter = tilt;
            TimeOfLastCentering = DateTime.UtcNow;
            WasEverCentered = true;
            Console.WriteLine($"[Memory] Updated centered location for {Label}: P={pan}, T={tilt}");
        }
    }

    public class RobotController : IDisposable
    {
        private readonly ArduinoController _arduinoController;
        private bool _isDisposed = false;
        private bool _isHoming = false;

        // --- State for startup homing ---
        private bool _startupHomingRequestedByViewModel = false;
        private Tool2ViewModel? _viewModelForStartupHoming = null;
        private bool _startupHomingAttemptCompleted = false;

        // --- Track last commanded AccelStepper logical positions ---
        private readonly Dictionary<string, int> _currentLogicalAccelStepperPositions;
        private bool _isArmHomedAndSystemReady = false; // New flag

        // --- Constants ---
        // (Ensure all your existing constants like FrameWidth, STEPS_PER_ENCODER_UNIT_X, etc. are here)
        private const double STEPS_PER_ENCODER_UNIT_X = 2.0; // Used for guidance, not direct step calc in new logic
        private const double STEPS_PER_ENCODER_UNIT_Y = 2.5;
        private const double STEPS_PER_ENCODER_UNIT_Z = 2.5;
        private const double STEPS_PER_ENCODER_UNIT_E0 = 1.8;
        private const int ARDUINO_REFERENCE_STEP_COUNT_FOR_90_DEG = 0;
        // ... (other controller constants from your file)
        private const int FrameWidth = 1280;
        private const int FrameHeight = 720;
        private const double ConfidenceThreshold = 0.6;
        private const int PanServoID = 2;
        private const int TiltServoID = 3;
        private const int CenterAngle = 90; // For Pan/Tilt servos
        private const int MinAngle = 0;
        private const int MaxAngle = 180;
        private ControllerState _currentState = ControllerState.SearchingForPerson;
        private DateTime _personCenteredTimestamp = DateTime.MinValue;
        private readonly string _priorityObjectName = "bottle";
        private Random _rng = new Random();
        private TimeSpan _currentMonitoringDuration = TimeSpan.FromSeconds(5);
        private const double pixelDeadZone = 32;
        private int _currentPanAngle = CenterAngle;
        private int _currentTiltAngle = CenterAngle;
        private bool _isMoving = false;
        private bool _isPerformingGreeting = false;
        private DateTime _lastRobotCommandBatchTime = DateTime.MinValue;
        private readonly TimeSpan _robotCommandCycleInterval = TimeSpan.FromSeconds(0.1);
        private Dictionary<string, TrackedObjectMemory> _objectMemories = new Dictionary<string, TrackedObjectMemory>();
        private int _searchObjectAttemptCounter = 0;
        private const int MaxSearchObjectAttempts = 12000;
        private DateTime _timeToIgnorePersonUntil = DateTime.MinValue;
        private readonly TimeSpan _ignorePersonDuration = TimeSpan.FromMinutes(1.0);
        private readonly TimeSpan _briefTrackingLossThreshold = TimeSpan.FromSeconds(1.5);
        private int? _latestTofHandValue = null;
        private int? _latestTofCamValue = null;
        private const int GRIPPER_SERVO_ID = 4; // Example: Assuming servo ID 4 controls the gripper
        private const int GRIPPER_OPEN_ANGLE = 30;  // Example: Angle for gripper to be open
        private const int GRIPPER_CLOSED_ANGLE = 120; // Example: Angle for gripper to be closed to grab a bottle


        private const int PanStepDegrees = 1;
        private const int TiltStepDegrees = 1;
        private bool _startupHomingIsPending = false;
        private Tool2ViewModel? _tool2ViewModelForPendingHoming = null;

        private enum ControllerState
        {
            SearchingForPerson,
            CenteringPerson,
            MonitoringCenteredPerson,
            DedicatedSearchingForObject,
            DedicatedCenteringObject,
            ObjectIsCentered,
            MovingToLastKnownPosition,
            PerformingGreeting
        }



        public RobotController(ArduinoController arduinoController)
        {
            _arduinoController = arduinoController ?? throw new ArgumentNullException(nameof(arduinoController));

            _currentLogicalAccelStepperPositions = new Dictionary<string, int>
        {
            { "X", ARDUINO_REFERENCE_STEP_COUNT_FOR_90_DEG },
            { "Y", ARDUINO_REFERENCE_STEP_COUNT_FOR_90_DEG },
            { "Z", ARDUINO_REFERENCE_STEP_COUNT_FOR_90_DEG },
            { "E0", ARDUINO_REFERENCE_STEP_COUNT_FOR_90_DEG }
        };

            SignalController.Instance.DetectionsReceived += OnDetectionsReceived;
            SignalController.Instance.StartupHomingRequestedAsync += HandleStartupHomingRequestedAsync;
            SignalController.Instance.RemoteSystemReadyForActions += OnRemoteSystemReadyForActions;
            SignalController.Instance.TofSensorsReceived += OnTofSensorsReceived;


            Console.WriteLine("[RobotController] Constructor: Initialized. Subscribed to DetectionsReceived, StartupHomingRequestedAsync, and RemoteSystemReadyForActions.");

            if (!_objectMemories.ContainsKey("person"))
                _objectMemories["person"] = new TrackedObjectMemory("person");
            if (!_objectMemories.ContainsKey(_priorityObjectName))
                _objectMemories[_priorityObjectName] = new TrackedObjectMemory(_priorityObjectName);

            Console.WriteLine("[RobotController] Constructor: Raising RobotControllerReadyForHomingCheck event.");
            SignalController.Instance.RaiseRobotControllerReadyForHomingCheck();
        }

        private bool IsTargetEffectivelyCentered(DetectionResult? target)
        {
            if (target?.Box == null || target.Box.Count != 4) return false;
            if (target.Box[2] < target.Box[0] || target.Box[3] < target.Box[1]) return false;

            double boxCenterX = target.Box[0] + (target.Box[2] - target.Box[0]) / 2.0;
            double boxCenterY = target.Box[1] + (target.Box[3] - target.Box[1]) / 2.0;
            double imageCenterX = FrameWidth / 2.0;
            double imageCenterY = FrameHeight / 2.0;
            double horizontalDifference = boxCenterX - imageCenterX;
            double verticalDifference = boxCenterY - imageCenterY;

            return Math.Abs(horizontalDifference) <= pixelDeadZone &&
                   Math.Abs(verticalDifference) <= pixelDeadZone;
        }

        private void OnDetectionsReceived(List<DetectionResult>? detections)
        {
            if (_isDisposed || !_isArmHomedAndSystemReady || _isHoming || _isPerformingGreeting)
            {
                if (!_isArmHomedAndSystemReady && !_isHoming) // Don't log if it's just because homing is in progress
                {
                    Console.WriteLine("[RobotController DEBUG] OnDetectionsReceived: Skipped - Arm not successfully homed yet or system not fully ready.");
                }
                return;
            }

            // Update object memories with current detections
            if (detections != null)
            {
                foreach (var det in detections)
                {
                    if (det.Confidence >= ConfidenceThreshold)
                    {
                        if (!_objectMemories.TryGetValue(det.Label, out var memory))
                        {
                            // Create memory for person or priority object if not existing
                            if (det.Label == "person" || det.Label == _priorityObjectName)
                            {
                                memory = new TrackedObjectMemory(det.Label);
                                _objectMemories[det.Label] = memory;
                            }
                        }
                        memory?.UpdateSighting(det); // Updates LastSeenTimestamp, confidence, etc.
                    }
                }
            }

            DetectionResult? currentPerson = detections?.FirstOrDefault(d => d.Label == "person" && d.Confidence >= ConfidenceThreshold);
            DetectionResult? currentObject = detections?.FirstOrDefault(d => d.Label == _priorityObjectName && d.Confidence >= ConfidenceThreshold);

            // Determine if person should be prioritized (ignore timer not active or expired)
            bool shouldPrioritizePerson = (_timeToIgnorePersonUntil == DateTime.MinValue || DateTime.UtcNow >= _timeToIgnorePersonUntil);

            Console.WriteLine($"[RobotController DEBUG] OnDetectionsReceived. State: {_currentState}. Ignoring Person Until: {(shouldPrioritizePerson ? "NOT ACTIVE" : _timeToIgnorePersonUntil.ToString("HH:mm:ss"))}");
            Console.WriteLine($"[RobotController DEBUG] Current Person: {(currentPerson == null ? "NULL" : $"{currentPerson.Label} (Conf: {currentPerson.Confidence:P1})")}");
            Console.WriteLine($"[RobotController DEBUG] Current Object ('{_priorityObjectName}'): {(currentObject == null ? "NULL" : $"{currentObject.Label} (Conf: {currentObject.Confidence:P1})")}");

            bool isPersonCurrentlyCentered = currentPerson != null && IsTargetEffectivelyCentered(currentPerson);
            bool isObjectCurrentlyCentered = currentObject != null && IsTargetEffectivelyCentered(currentObject);

            switch (_currentState)
            {
                case ControllerState.SearchingForPerson:
                    if (!shouldPrioritizePerson)
                    {
                        Console.WriteLine($"[RobotController SM] In SearchingForPerson, but must ignore person until {_timeToIgnorePersonUntil:HH:mm:ss}. Redirecting to DedicatedSearchingForObject.");
                        _currentState = ControllerState.DedicatedSearchingForObject;
                        _searchObjectAttemptCounter = 0;
                        _isMoving = false;
                        break;
                    }
                    _timeToIgnorePersonUntil = DateTime.MinValue; // Person is priority, so ensure timer is cleared

                    if (currentPerson != null)
                    {
                        Console.WriteLine($"[RobotController SM] Person detected. Transition to CenteringPerson.");
                        _currentState = ControllerState.CenteringPerson;
                        MoveTowardsTarget(currentPerson);
                    }
                    else // Person not currently seen
                    {
                        if (_objectMemories.TryGetValue("person", out var personMemory) &&
                            personMemory.WasEverCentered &&
                            (DateTime.UtcNow - personMemory.LastSeenTimestamp) <= _briefTrackingLossThreshold &&
                            (_currentPanAngle != personMemory.PanAngleAtLastCenter || _currentTiltAngle != personMemory.TiltAngleAtLastCenter)) // Avoid moving if already there
                        {
                            Console.WriteLine($"[RobotController SM] Person briefly lost. Moving to last known centered position: P={personMemory.PanAngleAtLastCenter}, T={personMemory.TiltAngleAtLastCenter}");
                            if (DateTime.UtcNow - _lastRobotCommandBatchTime >= _robotCommandCycleInterval)
                            {
                                if (_currentPanAngle != personMemory.PanAngleAtLastCenter)
                                    _ = SendServoCommand(PanServoID, personMemory.PanAngleAtLastCenter);
                                _currentPanAngle = personMemory.PanAngleAtLastCenter;
                                if (_currentTiltAngle != personMemory.TiltAngleAtLastCenter)
                                    _ = SendServoCommand(TiltServoID, personMemory.TiltAngleAtLastCenter);
                                _currentTiltAngle = personMemory.TiltAngleAtLastCenter;
                                _lastRobotCommandBatchTime = DateTime.UtcNow;
                                _isMoving = true;
                            }
                            else
                            {
                                Console.WriteLine($"[RobotController SM] Waiting for command cycle to move to last known person pos.");
                                _isMoving = false; // Will retry next cycle
                            }
                        }
                        else if (currentObject != null)
                        {
                            Console.WriteLine($"[RobotController SM] No person, but '{_priorityObjectName}' detected. Transition to DedicatedCenteringObject.");
                            _currentState = ControllerState.DedicatedCenteringObject;
                            // This implies a switch of priority, start ignore timer for person
                            _timeToIgnorePersonUntil = DateTime.UtcNow + _ignorePersonDuration;
                            MoveTowardsTarget(currentObject);
                        }
                        else
                        {
                            Console.WriteLine($"[RobotController SM] SearchingForPerson: Target not found.");
                            _isMoving = false;
                        }
                    }
                    break;

                case ControllerState.CenteringPerson:
                    _timeToIgnorePersonUntil = DateTime.MinValue;
                    if (currentPerson != null)
                    {
                        MoveTowardsTarget(currentPerson);
                        if (IsTargetEffectivelyCentered(currentPerson)) // Re-check after move
                        {
                            _personCenteredTimestamp = DateTime.UtcNow;
                            _currentMonitoringDuration = TimeSpan.FromSeconds(_rng.Next(5, 10));
                            if (_objectMemories.TryGetValue("person", out var personMemory))
                            {
                                personMemory.UpdateCenteredLocation(_currentPanAngle, _currentTiltAngle);
                            }
                            Console.WriteLine($"[RobotController SM] Person centered. Transition to MonitoringCenteredPerson for {_currentMonitoringDuration.TotalSeconds}s.");
                            _currentState = ControllerState.MonitoringCenteredPerson;
                            _isMoving = false; // Centered, stop moving for now
                            _ = Task.Run(() => PerformGreetingSequenceAsync()); // Run greeting asynchronously

                        }
                        else
                        {
                            _isMoving = true; // Still needs to move
                        }
                    }
                    else
                    {
                        Console.WriteLine("[RobotController SM] Person lost while CenteringPerson. Transition to SearchingForPerson.");
                        _currentState = ControllerState.SearchingForPerson;
                        _isMoving = false;
                    }
                    break;

                case ControllerState.PerformingGreeting:
                    Console.WriteLine($"[RobotController SM] Currently PerformingGreeting. Person: {(currentPerson == null ? "LOST" : "Present")}");
                    // While greeting, we might want to:
                    // 1. Keep the camera pan/tilt servos trying to stay on the person if possible (gentle adjustments).
                    // 2. Or, freeze pan/tilt servos and let the arm do its thing.
                    // 3. Or, if person is lost, maybe abort greeting.

                    // For now, let's do minimal pan/tilt adjustment if person is still there and not centered.
                    // But only if _isPerformingGreeting is true, to avoid race with its completion.
                    if (_isPerformingGreeting)
                    {
                        if (currentPerson != null && !IsTargetEffectivelyCentered(currentPerson))
                        {
                            // Optionally, make very gentle adjustments to keep person in view
                            // MoveTowardsTarget(currentPerson); 
                            // Or just hold position:
                            // _isMoving = false; 
                        }
                        else if (currentPerson == null)
                        {
                            Console.WriteLine("[RobotController SM] Person lost during greeting. Greeting will continue, then will search.");
                            // Greeting continues, will transition to search/monitor after via PerformGreetingSequenceAsync's finally block
                        }
                    }
                    // No further state changes here; PerformGreetingSequenceAsync's finally block handles it.
                    _isMoving = false; // Generally, don't make large pan/tilt moves during greeting
                    break;

                case ControllerState.MonitoringCenteredPerson:
                    _timeToIgnorePersonUntil = DateTime.MinValue;
                    if (currentPerson != null)
                    {
                        // Optional: Tiny adjustments if needed, or just check if still centered
                        bool stillCentered = IsTargetEffectivelyCentered(currentPerson);
                        if (!stillCentered && !_isPerformingGreeting) // Only adjust if not greeting
                        {
                            Console.WriteLine("[RobotController SM] Person moved from center during Monitoring. Transition back to CenteringPerson.");
                            _currentState = ControllerState.CenteringPerson;
                            MoveTowardsTarget(currentPerson); // Start moving
                        }
                        else if (DateTime.UtcNow - _personCenteredTimestamp >= _currentMonitoringDuration && !_isPerformingGreeting)
                        {
                            Console.WriteLine($"[RobotController SM] Person monitored for {_currentMonitoringDuration.TotalSeconds}s. Activating IgnorePerson timer. Transition to DedicatedSearchingForObject for '{_priorityObjectName}'.");
                            _timeToIgnorePersonUntil = DateTime.UtcNow + _ignorePersonDuration;
                            _currentState = ControllerState.DedicatedSearchingForObject;
                            _searchObjectAttemptCounter = 0;
                            _isMoving = false;
                        }
                        else if (stillCentered)
                        {
                            // Console.WriteLine($"[RobotController SM] MonitoringCenteredPerson: Person still centered. Time: {(DateTime.UtcNow - _personCenteredTimestamp).TotalSeconds:F1}s / {_currentMonitoringDuration.TotalSeconds:F1}s");
                            _isMoving = false;
                        }
                    }
                    else
                    {
                        Console.WriteLine("[RobotController SM] Person lost during Monitoring. Transition to SearchingForPerson.");
                        _currentState = ControllerState.SearchingForPerson;
                        _isMoving = false;
                    }
                    break;

                case ControllerState.DedicatedSearchingForObject:
                    Console.WriteLine($"[RobotController SM] In DedicatedSearchingForObject for '{_priorityObjectName}'. Person ignored until {(shouldPrioritizePerson ? "EXPIRED/INACTIVE" : _timeToIgnorePersonUntil.ToString("HH:mm:ss"))}.");
                    _isMoving = false;

                    if (shouldPrioritizePerson && currentPerson != null && !_isPerformingGreeting) // Added person check & greeting flag
                    {
                        Console.WriteLine($"[RobotController SM] Ignore period for person has ELAPSED or was inactive, AND PERSON DETECTED. Transition to CenteringPerson.");
                        _timeToIgnorePersonUntil = DateTime.MinValue;
                        _currentState = ControllerState.CenteringPerson;
                        MoveTowardsTarget(currentPerson);
                        break;
                    }

                    if (shouldPrioritizePerson)
                    {
                        Console.WriteLine($"[RobotController SM] Ignore period for person has ELAPSED or was inactive. Transition to SearchingForPerson.");
                        _timeToIgnorePersonUntil = DateTime.MinValue;
                        _currentState = ControllerState.SearchingForPerson;
                        break;
                    }

                    if (currentObject != null)
                    {
                        Console.WriteLine($"[RobotController SM] Object '{_priorityObjectName}' detected. Transition to DedicatedCenteringObject.");
                        _currentState = ControllerState.DedicatedCenteringObject;
                        MoveTowardsTarget(currentObject);
                    }
                    else // Object not currently seen
                    {
                        if (_objectMemories.TryGetValue(_priorityObjectName, out var objectMemory) &&
                           objectMemory.WasEverCentered &&
                           (DateTime.UtcNow - objectMemory.LastSeenTimestamp) <= _briefTrackingLossThreshold &&
                           (_currentPanAngle != objectMemory.PanAngleAtLastCenter || _currentTiltAngle != objectMemory.TiltAngleAtLastCenter))
                        {
                            Console.WriteLine($"[RobotController SM] Object '{_priorityObjectName}' briefly lost. Moving to last known centered position: P={objectMemory.PanAngleAtLastCenter}, T={objectMemory.TiltAngleAtLastCenter}");
                            if (DateTime.UtcNow - _lastRobotCommandBatchTime >= _robotCommandCycleInterval)
                            {
                                if (_currentPanAngle != objectMemory.PanAngleAtLastCenter)
                                    _ = SendServoCommand(PanServoID, objectMemory.PanAngleAtLastCenter);
                                _currentPanAngle = objectMemory.PanAngleAtLastCenter;
                                if (_currentTiltAngle != objectMemory.TiltAngleAtLastCenter)
                                    _ = SendServoCommand(TiltServoID, objectMemory.TiltAngleAtLastCenter);
                                _currentTiltAngle = objectMemory.TiltAngleAtLastCenter;
                                _lastRobotCommandBatchTime = DateTime.UtcNow;
                                _isMoving = true;
                            }
                            else
                            {
                                Console.WriteLine($"[RobotController SM] Waiting for command cycle to move to last known object pos.");
                                _isMoving = false;
                            }
                        }
                        else
                        {
                            _searchObjectAttemptCounter++;
                            Console.WriteLine($"[RobotController SM] DedicatedSearchingForObject: No '{_priorityObjectName}' found (Attempt: {_searchObjectAttemptCounter}/{MaxSearchObjectAttempts}).");
                            if (_searchObjectAttemptCounter >= MaxSearchObjectAttempts)
                            {
                                Console.WriteLine($"[RobotController SM] Max attempts for finding '{_priorityObjectName}'. Transition to SearchingForPerson.");
                                _timeToIgnorePersonUntil = DateTime.MinValue;
                                _currentState = ControllerState.SearchingForPerson;
                            }
                            // else, could implement active search pattern here
                        }
                    }
                    break;

                case ControllerState.DedicatedCenteringObject:
                    Console.WriteLine($"[RobotController SM] In DedicatedCenteringObject for '{_priorityObjectName}'. Person ignored until {(shouldPrioritizePerson ? "EXPIRED/INACTIVE" : _timeToIgnorePersonUntil.ToString("HH:mm:ss"))}.");

                    if (shouldPrioritizePerson) // Check if ignore period has naturally expired
                    {
                        Console.WriteLine($"[RobotController SM] Ignore period for person ELAPSED while centering object. Transition to SearchingForPerson.");
                        _timeToIgnorePersonUntil = DateTime.MinValue;
                        _currentState = ControllerState.SearchingForPerson;
                        _isMoving = false;
                        break;
                    }

                    if (currentObject != null)
                    {
                        MoveTowardsTarget(currentObject);
                        if (IsTargetEffectivelyCentered(currentObject)) // Re-check after move
                        {
                            if (_objectMemories.TryGetValue(_priorityObjectName, out var objMemory))
                            {
                                objMemory.UpdateCenteredLocation(_currentPanAngle, _currentTiltAngle);
                            }
                            Console.WriteLine($"[RobotController SM] Object '{_priorityObjectName}' centered. Transition to ObjectIsCentered (will respect ignore timer).");
                            _currentState = ControllerState.ObjectIsCentered;
                            _isMoving = false; // Centered
                        }
                        else
                        {
                            _isMoving = true; // Still needs to move
                        }
                    }
                    else
                    {
                        Console.WriteLine($"[RobotController SM] Object '{_priorityObjectName}' lost during DedicatedCenteringObject. Transition to DedicatedSearchingForObject.");
                        _currentState = ControllerState.DedicatedSearchingForObject;
                        _isMoving = false;
                    }
                    break;

                case ControllerState.ObjectIsCentered:
                    Console.WriteLine($"[RobotController SM] Object '{_priorityObjectName}' is centered. Checking ignore timer. Person ignored until {(shouldPrioritizePerson ? "EXPIRED/INACTIVE" : _timeToIgnorePersonUntil.ToString("HH:mm:ss"))}.");
                    if (!shouldPrioritizePerson) // Still ignoring person as timer is active
                    {
                        Console.WriteLine($"[RobotController SM] Still ignoring person. Transitioning back to DedicatedSearchingForObject to maintain object focus/re-track if it moves.");
                        _currentState = ControllerState.DedicatedSearchingForObject;
                        _searchObjectAttemptCounter = 0; // Reset search attempts as we were just on target
                    }
                    else
                    {
                        Console.WriteLine($"[RobotController SM] Ignore period for person has elapsed or was not active. Transitioning to SearchingForPerson.");
                        _timeToIgnorePersonUntil = DateTime.MinValue;
                        _currentState = ControllerState.SearchingForPerson;
                    }
                    _isMoving = false;
                    break;
            }
        }

        private void OnTofSensorsReceived(int? tofHand, int? tofCam)
        {
            _latestTofHandValue = tofHand;
            _latestTofCamValue = tofCam; // Store if needed
                                         // You can add a log here if you want to see the TOF values as RobotController receives them
                                         // Console.WriteLine($"[RobotController] TOF Data Updated - Hand: {_latestTofHandValue?.ToString() ?? "N/A"}, Cam: {_latestTofCamValue?.ToString() ?? "N/A"}");
        }

        private async Task OpenGripper()
        {
            Console.WriteLine($"[RobotController] Sending command: Open Gripper (Servo {GRIPPER_SERVO_ID} to {GRIPPER_OPEN_ANGLE})");
            await SendServoCommand(GRIPPER_SERVO_ID, GRIPPER_OPEN_ANGLE);
            await Task.Delay(700); // Allow time for servo to move
        }

        private async Task CloseGripper()
        {
            Console.WriteLine($"[RobotController] Sending command: Close Gripper (Servo {GRIPPER_SERVO_ID} to {GRIPPER_CLOSED_ANGLE})");
            await SendServoCommand(GRIPPER_SERVO_ID, GRIPPER_CLOSED_ANGLE);
            await Task.Delay(700); // Allow time for servo to move
        }

        private Task HandleStartupHomingRequestedAsync(Tool2ViewModel tool2ViewModel)
        {
            Console.WriteLine("[RobotController] HandleStartupHomingRequestedAsync: Received request from Tool2ViewModel.");
            if (!_startupHomingAttemptCompleted)
            {
                if (tool2ViewModel != null)
                {
                    _viewModelForStartupHoming = tool2ViewModel;
                    _startupHomingRequestedByViewModel = true;
                    Console.WriteLine("[RobotController] HandleStartupHomingRequestedAsync: Startup homing is PENDING, waiting for remote system readiness.");
                }
                else
                {
                    Console.WriteLine("[RobotController] HandleStartupHomingRequestedAsync: Tool2ViewModel instance was null. Cannot pend homing.");
                }
            }
            else
            {
                Console.WriteLine("[RobotController] HandleStartupHomingRequestedAsync: Startup homing was already attempted/completed. Ignoring.");
            }
            return Task.CompletedTask;
        }

        private async void OnRemoteSystemReadyForActions()
        {
            Console.WriteLine("[RobotController] OnRemoteSystemReadyForActions: Received signal that remote system is ready.");
            if (_startupHomingRequestedByViewModel && _viewModelForStartupHoming != null && !_startupHomingAttemptCompleted)
            {
                Console.WriteLine("[RobotController] OnRemoteSystemReadyForActions: Executing PENDING startup homing sequence.");
                _startupHomingAttemptCompleted = true;

                bool homingSuccess = await AutoHomeArmTo90Async(_viewModelForStartupHoming); // Wait for homing to complete

                if (homingSuccess)
                {
                    Console.WriteLine("[RobotController] OnRemoteSystemReadyForActions: Arm homing SUCCESSFUL. System is now ready for detection logic.");
                    _isArmHomedAndSystemReady = true;
                }
                else
                {
                    Console.WriteLine("[RobotController] OnRemoteSystemReadyForActions: Arm homing FAILED or was aborted. Detection logic will NOT start.");
                    _isArmHomedAndSystemReady = false;
                }
            }
            else
            {
                if (_startupHomingAttemptCompleted) Console.WriteLine("[RobotController] OnRemoteSystemReadyForActions: Startup homing already attempted.");
                else if (!_startupHomingRequestedByViewModel) Console.WriteLine("[RobotController] OnRemoteSystemReadyForActions: Startup homing was not requested by ViewModel.");
                else if (_viewModelForStartupHoming == null) Console.WriteLine("[RobotController] OnRemoteSystemReadyForActions: ViewModel for homing is null.");
                else Console.WriteLine("[RobotController] OnRemoteSystemReadyForActions: Conditions for startup homing not met (this should not happen if handshake is correct).");
            }
        }

        private async Task PerformGreetingSequenceAsync()
        {
            if (_isPerformingGreeting)
            {
                Console.WriteLine("[RobotController] Greeting already in progress.");
                return;
            }

            _isPerformingGreeting = true;
            Console.WriteLine("[RobotController] Starting greeting sequence...");

            // Store current pan/tilt to try and keep camera focused on person during greeting
            int panAtGreetingStart = _currentPanAngle;
            int tiltAtGreetingStart = _currentTiltAngle;

            try
            {
                // Example: A simple "nod" or "wave" using the E0 motor (upper elbow/wrist)
                // Positive E0 = backward, Negative E0 = forward
                // All step values and delays are EXAMPLES and NEED TUNING for your robot!

                const int greetingStepAmount = 150; // Example step amount for E0
                const int greetingSegmentDelayMs = 700; // Example delay for movement segment
                const int interMovementDelayMs = 200; // Short pause between distinct parts

                // 1. Move E0 slightly "backward" (e.g., lift wrist up)
                Console.WriteLine("[RobotController Greeting] Move E0 backward (up)");
                _arduinoController.EnqueueArduinoCommand($"moveE0 {greetingStepAmount}");
                await Task.Delay(greetingSegmentDelayMs);

                await Task.Delay(interMovementDelayMs);

                // 2. Move E0 "forward" from that new position (e.g., bring wrist down)
                Console.WriteLine("[RobotController Greeting] Move E0 forward (down)");
                _arduinoController.EnqueueArduinoCommand($"moveE0 -{greetingStepAmount * 2}"); // Move more to go past original
                await Task.Delay(greetingSegmentDelayMs);

                await Task.Delay(interMovementDelayMs);

                // 3. Move E0 back to roughly the initial greeting up position
                Console.WriteLine("[RobotController Greeting] Move E0 backward (up) to initial greeting spot");
                _arduinoController.EnqueueArduinoCommand($"moveE0 {greetingStepAmount * 2}");
                await Task.Delay(greetingSegmentDelayMs);

                await Task.Delay(interMovementDelayMs);

                // 4. Return E0 to its starting point before the greeting (neutralize the greeting motion)
                Console.WriteLine("[RobotController Greeting] Return E0 to neutral from greeting");
                _arduinoController.EnqueueArduinoCommand($"moveE0 -{greetingStepAmount}");
                await Task.Delay(greetingSegmentDelayMs);

                Console.WriteLine("[RobotController] Greeting sequence finished.");
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[RobotController] Error during greeting sequence: {ex.Message}");
            }
            finally
            {
                _isPerformingGreeting = false;
                Console.WriteLine("[RobotController] _isPerformingGreeting set to false.");

                // After greeting, decide what to do.
                // For now, let's transition to MonitoringCenteredPerson if the system was in greeting state.
                // This logic might need refinement based on whether the person is still there.
                if (_currentState == ControllerState.PerformingGreeting) // Check if we weren't interrupted by another state change
                {
                    Console.WriteLine("[RobotController SM] Greeting done. Transitioning to MonitoringCenteredPerson.");
                    _currentState = ControllerState.MonitoringCenteredPerson;
                    _personCenteredTimestamp = DateTime.UtcNow; // Reset monitoring timer
                                                                // Optionally, re-center camera on person if they moved slightly during greeting
                                                                // This would require getting latest detections again.
                }
            }
        }

        public async Task<bool> AutoHomeArmTo90Async(Tool2ViewModel tool2ViewModel) // Changed to return bool
        {
            if (_isHoming || _isPerformingGreeting)
            {
                Console.WriteLine("[RobotController Homing] AutoHomeArmTo90Async: Homing or Greeting already in progress.");
                return false; // Indicate failure/not run
            }
            _isHoming = true;
            Console.WriteLine("[RobotController Homing] AutoHomeArmTo90Async: Starting EXACT iterative arm homing with stability check...");

            bool allMotorsHomedSuccessfully = true; // Assume success until a motor fails

            int[]? liveRawEncoders = null;
            const int encoderWaitTimeoutMs = 15000;
            const int encoderPollIntervalMs = 250;
            Stopwatch sw = Stopwatch.StartNew();

            Console.WriteLine($"[RobotController Homing] AutoHomeArmTo90Async: Waiting up to {encoderWaitTimeoutMs / 1000}s for initial live encoder data...");
            while (sw.ElapsedMilliseconds < encoderWaitTimeoutMs)
            {
                liveRawEncoders = tool2ViewModel.GetLastRawEncoderValues();
                if (liveRawEncoders != null && liveRawEncoders.Length == 4)
                {
                    Console.WriteLine($"[RobotController Homing] AutoHomeArmTo90Async: Initial live encoder data received: {string.Join(",", liveRawEncoders)}");
                    break;
                }
                await Task.Delay(encoderPollIntervalMs);
            }
            sw.Stop();

            if (liveRawEncoders == null || liveRawEncoders.Length != 4)
            {
                Console.WriteLine($"[RobotController Homing] AutoHomeArmTo90Async: TIMED OUT waiting for live encoder data. Aborting homing.");
                _isHoming = false;
                return false; // Indicate failure
            }

            Dictionary<int, EncoderState> encoderStates = tool2ViewModel.GetEncoderStatesForHoming();
            if (encoderStates == null || !encoderStates.Any())
            {
                Console.WriteLine("[RobotController Homing] AutoHomeArmTo90Async: Could not get encoder states. Aborting homing.");
                _isHoming = false;
                return false; // Indicate failure
            }

            // --- CRITICAL: VERIFY THESE BOOLEAN VALUES FOR EACH MOTOR ---
            var motorDetails = new[] {
        new { Name = "X",  EncoderIndex = 0, PositiveAccelStepIncreasesEncoder = true }, // True: higher AccelStep pos -> higher encoder raw
        new { Name = "Y",  EncoderIndex = 1, PositiveAccelStepIncreasesEncoder = true }, // Verify this!
        new { Name = "Z",  EncoderIndex = 2, PositiveAccelStepIncreasesEncoder = false },// Based on your logs, this should be false
        new { Name = "E0", EncoderIndex = 3, PositiveAccelStepIncreasesEncoder = false } // E0 often behaves inverted relative to default assumptions
    };

            const int maxHomingAttemptsPerMotor = 60; // Increased attempts for stability checks
            const int moveDelayMsBase = 250;
            const int NUDGE_ACCELSTEPPER_STEPS = 1;   // Use 1 for precise nudging during stability
            const int stabilityMonitorDurationMs = 2000; // Monitor for 2 seconds
            const int stabilityPollIntervalMs = 200;    // Check encoder every 200ms during monitoring
            int readsRequiredForStable = (stabilityMonitorDurationMs / stabilityPollIntervalMs) - 1; // e.g. 9 reads for 10 intervals

            try
            {
                for (int i = 0; i < motorDetails.Length; i++)
                {
                    var motor = motorDetails[i];
                    bool thisMotorSuccessfullyHomedAndStable = false;

                    if (!encoderStates.TryGetValue(motor.EncoderIndex, out EncoderState? state) || !state.IsCalibrated)
                    {
                        Console.WriteLine($"[RobotController Homing] Motor {motor.Name}: Not calibrated. Skipping.");
                        allMotorsHomedSuccessfully = false; // If one motor isn't calibrated, overall homing isn't perfect
                        continue;
                    }

                    int targetRawValue = state.CalibratedRawAt90Degrees;
                    Console.WriteLine($"[RobotController Homing] Motor {motor.Name}: TargetRaw={targetRawValue}. Initial AccelStepper LogicalPos={_currentLogicalAccelStepperPositions[motor.Name]}");

                    for (int attempt = 0; attempt < maxHomingAttemptsPerMotor; attempt++)
                    {
                        int[]? currentRawEncodersAttempt = tool2ViewModel.GetLastRawEncoderValues();
                        if (currentRawEncodersAttempt == null || currentRawEncodersAttempt.Length != 4)
                        {
                            Console.WriteLine($"[RobotController Homing] Motor {motor.Name} Att {attempt + 1}: No live encoder data. Waiting...");
                            await Task.Delay(encoderPollIntervalMs);
                            continue;
                        }
                        int currentRawValue = currentRawEncodersAttempt[motor.EncoderIndex];
                        int rawDifference = targetRawValue - currentRawValue;

                        Console.WriteLine($"[RobotController Homing] Motor {motor.Name} Att {attempt + 1}/{maxHomingAttemptsPerMotor}: CurrentRaw={currentRawValue}, Target={targetRawValue}, RawDiff={rawDifference}");

                        if (rawDifference == 0)
                        {
                            Console.WriteLine($"[RobotController Homing] Motor {motor.Name}: Raw value matches target ({targetRawValue}). Monitoring for stability ({stabilityMonitorDurationMs / 1000}s)...");
                            bool stableAtTarget = true;
                            int stableReadCount = 0;
                            Stopwatch stabilitySw = Stopwatch.StartNew();

                            while (stabilitySw.ElapsedMilliseconds < stabilityMonitorDurationMs)
                            {
                                await Task.Delay(stabilityPollIntervalMs);
                                int[]? postMoveEncoders = tool2ViewModel.GetLastRawEncoderValues();
                                if (postMoveEncoders == null || postMoveEncoders.Length != 4)
                                {
                                    Console.WriteLine($"[RobotController Homing Stability] Motor {motor.Name}: Lost encoder values during stability check.");
                                    stableAtTarget = false; break;
                                }
                                int postMoveRawValue = postMoveEncoders[motor.EncoderIndex];
                                Console.WriteLine($"[RobotController Homing Stability] Motor {motor.Name}: Check ({(int)(stabilitySw.ElapsedMilliseconds / stabilityPollIntervalMs)}), Raw: {postMoveRawValue} (Target: {targetRawValue})");
                                if (postMoveRawValue != targetRawValue)
                                {
                                    Console.WriteLine($"[RobotController Homing Stability] Motor {motor.Name}: Value drifted to {postMoveRawValue} from {targetRawValue}. Unstable.");
                                    stableAtTarget = false; break;
                                }
                                stableReadCount++;
                            }
                            stabilitySw.Stop();

                            if (stableAtTarget && stableReadCount >= Math.Max(1, readsRequiredForStable * 0.8))
                            { // Ensure at least one good read if duration is short
                                Console.WriteLine($"[RobotController Homing] Motor {motor.Name} is STABLE and EXACTLY at target ({targetRawValue}) after {stableReadCount} stable checks.");
                                thisMotorSuccessfullyHomedAndStable = true;
                                break; // Exit the attempt loop for THIS MOTOR
                            }
                            else
                            {
                                Console.WriteLine($"[RobotController Homing Stability] Motor {motor.Name}: Value NOT stable (Stable reads: {stableReadCount}/{readsRequiredForStable}). Resuming homing attempts.");
                                // Let the loop continue for more nudges
                            }
                        } // End of if (rawDifference == 0) for stability check trigger

                        if (thisMotorSuccessfullyHomedAndStable) break; // Already homed and stable

                        // --- Nudging Logic ---
                        int encoderChangeSign = Math.Sign(rawDifference);
                        int stepperNudgeDirection;

                        if (motor.PositiveAccelStepIncreasesEncoder)
                        {
                            stepperNudgeDirection = encoderChangeSign;
                        }
                        else
                        {
                            stepperNudgeDirection = -encoderChangeSign;
                        }

                        int currentLogicalPosition = _currentLogicalAccelStepperPositions[motor.Name];
                        int newLogicalPosition = currentLogicalPosition + (stepperNudgeDirection * NUDGE_ACCELSTEPPER_STEPS);

                        Console.WriteLine($"[RobotController Homing] Motor {motor.Name}: Nudging from logical {currentLogicalPosition} to {newLogicalPosition} (TargetRawDiff: {rawDifference})");
                        _arduinoController.EnqueueArduinoCommand($"move{motor.Name}{newLogicalPosition}");
                        _currentLogicalAccelStepperPositions[motor.Name] = newLogicalPosition;

                        await Task.Delay(moveDelayMsBase);

                        if (attempt == maxHomingAttemptsPerMotor - 1 && !thisMotorSuccessfullyHomedAndStable)
                        {
                            Console.WriteLine($"[RobotController Homing] Motor {motor.Name} FAILED to reach and stabilize at exact target after {maxHomingAttemptsPerMotor} attempts. FinalRaw={currentRawValue}, Target={targetRawValue}");
                            allMotorsHomedSuccessfully = false; // Mark overall failure if any motor fails
                        }
                    } // End attempts loop for one motor

                    if (!thisMotorSuccessfullyHomedAndStable)
                    {
                        allMotorsHomedSuccessfully = false; // If this motor didn't succeed, overall homing failed.
                    }

                    Console.WriteLine($"[RobotController Homing] Motor {motor.Name} homing attempt loop finished. Setting Arduino current position to {ARDUINO_REFERENCE_STEP_COUNT_FOR_90_DEG}. Last Logical Pos was: {_currentLogicalAccelStepperPositions[motor.Name]}");
                    _arduinoController.EnqueueArduinoCommand($"setcurrentposition{motor.Name}{ARDUINO_REFERENCE_STEP_COUNT_FOR_90_DEG}");
                    _currentLogicalAccelStepperPositions[motor.Name] = ARDUINO_REFERENCE_STEP_COUNT_FOR_90_DEG;
                    await Task.Delay(100);
                } // End loop for all motors
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[RobotController Homing] CRITICAL Error during motor homing iterations: {ex.Message} {ex.StackTrace}");
                allMotorsHomedSuccessfully = false;
            }
            finally
            {
                _isHoming = false;
                Console.WriteLine($"[RobotController] Arm homing sequence attempt completed. Overall success: {allMotorsHomedSuccessfully}");
            }
            return allMotorsHomedSuccessfully;
        }

        private void MoveTowardsTarget(DetectionResult target)
        {
            if (target.Box == null || target.Box.Count != 4) return;
            if (target.Box[2] < target.Box[0] || target.Box[3] < target.Box[1])
            {
                Console.WriteLine($"[RobotController WARN] Invalid box in MoveTowardsTarget for {target.Label}: [{string.Join(",", target.Box)}]");
                return;
            }

            double boxCenterX = target.Box[0] + (target.Box[2] - target.Box[0]) / 2.0;
            double boxCenterY = target.Box[1] + (target.Box[3] - target.Box[1]) / 2.0;
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
                        Console.WriteLine($"[RobotController] Adjusting Pan from {_currentPanAngle} to {calculatedTargetPanAngle} (HDiff: {horizontalDifference:F2}) for {target.Label}");
                        _ = SendServoCommand(PanServoID, calculatedTargetPanAngle);
                        _currentPanAngle = calculatedTargetPanAngle;
                        commandActuallySentThisCycle = true;
                    }
                    if (tiltAdjustmentCalculated)
                    {
                        Console.WriteLine($"[RobotController] Adjusting Tilt from {_currentTiltAngle} to {calculatedTargetTiltAngle} (VDiff: {verticalDifference:F2}) for {target.Label}");
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

            if (isPhysicallyAlignedNow && !anyMovementCalculated) // Only log aligned if no further movement was calculated for this step
            {
                Console.WriteLine($"[RobotController] Target ({target.Label}) is ALIGNED (DEAD CENTER). (HDiff: {hDiff:F2}, VDiff: {vDiff:F2})");
            }
            else if (anyMovementCalculated && calcPan == _currentPanAngle && calcTilt == _currentTiltAngle && !commandSentThisCycle && !isWaitingForGlobalInterval)
            {
                // This case means movement was calculated but resulted in no change (e.g. already at physical servo limit MinAngle/MaxAngle)
                Console.WriteLine($"[RobotController] Target ({target.Label}) AT SERVO LIMIT or no change, cannot improve alignment further this step. (HDiff: {hDiff:F2}, VDiff: {vDiff:F2})");
            }
            else if (isWaitingForGlobalInterval)
            {
                Console.WriteLine($"[RobotController] Target ({target.Label}) requires adjustment (Desired P:{calcPan}, T:{calcTilt}), waiting for GLOBAL command interval.");
            }
            // Logging for active adjustment is done in MoveTowardsTarget before SendServoCommand
        }

        private async Task SendServoCommand(int servoId, int angle)
        {
            if (_isDisposed) return;
            string command = $"servo{servoId}{angle}";
            _arduinoController.EnqueueArduinoCommand(command);
            await Task.CompletedTask;
        }

        public void Dispose()
        {
            if (!_isDisposed)
            {
                Console.WriteLine("[RobotController] Disposing...");
                _isDisposed = true;
                if (SignalController.Instance != null)
                {
                    SignalController.Instance.DetectionsReceived -= OnDetectionsReceived;
                    SignalController.Instance.StartupHomingRequestedAsync -= HandleStartupHomingRequestedAsync;
                    SignalController.Instance.RemoteSystemReadyForActions -= OnRemoteSystemReadyForActions;
                    SignalController.Instance.TofSensorsReceived -= OnTofSensorsReceived;

                }
                Console.WriteLine("[RobotController] Unsubscribed from DetectionsReceived.");
            }
        }
    }
}