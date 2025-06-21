using RobotAIArm.ViewModels.Tools;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Threading.Tasks;

// Assuming DetectionResult, SignalController, ArduinoController are accessible

namespace RobotAIArm.Controllers
{
    internal record ArmMotorConfig(string Name, int EncoderIndex, bool PositiveAccelStepIncreasesEncoder);

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
        private bool _startupHomingRequestedByViewModel = false;
        private Tool2ViewModel? _viewModelForStartupHoming = null;
        private bool _startupHomingAttemptCompleted = false;
        private readonly Dictionary<string, int> _currentLogicalAccelStepperPositions;
        private bool _isArmHomedAndSystemReady = false;
        private bool _isGrabSequenceActive = false; // Lock for grab sequence

        private const double KIN_L1_BASE_TO_SHOULDER_Y_MM = 140.51;
        private const double KIN_L2_SHOULDER_TO_ELBOW_MM = 118.0;
        private const double KIN_L3_ELBOW_TO_WRIST_MM = 140.51;
        private const double KIN_L4_WRIST_TO_HAND_BASE_MM = 66.2;
        private const double KIN_L5_HAND_BASE_TO_TIP_MM = 77.6;
        private const double KIN_L_EFFECTOR_MM = KIN_L4_WRIST_TO_HAND_BASE_MM + KIN_L5_HAND_BASE_TO_TIP_MM;

        private const string TARGET_GRAB_OBJECT_LABEL = "bottle";
        private const int TOF_HAND_GRIPPING_DISTANCE_MM = 45; // USER TUNE
        private const int TOF_HAND_TOO_CLOSE_MM = 20;      // USER TUNE
        private const int TOF_HAND_NO_OBJECT_OR_ERROR_MM = 0;
        // ***** CRITICAL CALIBRATION FOR PHYSICAL OFFSETS (USER MUST MEASURE!) *****
        // Distance in millimeters from the camera's pivot point to the arm's base pivot point.
        private const double CAMERA_TO_ARM_BASE_OFFSET_X_MM = 0.0; // Left/Right offset. + is right.
        private const double CAMERA_TO_ARM_BASE_OFFSET_Y_MM = -140.51; // Front/Back offset. + is in front of cam, - is behind.

        private const int SAFE_POSE_LOGICAL_Y = 1000; // EXAMPLE: May need to be negative
        private const int SAFE_POSE_LOGICAL_Z = 1000;  // EXAMPLE: May need to be positive
        // ***** UPDATED LOGICAL_STEPS_PER_DEGREE_E0 based on your input "25500 steps for 90 degrees" *****
        private const double LOGICAL_STEPS_PER_DEGREE_X = 25500.0 / 180.0; // USER MUST CALIBRATE!
        private const double LOGICAL_STEPS_PER_DEGREE_Y = 25500.0 / 90.0; // USER MUST CALIBRATE!
        private const double LOGICAL_STEPS_PER_DEGREE_Z = -25500.0 / 90.0; // USER MUST CALIBRATE!
        private const double LOGICAL_STEPS_PER_DEGREE_E0 = -25500.0 / 90.0; // Approx 283.333

        private const int KINEMATIC_Z_FORWARD_LOGICAL_DIRECTION = 1;  // USER MUST VERIFY! (e.g., 1 if +logical steps extend Z phys angle towards 180)
        private const int KINEMATIC_Y_COMPENSATION_LOGICAL_DIRECTION = -1; // USER MUST VERIFY! (e.g., 1 if +logical steps makes Y phys angle help reach)

        private const int ARDUINO_REFERENCE_STEP_COUNT_FOR_90_DEG = 0;

        // ***** CORRECTED MIN/MAX LOGICAL POSITIONS based on ArmMotors having Y,Z,E0 as TRUE for PositiveAccelStepIncreasesEncoder *****
        private static readonly int MIN_LOGICAL_POS_X = ARDUINO_REFERENCE_STEP_COUNT_FOR_90_DEG + (int)(90.0 * LOGICAL_STEPS_PER_DEGREE_X);
        private static readonly int MAX_LOGICAL_POS_X = ARDUINO_REFERENCE_STEP_COUNT_FOR_90_DEG - (int)(90.0 * LOGICAL_STEPS_PER_DEGREE_X);

        private static readonly int MIN_LOGICAL_POS_Y = ARDUINO_REFERENCE_STEP_COUNT_FOR_90_DEG - (int)(90.0 * LOGICAL_STEPS_PER_DEGREE_Y);
        private static readonly int MAX_LOGICAL_POS_Y = ARDUINO_REFERENCE_STEP_COUNT_FOR_90_DEG + (int)(90.0 * LOGICAL_STEPS_PER_DEGREE_Y);

        private static readonly int MIN_LOGICAL_POS_Z = ARDUINO_REFERENCE_STEP_COUNT_FOR_90_DEG - (int)(90.0 * LOGICAL_STEPS_PER_DEGREE_Z);
        private static readonly int MAX_LOGICAL_POS_Z = ARDUINO_REFERENCE_STEP_COUNT_FOR_90_DEG + (int)(90.0 * LOGICAL_STEPS_PER_DEGREE_Z);

        private static readonly int MIN_LOGICAL_POS_E0 = ARDUINO_REFERENCE_STEP_COUNT_FOR_90_DEG - (int)(90.0 * LOGICAL_STEPS_PER_DEGREE_E0); // Should be -25500
        private static readonly int MAX_LOGICAL_POS_E0 = ARDUINO_REFERENCE_STEP_COUNT_FOR_90_DEG + (int)(90.0 * LOGICAL_STEPS_PER_DEGREE_E0); // Should be +25500

        // ***** CRITICAL CALIBRATION FOR E0 TOF SENSING POSITION *****
        // You want E0 "90 degrees forward". With PositiveAccelStepIncreasesEncoder=true for E0, and 0 logical = 90 physical:
        // To make E0 physical angle 0 degrees (wrist bent fully "forward/down"):
        private const int E0_FOR_TOF_SENSING_LOGICAL_POSITION = ARDUINO_REFERENCE_STEP_COUNT_FOR_90_DEG - (int)(90.0 * LOGICAL_STEPS_PER_DEGREE_E0); // Should be -25500. VERIFY THIS IS "FORWARD"!
        // If "forward" meant physical 180 degrees (wrist bent fully "up/back"), it would be +25500.
        // If "forward" meant physical 90 degrees (aligned with forearm), it would be 0.

        private const int GRAB_PRE_APPROACH_LOGICAL_Y = 0;  // USER MUST CALIBRATE! (Initial safe Y position)
        private const int GRAB_PRE_APPROACH_LOGICAL_Z = 0;  // USER MUST CALIBRATE! (Initial safe Z position)
        // This E0 is for the actual gripping action, might be different from TOF sensing orientation.
        private const int GRAB_GRIPPING_LOGICAL_E0 = 0; // USER MUST CALIBRATE! 

        private const int GRAB_APPROACH_NUDGE_STEPS = 100;
        private const int GRAB_APPROACH_NUDGE_Z_STEPS = 200; // USER TUNE: Larger initial nudge for Z
        private const int GRAB_APPROACH_NUDGE_Y_STEPS = 200; // USER TUNE: Nudge for Y
        private const int GRAB_APPROACH_NUDGE_Z_SLOW_STEPS = 50; // USER TUNE: Slower Z nudge once TOF detects something

        // E0 at 90 degrees physical (logical 0) for safe travel
        private const int E0_SAFE_TRAVEL_LOGICAL_POSITION = 0;
        // E0 "90 degrees forward" for TOF sensing.
        // With PositiveAccelStepIncreasesEncoder=true, moving to physical 0° requires negative steps from the 90° reference.
        //private const int E0_TOF_SENSING_LOGICAL_POSITION = ARDUINO_REFERENCE_STEP_COUNT_FOR_90_DEG - (int)(90.0 * LOGICAL_STEPS_PER_DEGREE_E0); // Should be -25500


        private const int GRAB_LIFT_LOGICAL_Y_OFFSET = -200; // USER MUST VERIFY DIRECTION!

        private const int FrameWidth = 1280;
        private const int FrameHeight = 720;
        private const double ConfidenceThreshold = 0.6;
        private const int PanServoID = 2;
        private const int TiltServoID = 3;
        private const int CenterAngle = 90;
        private const int MinAngle = 0;
        private const int MaxAngle = 180;
        private ControllerState _currentState = ControllerState.SearchingForPerson;
        private DateTime _personCenteredTimestamp = DateTime.MinValue;
        private readonly string _priorityObjectName = "bottle";
        private Random _rng = new Random();
        private TimeSpan _currentMonitoringDuration = TimeSpan.FromSeconds(5);
        private const double pixelDeadZone = 40;
        private int _currentPanAngle = CenterAngle;
        private int _currentTiltAngle = CenterAngle;
        private bool _isMoving = false;
        private bool _isPerformingGreeting = false;
        private DateTime _lastRobotCommandBatchTime = DateTime.MinValue;
        private readonly TimeSpan _robotCommandCycleInterval = TimeSpan.FromSeconds(0.5);
        private Dictionary<string, TrackedObjectMemory> _objectMemories = new Dictionary<string, TrackedObjectMemory>();
        private int _searchObjectAttemptCounter = 0;
        private const int MaxSearchObjectAttempts = 12000;
        private DateTime _timeToIgnorePersonUntil = DateTime.MinValue;
        private readonly TimeSpan _ignorePersonDuration = TimeSpan.FromMinutes(1.0);
        private readonly TimeSpan _briefTrackingLossThreshold = TimeSpan.FromSeconds(1.5);
        private int? _latestTofHandValue = null;
        private int? _latestTofCamValue = null;
        private const int GRIPPER_SERVO_ID = 4;
        private const int GRIPPER_OPEN_ANGLE = 30;
        private const int GRIPPER_CLOSED_ANGLE = 120;
        private DetectionResult? _grabTargetBottle = null;
        private double _grabTargetPanAngle = 0;
        private const int PanStepDegrees = 1;
        private const int TiltStepDegrees = 1;

        private static readonly ArmMotorConfig[] ArmMotors = new[] {
            new ArmMotorConfig("X",  0, false),
            new ArmMotorConfig("Y",  1, false),
            new ArmMotorConfig("Z",  2, true),
            new ArmMotorConfig("E0", 3, true)
        };

        private enum ControllerState
        { /* ... Same states ... */
            SearchingForPerson, CenteringPerson, MonitoringCenteredPerson,
            DedicatedSearchingForObject, DedicatedCenteringObject, ObjectIsCentered,
            MovingToLastKnownPosition, PerformingGreeting,
            IDLE_ARM_HOMED,
            ALIGNING_BASE_FOR_BOTTLE,
            SETTING_PRE_APPROACH_POSE,
            // New, simpler states
            APPROACH_WITH_Z,
            SEARCH_WITH_Y,
            FINAL_APPROACH_NUDGE,
            GRIPPING_BOTTLE,
            LIFTING_BOTTLE,
            BOTTLE_GRAB_COMPLETED,
            GRAB_FAILED_RECOVERY
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
            if (_isDisposed || !_isArmHomedAndSystemReady || _isHoming || _isPerformingGreeting || _isGrabSequenceActive)
            {
                //if (!_isArmHomedAndSystemReady && !_isHoming && !_isPerformingGreeting)
                //{
                //    Console.WriteLine($"[RobotController DEBUG] OnDetectionsReceived: Skipped - SystemReady: {_isArmHomedAndSystemReady}, Homing: {_isHoming}, Greeting: {_isPerformingGreeting}");
                //}
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
                            _isMoving = false; 
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

                case ControllerState.IDLE_ARM_HOMED: // New state after homing, ready for tasks


                case ControllerState.ObjectIsCentered:
                    Console.WriteLine($"[RobotController SM] Object '{_priorityObjectName}' is centered. Checking ignore timer. Person ignored until {(shouldPrioritizePerson ? "EXPIRED/INACTIVE" : _timeToIgnorePersonUntil.ToString("HH:mm:ss"))}.");
                    if ((_currentState == ControllerState.IDLE_ARM_HOMED || _currentState == ControllerState.ObjectIsCentered) &&
                 currentObject != null && currentObject.Label == TARGET_GRAB_OBJECT_LABEL &&
                 currentObject.Confidence > (ConfidenceThreshold + 0.1) &&
                 IsTargetEffectivelyCentered(currentObject))
                    {
                        if (!_isGrabSequenceActive) // Double check lock
                        {
                            Console.WriteLine($"[RobotController] OnDetectionsReceived: Target bottle '{TARGET_GRAB_OBJECT_LABEL}' is centered. Initiating grab sequence.");
                            _grabTargetBottle = currentObject;
                            _grabTargetPanAngle = _currentPanAngle;
                            _currentState = ControllerState.ALIGNING_BASE_FOR_BOTTLE; // Set initial grab state for the SM
                            _ = ProcessGrabStateMachineAsync();
                        }
                        else
                        {
                            Console.WriteLine($"[RobotController] OnDetectionsReceived: Grab conditions met, but grab already active.");
                        }
                        return;
                    }
                    // If not grabbing, continue with other logic for ObjectIsCentered or IDLE_ARM_HOMED
                    else if (_currentState == ControllerState.ObjectIsCentered)
                    {
                        // ... (your existing ObjectIsCentered logic if not grabbing a bottle) ...
                        // Example: Check if it should go back to searching for person if ignore timer expired
                        shouldPrioritizePerson = (_timeToIgnorePersonUntil == DateTime.MinValue || DateTime.UtcNow >= _timeToIgnorePersonUntil);
                        if (shouldPrioritizePerson)
                        {
                            _currentState = ControllerState.SearchingForPerson;
                        }
                        else
                        {
                            _currentState = ControllerState.DedicatedSearchingForObject; // Maintain focus on objects
                        }
                    }
                    else if (_currentState == ControllerState.IDLE_ARM_HOMED)
                    {
                        _currentState = ControllerState.SearchingForPerson; // Default to searching if idle
                    }
                    break;
            }
        }

        private async Task ProcessGrabStateMachineAsync()
        {
            if (_isHoming || _isPerformingGreeting || _isGrabSequenceActive)
            {
                Console.WriteLine($"[RobotController Grab SM] Cannot start grab: Homing={_isHoming}, Greeting={_isPerformingGreeting}, GrabActive={_isGrabSequenceActive}");
                return;
            }

            _isGrabSequenceActive = true; // Set the lock
            Console.WriteLine("[RobotController Grab SM] LOCK ACQUIRED, grab sequence starting.");


            try
            {
                bool grabActive = true;
                _currentState = ControllerState.ALIGNING_BASE_FOR_BOTTLE;

                while (grabActive && !_isDisposed && _isArmHomedAndSystemReady)
                {
                    Console.WriteLine($"[RobotController Grab SM] Current Grab State: {_currentState}");

                    switch (_currentState)
                    {
                        case ControllerState.ALIGNING_BASE_FOR_BOTTLE:
                            Console.WriteLine("[RobotController Grab SM] STEP 1: ALIGNING_BASE_FOR_BOTTLE with kinematics...");
                            if (!_latestTofCamValue.HasValue || _latestTofCamValue.Value <= 0)
                            {
                                Console.WriteLine($"[RobotController Grab SM] Grab failed: Invalid TOF reading from camera sensor: {_latestTofCamValue?.ToString() ?? "null"}. Cannot calculate object position.");
                                _currentState = ControllerState.GRAB_FAILED_RECOVERY;
                                break;
                            }

                            double cameraDistanceToObject = _latestTofCamValue.Value;
                            double cameraAngleDegrees = _grabTargetPanAngle - 90.0;
                            double cameraAngleRadians = cameraAngleDegrees * (Math.PI / 180.0);

                            double objectX = cameraDistanceToObject * Math.Sin(cameraAngleRadians);
                            double objectY = cameraDistanceToObject * Math.Cos(cameraAngleRadians);

                            double deltaX = objectX - CAMERA_TO_ARM_BASE_OFFSET_X_MM;
                            double deltaY = objectY - CAMERA_TO_ARM_BASE_OFFSET_Y_MM;

                            double armRotationRadians = Math.Atan2(deltaX, deltaY);
                            double armRotationDegrees = armRotationRadians * (180.0 / Math.PI);

                            int targetXLogical = ARDUINO_REFERENCE_STEP_COUNT_FOR_90_DEG + (int)Math.Round(armRotationDegrees * LOGICAL_STEPS_PER_DEGREE_X);
                            int clampedTargetX = ClampLogicalPosition("X", targetXLogical);

                            if (_currentLogicalAccelStepperPositions["X"] != clampedTargetX)
                            {
                                Console.WriteLine($"[RobotController Grab SM] Aligning Base. Cam Angle: {cameraAngleDegrees:F1}°, Obj Pos: (X:{objectX:F1}, Y:{objectY:F1}). Arm needs {armRotationDegrees:F1}° rotation. Target Logical: {clampedTargetX}");
                                _currentLogicalAccelStepperPositions["X"] = clampedTargetX;
                                _arduinoController.EnqueueArduinoCommand($"moveX{clampedTargetX}");
                                await Task.Delay(2500);
                            }
                            _currentState = ControllerState.SETTING_PRE_APPROACH_POSE;
                            break;

                        case ControllerState.SETTING_PRE_APPROACH_POSE:
                            Console.WriteLine("[RobotController Grab SM] STEP 2: SETTING INITIAL SAFE POSE...");
                            await OpenGripper();

                            int targetE0_Tof = ClampLogicalPosition("E0", E0_SAFE_TRAVEL_LOGICAL_POSITION);
                            Console.WriteLine($"[RobotController Grab SM] Setting E0 to safe travel position: {targetE0_Tof}");
                            _currentLogicalAccelStepperPositions["E0"] = targetE0_Tof;
                            _arduinoController.EnqueueArduinoCommand($"moveE0{targetE0_Tof}");
                            await Task.Delay(1500);

                            int targetY_Standby = ClampLogicalPosition("Y", SAFE_POSE_LOGICAL_Y);
                            int targetZ_Standby = ClampLogicalPosition("Z", SAFE_POSE_LOGICAL_Z);
                            _currentLogicalAccelStepperPositions["Y"] = targetY_Standby;
                            _currentLogicalAccelStepperPositions["Z"] = targetZ_Standby;
                            _arduinoController.EnqueueArduinoCommand($"moveY{targetY_Standby}");
                            _arduinoController.EnqueueArduinoCommand($"moveZ{targetZ_Standby}");
                            await Task.Delay(2000);

                            _currentState = ControllerState.APPROACH_WITH_Z;
                            break;

                        case ControllerState.APPROACH_WITH_Z:
                            Console.WriteLine("[RobotController Grab SM] Bending Z joint to maximum forward position...");
                            int zTargetForward = (KINEMATIC_Z_FORWARD_LOGICAL_DIRECTION == 1) ? MAX_LOGICAL_POS_Z : MIN_LOGICAL_POS_Z;
                            _currentLogicalAccelStepperPositions["Z"] = ClampLogicalPosition("Z", zTargetForward);
                            _arduinoController.EnqueueArduinoCommand($"moveZ{_currentLogicalAccelStepperPositions["Z"]}");
                            Console.WriteLine($"[RobotController Grab SM] Commanded Z to max forward: {_currentLogicalAccelStepperPositions["Z"]}");
                            await Task.Delay(4000); // Wait for the long Z move to complete

                            _currentState = ControllerState.SEARCH_WITH_Y; // Move on to Y regardless of TOF for now
                            break;

                        // ... inside the switch (_currentState) block of ProcessGrabStateMachineAsync ...

                        case ControllerState.SEARCH_WITH_Y:
                            Console.WriteLine($"[RobotController Grab SM] SEARCH_WITH_Y. TOF: {_latestTofHandValue?.ToString() ?? "N/A"}mm");

                            // Check TOF before moving Y
                            if (_latestTofHandValue.HasValue && _latestTofHandValue.Value > TOF_HAND_NO_OBJECT_OR_ERROR_MM)
                            {
                                Console.WriteLine($"[RobotController Grab SM] Object detected by TOF ({_latestTofHandValue.Value}mm). Switching to slow final nudge.");
                                _currentState = ControllerState.FINAL_APPROACH_NUDGE;
                                break;
                            }

                            // Nudge Y with the larger step value because we haven't found the object yet
                            int nextY = ClampLogicalPosition("Y", _currentLogicalAccelStepperPositions["Y"] + (KINEMATIC_Y_COMPENSATION_LOGICAL_DIRECTION * GRAB_APPROACH_NUDGE_Y_STEPS));

                            if (nextY == _currentLogicalAccelStepperPositions["Y"])
                            {
                                Console.WriteLine("[RobotController Grab SM] Y is at its limit during search. Grab failed.");
                                _currentState = ControllerState.GRAB_FAILED_RECOVERY;
                                break;
                            }

                            _currentLogicalAccelStepperPositions["Y"] = nextY;
                            _arduinoController.EnqueueArduinoCommand($"moveY{nextY}");
                            Console.WriteLine($"[RobotController Grab SM] Searching with Y, nudging to {nextY}");
                            await Task.Delay(600); // Wait for Y nudge
                            break;

                        case ControllerState.FINAL_APPROACH_NUDGE:
                            Console.WriteLine($"[RobotController Grab SM] FINAL_APPROACH_NUDGE. TOF: {_latestTofHandValue?.ToString() ?? "N/A"}mm");

                            // Fail if we lost the object during the slow approach
                            if (!_latestTofHandValue.HasValue || _latestTofHandValue.Value <= TOF_HAND_NO_OBJECT_OR_ERROR_MM)
                            {
                                Console.WriteLine("[RobotController Grab SM] Lost object during final nudge. Failing.");
                                _currentState = ControllerState.GRAB_FAILED_RECOVERY;
                                break;
                            }

                            // Success Condition: If we are in the perfect gripping zone
                            if (_latestTofHandValue.Value > TOF_HAND_TOO_CLOSE_MM && _latestTofHandValue.Value <= TOF_HAND_GRIPPING_DISTANCE_MM)
                            {
                                Console.WriteLine($"[RobotController Grab SM] Final nudge: In gripping distance ({_latestTofHandValue.Value}mm).");
                                _currentState = ControllerState.GRIPPING_BOTTLE;
                                break;
                            }

                            // Adjustment Condition 1: Too close, back up slowly
                            if (_latestTofHandValue.Value <= TOF_HAND_TOO_CLOSE_MM)
                            {
                                Console.WriteLine($"[RobotController Grab SM] Final nudge: Too close. Backing up Y slowly.");
                                int yBackup = ClampLogicalPosition("Y", _currentLogicalAccelStepperPositions["Y"] - (KINEMATIC_Y_COMPENSATION_LOGICAL_DIRECTION * GRAB_APPROACH_NUDGE_Z_SLOW_STEPS));
                                if (yBackup != _currentLogicalAccelStepperPositions["Y"])
                                {
                                    _currentLogicalAccelStepperPositions["Y"] = yBackup;
                                    _arduinoController.EnqueueArduinoCommand($"moveY{yBackup}");
                                    await Task.Delay(500);
                                }
                                else // Y is at limit, try backing up Z as a last resort
                                {
                                    Console.WriteLine("[RobotController Grab SM] Y at backup limit. Trying Z.");
                                    int zBackup = ClampLogicalPosition("Z", _currentLogicalAccelStepperPositions["Z"] - (KINEMATIC_Z_FORWARD_LOGICAL_DIRECTION * GRAB_APPROACH_NUDGE_Z_SLOW_STEPS));
                                    if (zBackup == _currentLogicalAccelStepperPositions["Z"])
                                    {
                                        Console.WriteLine("[RobotController Grab SM] Y and Z at backup limits. Failing.");
                                        _currentState = ControllerState.GRAB_FAILED_RECOVERY;
                                    }
                                    else
                                    {
                                        _currentLogicalAccelStepperPositions["Z"] = zBackup;
                                        _arduinoController.EnqueueArduinoCommand($"moveZ{zBackup}");
                                        await Task.Delay(500);
                                    }
                                }
                            }
                            // Adjustment Condition 2: Too far, move forward slowly
                            else if (_latestTofHandValue.Value > TOF_HAND_GRIPPING_DISTANCE_MM)
                            {
                                Console.WriteLine($"[RobotController Grab SM] Final nudge: Still too far. Nudging Y slowly.");
                                int yForward = ClampLogicalPosition("Y", _currentLogicalAccelStepperPositions["Y"] + (KINEMATIC_Y_COMPENSATION_LOGICAL_DIRECTION * GRAB_APPROACH_NUDGE_Z_SLOW_STEPS));
                                if (yForward != _currentLogicalAccelStepperPositions["Y"])
                                {
                                    _currentLogicalAccelStepperPositions["Y"] = yForward;
                                    _arduinoController.EnqueueArduinoCommand($"moveY{yForward}");
                                    await Task.Delay(500);
                                }
                                else // Y is at limit, try nudging Z as a last resort
                                {
                                    Console.WriteLine("[RobotController Grab SM] Y at forward limit. Trying Z.");
                                    int zForward = ClampLogicalPosition("Z", _currentLogicalAccelStepperPositions["Z"] + (KINEMATIC_Z_FORWARD_LOGICAL_DIRECTION * GRAB_APPROACH_NUDGE_Z_SLOW_STEPS));
                                    if (zForward == _currentLogicalAccelStepperPositions["Z"])
                                    {
                                        Console.WriteLine("[RobotController Grab SM] Y and Z at forward limits. Failing.");
                                        _currentState = ControllerState.GRAB_FAILED_RECOVERY;
                                    }
                                    else
                                    {
                                        _currentLogicalAccelStepperPositions["Z"] = zForward;
                                        _arduinoController.EnqueueArduinoCommand($"moveZ{zForward}");
                                        await Task.Delay(500);
                                    }
                                }
                            }
                            break;

                        case ControllerState.GRIPPING_BOTTLE:
                            Console.WriteLine("[RobotController Grab SM] GRIPPING_BOTTLE...");
                            await CloseGripper();
                            _currentState = ControllerState.LIFTING_BOTTLE;
                            break;

                        case ControllerState.LIFTING_BOTTLE:
                            Console.WriteLine("[RobotController Grab SM] LIFTING_BOTTLE...");
                            int targetYLift = ClampLogicalPosition("Y", _currentLogicalAccelStepperPositions["Y"] + GRAB_LIFT_LOGICAL_Y_OFFSET);
                            _currentLogicalAccelStepperPositions["Y"] = targetYLift;
                            _arduinoController.EnqueueArduinoCommand($"moveY{targetYLift}");
                            await Task.Delay(1500);
                            _currentState = ControllerState.BOTTLE_GRAB_COMPLETED;
                            break;

                        case ControllerState.BOTTLE_GRAB_COMPLETED:
                            Console.WriteLine("[RobotController Grab SM] BOTTLE_GRAB_COMPLETED.");
                            grabActive = false; // This will exit the loop
                            break;

                        case ControllerState.GRAB_FAILED_RECOVERY:
                            Console.WriteLine("[RobotController Grab SM] GRAB_FAILED_RECOVERY.");
                            await OpenGripper();
                            grabActive = false; // This will exit the loop
                            break;

                        default:
                            Console.WriteLine($"[RobotController Grab SM] Unexpected state {_currentState}. Ending grab.");
                            await OpenGripper();
                            grabActive = false; // This will exit the loop
                            break;
                    }
                    if (!grabActive) break;
                    await Task.Delay(50);
                }
            }
            finally
            {
                _isGrabSequenceActive = false; // Release the lock
                Console.WriteLine("[RobotController Grab SM] LOCK RELEASED, grab sequence ended/aborted.");
                // Ensure a sensible default state if the grab didn't complete successfully to IDLE_ARM_HOMED
                if (_currentState != ControllerState.IDLE_ARM_HOMED && _currentState != ControllerState.BOTTLE_GRAB_COMPLETED)
                {
                    Console.WriteLine($"[RobotController Grab SM] Grab sequence ended in non-final state: {_currentState}. Resetting to IDLE_ARM_HOMED.");
                    _currentState = ControllerState.IDLE_ARM_HOMED;
                }
            }
        }


        //private async Task ProcessGrabStateMachineAsync()
        //{
        //    if (_isHoming || _isPerformingGreeting)
        //    {
        //        Console.WriteLine("[RobotController Grab SM] Cannot start grab: Arm is homing or greeting.");
        //        _currentState = ControllerState.IDLE_ARM_HOMED;
        //        return;
        //    }

        //    if (_isGrabSequenceActive)
        //    {
        //        Console.WriteLine("[RobotController Grab SM] Attempted to start grab, but one is already active.");
        //        return;
        //    }

        //    _isGrabSequenceActive = true; // Set the lock
        //    Console.WriteLine("[RobotController Grab SM] LOCK ACQUIRED, grab sequence starting.");


        //    try
        //    {
        //        bool grabActive = true;
        //        int finalApproachAttemptCounter = 0;
        //        const int MAX_FINAL_APPROACH_ATTEMPTS = 10000;
        //        bool bottleLostVisuallyDuringApproach = false;
        //        int tofMovesWithoutVisionCount = 0;
        //        const int MAX_TOF_MOVES_WITHOUT_VISION = 10000;
        //        // DetectionResult? lastGoodVisionBottle = _grabTargetBottle; // Initialize with the bottle that triggered the grab

        //        while (grabActive && !_isDisposed && _isArmHomedAndSystemReady)
        //        {
        //            Console.WriteLine($"[RobotController Grab SM] Current Grab State: {_currentState}");
        //            var currentDetections = SignalController.Instance.GetLastDetections();
        //            var visionBottle = currentDetections?.FirstOrDefault(d => d.Label == TARGET_GRAB_OBJECT_LABEL && d.Confidence > ConfidenceThreshold);

        //            switch (_currentState)
        //            {
        //                case ControllerState.ALIGNING_BASE_FOR_BOTTLE:
        //                    Console.WriteLine("[RobotController Grab SM] ALIGNING_BASE_FOR_BOTTLE...");
        //                    double baseRotationDegrees = _grabTargetPanAngle - 90.0;
        //                    int targetXLogical = ARDUINO_REFERENCE_STEP_COUNT_FOR_90_DEG + (int)Math.Round(baseRotationDegrees * LOGICAL_STEPS_PER_DEGREE_X);
        //                    int clampedTargetX = ClampLogicalPosition("X", targetXLogical);

        //                    if (_currentLogicalAccelStepperPositions["X"] != clampedTargetX)
        //                    {
        //                        _currentLogicalAccelStepperPositions["X"] = clampedTargetX;
        //                        _arduinoController.EnqueueArduinoCommand($"moveX{clampedTargetX}");
        //                        await Task.Delay(1500);
        //                    }
        //                    _currentState = ControllerState.SETTING_PRE_APPROACH_POSE;
        //                    break;

        //                case ControllerState.SETTING_PRE_APPROACH_POSE:
        //                    Console.WriteLine("[RobotController Grab SM] SETTING_PRE_APPROACH_POSE: Aiming TOF, then moving to standby.");
        //                    await OpenGripper();

        //                    // 1. Aim E0 first
        //                    int targetE0_Tof = ClampLogicalPosition("E0", E0_FOR_TOF_SENSING_LOGICAL_POSITION);
        //                    Console.WriteLine($"[RobotController Grab SM] Aiming E0 for TOF to logical: {targetE0_Tof}");
        //                    _currentLogicalAccelStepperPositions["E0"] = targetE0_Tof;
        //                    _arduinoController.EnqueueArduinoCommand($"moveE0{targetE0_Tof}");
        //                    await Task.Delay(2000); // Wait for E0 to aim

        //                    // 2. Set Y/Z to standby positions
        //                    int targetY_Standby = ClampLogicalPosition("Y", GRAB_PRE_APPROACH_LOGICAL_Y);
        //                    int targetZ_Standby = ClampLogicalPosition("Z", GRAB_PRE_APPROACH_LOGICAL_Z);
        //                    _currentLogicalAccelStepperPositions["Y"] = targetY_Standby;
        //                    _currentLogicalAccelStepperPositions["Z"] = targetZ_Standby;
        //                    _arduinoController.EnqueueArduinoCommand($"moveY{targetY_Standby}");
        //                    _arduinoController.EnqueueArduinoCommand($"moveZ{targetZ_Standby}");
        //                    await Task.Delay(2000); // Wait for Y/Z to get to standby

        //                    _currentState = ControllerState.APPROACH_WITH_Z;
        //                    break;

        //                case ControllerState.APPROACH_WITH_Z:
        //                    Console.WriteLine("[RobotController Grab SM] Bending Z joint to maximum forward position...");
        //                    int zTargetForward = (KINEMATIC_Z_FORWARD_LOGICAL_DIRECTION == 1) ? MAX_LOGICAL_POS_Z : MIN_LOGICAL_POS_Z;
        //                    _currentLogicalAccelStepperPositions["Z"] = ClampLogicalPosition("Z", zTargetForward);
        //                    _arduinoController.EnqueueArduinoCommand($"moveZ{_currentLogicalAccelStepperPositions["Z"]}");
        //                    Console.WriteLine($"[RobotController Grab SM] Commanded Z to max forward: {_currentLogicalAccelStepperPositions["Z"]}");
        //                    await Task.Delay(4000); // Wait for the long Z move to complete

        //                    _currentState = ControllerState.SEARCH_WITH_Y; // Move on to Y regardless of TOF for now
        //                    break;

        //                // ... inside the switch (_currentState) block of ProcessGrabStateMachineAsync ...

        //                case ControllerState.SEARCH_WITH_Y:
        //                    Console.WriteLine($"[RobotController Grab SM] SEARCH_WITH_Y. TOF: {_latestTofHandValue?.ToString() ?? "N/A"}mm");

        //                    // Check TOF before moving Y
        //                    if (_latestTofHandValue.HasValue && _latestTofHandValue.Value > TOF_HAND_NO_OBJECT_OR_ERROR_MM)
        //                    {
        //                        Console.WriteLine($"[RobotController Grab SM] Object detected by TOF ({_latestTofHandValue.Value}mm). Switching to slow final nudge.");
        //                        _currentState = ControllerState.FINAL_APPROACH_NUDGE;
        //                        break;
        //                    }

        //                    // Nudge Y with the larger step value because we haven't found the object yet
        //                    int nextY = ClampLogicalPosition("Y", _currentLogicalAccelStepperPositions["Y"] + (KINEMATIC_Y_COMPENSATION_LOGICAL_DIRECTION * GRAB_APPROACH_NUDGE_Y_STEPS));

        //                    if (nextY == _currentLogicalAccelStepperPositions["Y"])
        //                    {
        //                        Console.WriteLine("[RobotController Grab SM] Y is at its limit during search. Grab failed.");
        //                        _currentState = ControllerState.GRAB_FAILED_RECOVERY;
        //                        break;
        //                    }

        //                    _currentLogicalAccelStepperPositions["Y"] = nextY;
        //                    _arduinoController.EnqueueArduinoCommand($"moveY{nextY}");
        //                    Console.WriteLine($"[RobotController Grab SM] Searching with Y, nudging to {nextY}");
        //                    await Task.Delay(600); // Wait for Y nudge
        //                    break;

        //                case ControllerState.FINAL_APPROACH_NUDGE:
        //                    Console.WriteLine($"[RobotController Grab SM] FINAL_APPROACH_NUDGE. TOF: {_latestTofHandValue?.ToString() ?? "N/A"}mm");

        //                    // Fail if we lost the object during the slow approach
        //                    if (!_latestTofHandValue.HasValue || _latestTofHandValue.Value <= TOF_HAND_NO_OBJECT_OR_ERROR_MM)
        //                    {
        //                        Console.WriteLine("[RobotController Grab SM] Lost object during final nudge. Failing.");
        //                        _currentState = ControllerState.GRAB_FAILED_RECOVERY;
        //                        break;
        //                    }

        //                    // Success Condition: If we are in the perfect gripping zone
        //                    if (_latestTofHandValue.Value > TOF_HAND_TOO_CLOSE_MM && _latestTofHandValue.Value <= TOF_HAND_GRIPPING_DISTANCE_MM)
        //                    {
        //                        Console.WriteLine($"[RobotController Grab SM] Final nudge: In gripping distance ({_latestTofHandValue.Value}mm).");
        //                        _currentState = ControllerState.GRIPPING_BOTTLE;
        //                        break;
        //                    }

        //                    // Adjustment Condition 1: Too close, back up slowly
        //                    if (_latestTofHandValue.Value <= TOF_HAND_TOO_CLOSE_MM)
        //                    {
        //                        Console.WriteLine($"[RobotController Grab SM] Final nudge: Too close. Backing up Y slowly.");
        //                        int yBackup = ClampLogicalPosition("Y", _currentLogicalAccelStepperPositions["Y"] - (KINEMATIC_Y_COMPENSATION_LOGICAL_DIRECTION * GRAB_APPROACH_NUDGE_Z_SLOW_STEPS));
        //                        if (yBackup != _currentLogicalAccelStepperPositions["Y"])
        //                        {
        //                            _currentLogicalAccelStepperPositions["Y"] = yBackup;
        //                            _arduinoController.EnqueueArduinoCommand($"moveY{yBackup}");
        //                            await Task.Delay(500);
        //                        }
        //                        else // Y is at limit, try backing up Z as a last resort
        //                        {
        //                            Console.WriteLine("[RobotController Grab SM] Y at backup limit. Trying Z.");
        //                            int zBackup = ClampLogicalPosition("Z", _currentLogicalAccelStepperPositions["Z"] - (KINEMATIC_Z_FORWARD_LOGICAL_DIRECTION * GRAB_APPROACH_NUDGE_Z_SLOW_STEPS));
        //                            if (zBackup == _currentLogicalAccelStepperPositions["Z"])
        //                            {
        //                                Console.WriteLine("[RobotController Grab SM] Y and Z at backup limits. Failing.");
        //                                _currentState = ControllerState.GRAB_FAILED_RECOVERY;
        //                            }
        //                            else
        //                            {
        //                                _currentLogicalAccelStepperPositions["Z"] = zBackup;
        //                                _arduinoController.EnqueueArduinoCommand($"moveZ{zBackup}");
        //                                await Task.Delay(500);
        //                            }
        //                        }
        //                    }
        //                    // Adjustment Condition 2: Too far, move forward slowly
        //                    else if (_latestTofHandValue.Value > TOF_HAND_GRIPPING_DISTANCE_MM)
        //                    {
        //                        Console.WriteLine($"[RobotController Grab SM] Final nudge: Still too far. Nudging Y slowly.");
        //                        int yForward = ClampLogicalPosition("Y", _currentLogicalAccelStepperPositions["Y"] + (KINEMATIC_Y_COMPENSATION_LOGICAL_DIRECTION * GRAB_APPROACH_NUDGE_Z_SLOW_STEPS));
        //                        if (yForward != _currentLogicalAccelStepperPositions["Y"])
        //                        {
        //                            _currentLogicalAccelStepperPositions["Y"] = yForward;
        //                            _arduinoController.EnqueueArduinoCommand($"moveY{yForward}");
        //                            await Task.Delay(500);
        //                        }
        //                        else // Y is at limit, try nudging Z as a last resort
        //                        {
        //                            Console.WriteLine("[RobotController Grab SM] Y at forward limit. Trying Z.");
        //                            int zForward = ClampLogicalPosition("Z", _currentLogicalAccelStepperPositions["Z"] + (KINEMATIC_Z_FORWARD_LOGICAL_DIRECTION * GRAB_APPROACH_NUDGE_Z_SLOW_STEPS));
        //                            if (zForward == _currentLogicalAccelStepperPositions["Z"])
        //                            {
        //                                Console.WriteLine("[RobotController Grab SM] Y and Z at forward limits. Failing.");
        //                                _currentState = ControllerState.GRAB_FAILED_RECOVERY;
        //                            }
        //                            else
        //                            {
        //                                _currentLogicalAccelStepperPositions["Z"] = zForward;
        //                                _arduinoController.EnqueueArduinoCommand($"moveZ{zForward}");
        //                                await Task.Delay(500);
        //                            }
        //                        }
        //                    }
        //                    break;

        //                case ControllerState.GRIPPING_BOTTLE:
        //                    Console.WriteLine("[RobotController Grab SM] GRIPPING_BOTTLE...");
        //                    await CloseGripper();
        //                    await Task.Delay(700);
        //                    _currentState = ControllerState.LIFTING_BOTTLE;
        //                    break;

        //                case ControllerState.LIFTING_BOTTLE:
        //                    Console.WriteLine("[RobotController Grab SM] LIFTING_BOTTLE...");
        //                    int targetYLift = ClampLogicalPosition("Y", _currentLogicalAccelStepperPositions["Y"] + GRAB_LIFT_LOGICAL_Y_OFFSET);
        //                    _currentLogicalAccelStepperPositions["Y"] = targetYLift;
        //                    _arduinoController.EnqueueArduinoCommand($"moveY{targetYLift}");
        //                    Console.WriteLine($"[RobotController Grab SM] Commanded Lift Y to {targetYLift}");
        //                    await Task.Delay(1500);
        //                    _currentState = ControllerState.BOTTLE_GRAB_COMPLETED;
        //                    break;

        //                case ControllerState.BOTTLE_GRAB_COMPLETED:
        //                    Console.WriteLine("[RobotController Grab SM] BOTTLE_GRAB_COMPLETED.");
        //                    grabActive = false;
        //                    _currentState = ControllerState.IDLE_ARM_HOMED;
        //                    break;

        //                case ControllerState.GRAB_FAILED_RECOVERY:
        //                    Console.WriteLine("[RobotController Grab SM] GRAB_FAILED_RECOVERY.");
        //                    await OpenGripper();
        //                    _currentState = ControllerState.IDLE_ARM_HOMED;
        //                    grabActive = false;
        //                    break;
        //                default:
        //                    Console.WriteLine($"[RobotController Grab SM] Unexpected state {_currentState}. Ending grab.");
        //                    await OpenGripper();
        //                    grabActive = false;
        //                    _currentState = ControllerState.IDLE_ARM_HOMED;
        //                    break;
        //            }
        //            if (!grabActive) break;
        //            await Task.Delay(50); // Main loop cycle delay - USER TUNE
        //        }
        //    }
        //    finally
        //    {
        //        _isGrabSequenceActive = false; // Release the lock
        //        Console.WriteLine("[RobotController Grab SM] LOCK RELEASED, grab sequence ended/aborted.");
        //        // Ensure a sensible default state if the grab didn't complete successfully to IDLE_ARM_HOMED
        //        if (_currentState != ControllerState.IDLE_ARM_HOMED && _currentState != ControllerState.BOTTLE_GRAB_COMPLETED)
        //        {
        //            Console.WriteLine($"[RobotController Grab SM] Grab sequence ended in non-final state: {_currentState}. Resetting to IDLE_ARM_HOMED.");
        //            _currentState = ControllerState.IDLE_ARM_HOMED;
        //        }
        //    }
        //}


        private int ClampLogicalPosition(string motorName, int desiredLogicalPosition)
        {
            var motorConfig = ArmMotors.FirstOrDefault(m => m.Name == motorName);
            if (motorConfig == null)
            {
                Console.WriteLine($"[RobotController Clamp] ERROR: No motor config found for {motorName}. Not clamping.");
                return desiredLogicalPosition;
            }

            // Skip clamping for X (base) for now, as it might have >180 deg range or continuous rotation.
            // You can add specific logic for X if it also has 0-180 limits you want to enforce.
            if (motorName == "X")
            {
                return desiredLogicalPosition;
            }

            double stepsPerDegree;
            switch (motorName)
            {
                case "Y": stepsPerDegree = LOGICAL_STEPS_PER_DEGREE_Y; break;
                case "Z": stepsPerDegree = LOGICAL_STEPS_PER_DEGREE_Z; break;
                case "E0": stepsPerDegree = LOGICAL_STEPS_PER_DEGREE_E0; break;
                default:
                    Console.WriteLine($"[RobotController Clamp] WARNING: No stepsPerDegree defined for motor {motorName}. Not clamping.");
                    return desiredLogicalPosition;
            }

            int minPhysicalAngleDegrees = 0;
            int maxPhysicalAngleDegrees = 180;
            int homePhysicalAngleDegrees = 90;

            int minLogicalLimit;
            int maxLogicalLimit;

            if (motorConfig.PositiveAccelStepIncreasesEncoder)
            {
                minLogicalLimit = ARDUINO_REFERENCE_STEP_COUNT_FOR_90_DEG - (int)((homePhysicalAngleDegrees - minPhysicalAngleDegrees) * stepsPerDegree);
                maxLogicalLimit = ARDUINO_REFERENCE_STEP_COUNT_FOR_90_DEG + (int)((maxPhysicalAngleDegrees - homePhysicalAngleDegrees) * stepsPerDegree);
            }
            else
            {
                minLogicalLimit = ARDUINO_REFERENCE_STEP_COUNT_FOR_90_DEG + (int)((homePhysicalAngleDegrees - minPhysicalAngleDegrees) * stepsPerDegree);
                maxLogicalLimit = ARDUINO_REFERENCE_STEP_COUNT_FOR_90_DEG - (int)((maxPhysicalAngleDegrees - homePhysicalAngleDegrees) * stepsPerDegree);
            }

            // Ensure the operational min is numerically smaller than max for Math.Clamp
            int operationalMin = Math.Min(minLogicalLimit, maxLogicalLimit);
            int operationalMax = Math.Max(minLogicalLimit, maxLogicalLimit);

            int clampedPosition = Math.Clamp(desiredLogicalPosition, operationalMin, operationalMax);

            if (clampedPosition != desiredLogicalPosition)
            {
                Console.WriteLine($"[RobotController Clamp] Motor {motorName}: Desired logical pos {desiredLogicalPosition} clamped to {clampedPosition}. (Logical limits for 0-180° phys: [{operationalMin} to {operationalMax}])");
            }
            return clampedPosition;
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
                else { Console.WriteLine("[RobotController] HandleStartupHomingRequestedAsync: Tool2ViewModel instance was null."); }
            }
            else { Console.WriteLine("[RobotController] HandleStartupHomingRequestedAsync: Startup homing already attempted/completed. Ignoring."); }
            return Task.CompletedTask;
        }

        private async void OnRemoteSystemReadyForActions()
        {
            Console.WriteLine("[RobotController] OnRemoteSystemReadyForActions: Received signal that remote system is ready.");
            if (_startupHomingRequestedByViewModel && _viewModelForStartupHoming != null && !_startupHomingAttemptCompleted)
            {
                Console.WriteLine("[RobotController] OnRemoteSystemReadyForActions: Executing PENDING startup homing sequence.");
                _startupHomingAttemptCompleted = true;

                bool homingSuccess = await AutoHomeArmTo90Async(_viewModelForStartupHoming);

                if (homingSuccess)
                {
                    Console.WriteLine("[RobotController] OnRemoteSystemReadyForActions: Arm homing SUCCESSFUL. System is now ready for detection logic.");
                    _isArmHomedAndSystemReady = true;
                    _currentState = ControllerState.IDLE_ARM_HOMED;
                }
                else
                {
                    Console.WriteLine("[RobotController] OnRemoteSystemReadyForActions: Arm homing FAILED. Detection logic will NOT start.");
                    _isArmHomedAndSystemReady = false;
                }
            }
            else
            {
                //if (_startupHomingAttemptCompleted) Console.WriteLine("[RobotController] OnRemoteSystemReadyForActions: Startup homing already attempted.");
                if (!_startupHomingRequestedByViewModel) Console.WriteLine("[RobotController] OnRemoteSystemReadyForActions: Startup homing was not requested by ViewModel.");
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

                const int greetingStepAmount = 1500; // Example step amount for E0
                const int greetingSegmentDelayMs = 700; // Example delay for movement segment
                const int interMovementDelayMs = 200; // Short pause between distinct parts

                // 1. Move E0 slightly "backward" (e.g., lift wrist up)
                Console.WriteLine("[RobotController Greeting] Move E0 backward (up)");
                _arduinoController.EnqueueArduinoCommand($"moveE0{greetingStepAmount}");
                await Task.Delay(greetingSegmentDelayMs);

                await Task.Delay(interMovementDelayMs);

                // 2. Move E0 "forward" from that new position (e.g., bring wrist down)
                Console.WriteLine("[RobotController Greeting] Move E0 forward (down)");
                _arduinoController.EnqueueArduinoCommand($"moveE0-{greetingStepAmount * 2}"); // Move more to go past original
                await Task.Delay(greetingSegmentDelayMs);

                await Task.Delay(interMovementDelayMs);

                // 3. Move E0 back to roughly the initial greeting up position
                Console.WriteLine("[RobotController Greeting] Move E0 backward (up) to initial greeting spot");
                _arduinoController.EnqueueArduinoCommand($"moveE0{greetingStepAmount * 2}");
                await Task.Delay(greetingSegmentDelayMs);

                await Task.Delay(interMovementDelayMs);

                // 4. Return E0 to its starting point before the greeting (neutralize the greeting motion)
                Console.WriteLine("[RobotController Greeting] Return E0 to neutral from greeting");
                _arduinoController.EnqueueArduinoCommand($"moveE0-{greetingStepAmount}");
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

        public async Task<bool> AutoHomeArmTo90Async(Tool2ViewModel tool2ViewModel)
        {
            if (_isHoming || _isPerformingGreeting)
            {
                Console.WriteLine("[RobotController Homing] AutoHomeArmTo90Async: Homing or Greeting already in progress.");
                return false;
            }
            _isHoming = true;
            Console.WriteLine("[RobotController Homing] AutoHomeArmTo90Async: Starting EXACT iterative arm homing with stability check...");

            bool allMotorsHomedSuccessfully = true;

            int[]? liveRawEncoders = null;
            const int encoderWaitTimeoutMs = 20000; // Increased to 20 seconds
            const int encoderPollIntervalMs = 100;
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
                Console.WriteLine($"[RobotController Homing] Still waiting for encoders... {sw.ElapsedMilliseconds}ms. LastRawEncoders is null: {tool2ViewModel.GetLastRawEncoderValues() == null}");
            }
            sw.Stop();

            if (liveRawEncoders == null || liveRawEncoders.Length != 4)
            {
                Console.WriteLine($"[RobotController Homing] AutoHomeArmTo90Async: TIMED OUT waiting for live encoder data. Aborting homing.");
                _isHoming = false;
                return false;
            }

            Dictionary<int, EncoderState> encoderStates = tool2ViewModel.GetEncoderStatesForHoming();
            if (encoderStates == null || !encoderStates.Any())
            {
                Console.WriteLine("[RobotController Homing] AutoHomeArmTo90Async: Could not get encoder states. Aborting homing.");
                _isHoming = false;
                return false;
            }

            // Using the class-level ArmMotors definition
            const int maxHomingAttemptsPerMotor = 10000;
            const int moveDelayMsBase = 500; // Increased for potentially slower physical response        
            const int ACCEL_STEPS_FOR_VERY_LARGE_ANGLE_DIFF = 100; // If > 2.0 deg (as per your request)
            const int ACCEL_STEPS_FOR_LARGE_ANGLE_DIFF = 25;  // Adjusted from 100 for safety, for > 2.0 deg
            const int ACCEL_STEPS_FOR_MEDIUM_ANGLE_DIFF = 5; // For 0.5 < angle diff <= 2.0 deg
            const int ACCEL_STEPS_FOR_SMALL_ANGLE_DIFF = 2;   // For 0.0 < angle diff <= 0.5 deg
            const int ACCEL_STEPS_FOR_FINE_NUDGE = 1;         // If rawDiff is non-zero but angle is very close

            _ = SendServoCommand(PanServoID, 90);
            _ = SendServoCommand(TiltServoID, 90);

            const int stabilityMonitorDurationMs = 2500; // Monitor for 2.5 seconds
            const int stabilityPollIntervalMs = 300;
            int readsRequiredForStable = Math.Max(1, stabilityMonitorDurationMs / stabilityPollIntervalMs);

            try
            {
                for (int i = 0; i < ArmMotors.Length; i++)
                {
                    var motor = ArmMotors[i];
                    bool thisMotorSuccessfullyHomedAndStable = false;

                    if (!encoderStates.TryGetValue(motor.EncoderIndex, out EncoderState? state) || !state.IsCalibrated)
                    {
                        Console.WriteLine($"[RobotController Homing] Motor {motor.Name}: Not calibrated. Skipping.");
                        allMotorsHomedSuccessfully = false;
                        continue;
                    }

                    int targetRawValue = state.CalibratedRawAt90Degrees;
                    Console.WriteLine($"[RobotController Homing] Motor {motor.Name}: TargetRaw={targetRawValue}. Initial LogicalPos={_currentLogicalAccelStepperPositions[motor.Name]}");

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
                        double currentAngle = tool2ViewModel.GetSpecificAngle(motor.EncoderIndex) ?? 90.0;
                        double angleDifference = 90.0 - currentAngle; // Positive if currentAngle < 90 (needs to increase angle)
                                                                      // Negative if currentAngle > 90 (needs to decrease angle)
                        double absAngleDiff = Math.Abs(angleDifference);

                        Console.WriteLine($"[RobotController Homing] Motor {motor.Name} Att {attempt + 1}/{maxHomingAttemptsPerMotor}: Raw={currentRawValue} (Target:{targetRawValue}, Diff:{rawDifference}), Angle={currentAngle:F1}° (Target:90°, Diff:{angleDifference:F1}°)");

                        if (rawDifference == 0) // Encoder is at the exact target raw value
                        {
                            Console.WriteLine($"[RobotController Homing] Motor {motor.Name}: Raw value matches target ({targetRawValue}). Monitoring for stability ({stabilityMonitorDurationMs / 1000}s)...");
                            bool stableAtTarget = true;
                            int stableReadCount = 0;
                            Stopwatch stabilitySwCheck = Stopwatch.StartNew(); // Corrected variable name
                            for (int k = 0; k < readsRequiredForStable; k++)
                            {
                                await Task.Delay(stabilityPollIntervalMs);
                                int[]? postMoveEncoders = tool2ViewModel.GetLastRawEncoderValues();
                                if (postMoveEncoders == null || postMoveEncoders.Length != 4) { Console.WriteLine($"[RobotController Homing Stability] Motor {motor.Name}: Lost encoder values."); stableAtTarget = false; break; }
                                int postMoveRawValue = postMoveEncoders[motor.EncoderIndex];
                                // Console.WriteLine($"[RobotController Homing Stability] Motor {motor.Name}: Check {k + 1}/{readsRequiredForStable}, Raw: {postMoveRawValue} (Target: {targetRawValue})");
                                if (postMoveRawValue != targetRawValue)
                                {
                                    Console.WriteLine($"[RobotController Homing Stability] Motor {motor.Name}: Value drifted to {postMoveRawValue} from {targetRawValue}. Unstable.");
                                    stableAtTarget = false; break;
                                }
                                stableReadCount++;
                            }
                            stabilitySwCheck.Stop();

                            if (stableAtTarget && stableReadCount >= (int)(readsRequiredForStable * 0.9)) // Require 90% of checks to be stable
                            {
                                Console.WriteLine($"[RobotController Homing] Motor {motor.Name}: STABLE and EXACTLY at target ({targetRawValue}).");
                                thisMotorSuccessfullyHomedAndStable = true;
                                break; // Exit the attempt loop for THIS MOTOR
                            }
                            else
                            {
                                Console.WriteLine($"[RobotController Homing Stability] Motor {motor.Name}: Value NOT stable (Stable reads: {stableReadCount}/{readsRequiredForStable}). Will re-nudge.");
                                var latestEncoders = tool2ViewModel.GetLastRawEncoderValues();
                                if (latestEncoders != null && latestEncoders.Length == 4)
                                {
                                    rawDifference = targetRawValue - latestEncoders[motor.EncoderIndex]; // Update rawDifference for next nudge
                                    angleDifference = 90.0 - (tool2ViewModel.GetSpecificAngle(motor.EncoderIndex) ?? 90.0); // Update angleDifference
                                    absAngleDiff = Math.Abs(angleDifference);
                                }
                                else { continue; }
                                if (rawDifference == 0) continue; // If it settled back to 0 raw diff, re-check stability
                            }
                        }

                        if (thisMotorSuccessfullyHomedAndStable) break;

                        int stepsToNudgeAccelStepper;
                        if (absAngleDiff > 2.0) { stepsToNudgeAccelStepper = ACCEL_STEPS_FOR_LARGE_ANGLE_DIFF; }
                        else if (absAngleDiff > 0.5) { stepsToNudgeAccelStepper = ACCEL_STEPS_FOR_MEDIUM_ANGLE_DIFF; }
                        else { stepsToNudgeAccelStepper = ACCEL_STEPS_FOR_SMALL_ANGLE_DIFF; }

                        // If angle is very close (within 0.5 deg), but raw is still off, use the finest nudge.
                        if (absAngleDiff <= 0.5 && rawDifference != 0)
                        {
                            stepsToNudgeAccelStepper = ACCEL_STEPS_FOR_FINE_NUDGE;
                        }

                        // Determine direction based on rawDifference to ensure we always move towards the target raw value
                        int directionalSignForEncoder = Math.Sign(rawDifference);
                        int stepperNudgeDirection;

                        if (motor.PositiveAccelStepIncreasesEncoder) { stepperNudgeDirection = directionalSignForEncoder; }
                        else { stepperNudgeDirection = -directionalSignForEncoder; }

                        // If after applying tiered logic, stepsToNudgeAccelStepper ended up 0 due to very small angle diff,
                        // but rawDifference is still non-zero, ensure we make at least a fine nudge.
                        if (stepsToNudgeAccelStepper == 0 && rawDifference != 0)
                        {
                            stepsToNudgeAccelStepper = ACCEL_STEPS_FOR_FINE_NUDGE;
                        }


                        int currentLogicalPosition = _currentLogicalAccelStepperPositions[motor.Name];
                        int newLogicalPosition = currentLogicalPosition + (stepperNudgeDirection * stepsToNudgeAccelStepper);

                        Console.WriteLine($"[RobotController Homing] Motor {motor.Name}: Nudging (AngleDiff {angleDifference:F1}deg, RawDiff {rawDifference}) from logical {currentLogicalPosition} to {newLogicalPosition} (NudgeAmount: {stepsToNudgeAccelStepper})");
                        _arduinoController.EnqueueArduinoCommand($"move{motor.Name}{newLogicalPosition}");
                        _currentLogicalAccelStepperPositions[motor.Name] = newLogicalPosition;

                        await Task.Delay(moveDelayMsBase + (int)(stepsToNudgeAccelStepper * 0.2)); // Dynamic delay

                        if (attempt == maxHomingAttemptsPerMotor - 1 && !thisMotorSuccessfullyHomedAndStable)
                        {
                            Console.WriteLine($"[RobotController Homing] Motor {motor.Name} FAILED to reach and stabilize at exact target. FinalRaw={currentRawValue}, Target={targetRawValue}");
                            allMotorsHomedSuccessfully = false;
                        }
                    }

                    if (!thisMotorSuccessfullyHomedAndStable) { allMotorsHomedSuccessfully = false; }

                    Console.WriteLine($"[RobotController Homing] Motor {motor.Name} loop finished. Setting Arduino current pos to {ARDUINO_REFERENCE_STEP_COUNT_FOR_90_DEG}. Current Logical Pos was: {_currentLogicalAccelStepperPositions[motor.Name]}");
                    _arduinoController.EnqueueArduinoCommand($"setcurrentposition{motor.Name}{ARDUINO_REFERENCE_STEP_COUNT_FOR_90_DEG}");
                    _currentLogicalAccelStepperPositions[motor.Name] = ARDUINO_REFERENCE_STEP_COUNT_FOR_90_DEG;
                    await Task.Delay(100);
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[RobotController Homing] CRITICAL Error: {ex.Message} {ex.StackTrace}");
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