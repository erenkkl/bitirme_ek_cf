using UnityEngine;
using System.Collections.Generic;
using System.IO;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class SiftPathFollower : MonoBehaviour
{
    public enum DeviationDirection
    {
        Forward,
        Left90,
        Right90
    }

    private enum MissionState
    {
        WAITING_FOR_DATA,
        FLYING,
        HOVERING,
        BACKTRACKING,
        RELOCKING,
        ARRIVED,
        LANDING,
        COMPLETED
    }

    [Header("ROS Settings")]
    public string poseTopic = "/drone/sift_pose";
    public string setDbTopic = "/drone/sift/set_db";
    public string stateTopic = "/drone/state";
    public string referencesCsvPath = "/home/fidan/bitirme_dataset/references.csv";

    [Header("Drone Components")]
    public DronePhysics drone;
    public MissionCompleteUI missionCompleteUI;

    [Header("Navigation")]
    public float lookAhead = 4.0f;
    [Range(0f, 1f)]
    public float minConfidence = 0.4f;

    [Header("Safety Filters")]
    public bool useSmoothing = true;
    public int seqWindowSize = 5;
    public int lostTimeoutFrames = 10;
    public float watchdogTimeout = 2.0f;

    [Header("Recovery Settings")]
    public float hoverDuration = 1.0f;
    public float backtrackSpeed = -3.0f;
    public float backtrackTimeout = 10.0f;
    public float relockWaitTime = 1.5f;
    public float relockAttemptDuration = 3.0f; // How long to keep trying relock before giving up
    public float recoveryCooldown = 5.0f;

    [Header("Control Settings")]
    public float forwardSpeed = 15.0f;
    // public float minSpeed = 5.0f; // Artık kullanılmıyor (adaptive speed kaldırıldı)
    public float yawGain = 1.0f;
    public float visualGainX = 0.05f;
    public float backtrackVisualGainX = 0.1f;
    public float backtrackYawGain = 0.5f;
    public float backtrackDeadband = 15.0f;
    public float backtrackRotationDeadband = 5.0f;
    public bool invertRoll = true;
    public float deadband = 10.0f;

    [Header("Landing Settings")]
    public float descentSpeed = 5.0f;
    public float groundAltitude = 0.5f;

    [Header("Deviation Test Settings")]
    public KeyCode deviationTestKey = KeyCode.T;
    public DeviationDirection deviationDirection = DeviationDirection.Forward;
    public float deviationTestSpeed = 15.0f;
    public float deviationTestDuration = 3.0f;
    public float deviationYawSpeed = 30.0f; // degrees per second for 90 degree turns

    [Header("Debug")]
    public bool debugLogs = true;
    public float debugLogPeriod = 1.0f;

    private ROSConnection ros;
    private List<Vector3> mapPoints = new List<Vector3>();
    private Queue<float> seqHistory = new Queue<float>();

    private float rawSeq = -1;
    private float visualErrX = 0;
    private float currentConf = 0;
    private float currentMode = 0;
    private float currentRecConf = 0;
    private float currentRotation = 0;
    private float estimatedSeq = -1;
    private float effectiveErrorX = 0;
    private float lastLogTime;
    private bool isInitialized = false;
    private float lastRosMsgTime = 0f;

    private MissionState currentState = MissionState.WAITING_FOR_DATA;
    private int lostCounter = 0;
    private int consistentSafeFrames = 0;
    private float missionStartTime = 0f;
    private float stateStartTime = 0f;
    private float recoveryEndTime = -999f;
    private float attemptedRelockSeq = -1;
    private float lastFailedRelockSeq = -1;

    // Deviation Test State
    private bool isDeviationTestActive = false;
    private float deviationTestStartTime = 0f;
    private float deviationTargetYaw = 0f;
    private bool deviationYawComplete = false;
    
    // State publishing
    private float lastStatePublishTime = 0f;
    private float statePublishInterval = 0.1f;

    void Start()
    {
        if (drone == null) drone = GetComponent<DronePhysics>();
        if (drone != null) drone.useGPS = false;

        LoadReferenceMap();

        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<Float32MultiArrayMsg>(poseTopic, OnPoseReceived);
        ros.RegisterPublisher<StringMsg>(setDbTopic);
        ros.RegisterPublisher<StringMsg>(stateTopic);

        currentState = MissionState.WAITING_FOR_DATA;
        lastRosMsgTime = Time.time;
        isInitialized = true;
        Debug.Log($"[SiftPathFollower] Ready. Watchdog: {watchdogTimeout}s");

        ChangeDB("MAP");
    }

    void OnPoseReceived(Float32MultiArrayMsg msg)
    {
        lastRosMsgTime = Time.time;
        if (msg.data.Length < 6) return;

        float incomingSeq = msg.data[0];
        currentMode = msg.data[4];

        if (msg.data.Length >= 7) currentRecConf = msg.data[6];
        else currentRecConf = 0;

        if (msg.data.Length >= 8) currentRotation = msg.data[7];
        else currentRotation = 0;

        if (incomingSeq != -1)
        {
            rawSeq = incomingSeq;
            visualErrX = msg.data[1];
            currentConf = msg.data[3];

            if (useSmoothing)
            {
                seqHistory.Enqueue(rawSeq);
                if (seqHistory.Count > seqWindowSize) seqHistory.Dequeue();
                float sum = 0;
                foreach (float s in seqHistory) sum += s;
                estimatedSeq = sum / seqHistory.Count;
            }
            else estimatedSeq = rawSeq;
        }
        else
        {
            rawSeq = -1;
            currentConf = 0;
        }
    }

    void ChangeDB(string mode)
    {
        if (!isInitialized) return;
        ros.Publish(setDbTopic, new StringMsg(mode));
    }

    void PublishState()
    {
        if (!isInitialized) return;
        
        string state = currentState.ToString();
        string deviationStatus = "NORMAL";
        
        if (isDeviationTestActive)
        {
            string phase = deviationYawComplete ? "MOVING" : "ROTATING";
            string direction = deviationDirection.ToString();
            deviationStatus = $"DEVIATION_{phase}_{direction}";
        }
        
        string stateMsg = $"{state}|{deviationStatus}";
        ros.Publish(stateTopic, new StringMsg(stateMsg));
    }

    void Update()
    {
        // Check for deviation test trigger
        if (Input.GetKeyDown(deviationTestKey) && !isDeviationTestActive)
        {
            if (currentState == MissionState.FLYING)
            {
                isDeviationTestActive = true;
                deviationTestStartTime = Time.time;
                deviationYawComplete = false;
                
                // Calculate target yaw based on direction
                float currentYaw = transform.eulerAngles.y;
                switch (deviationDirection)
                {
                    case DeviationDirection.Forward:
                        deviationTargetYaw = currentYaw;
                        deviationYawComplete = true; // No yaw needed
                        break;
                    case DeviationDirection.Left90:
                        deviationTargetYaw = currentYaw - 60f;
                        break;
                    case DeviationDirection.Right90:
                        deviationTargetYaw = currentYaw + 60f;
                        break;
                }
                
                // Normalize to 0-360
                while (deviationTargetYaw < 0) deviationTargetYaw += 360f;
                while (deviationTargetYaw >= 360) deviationTargetYaw -= 360f;
                
                string directionStr = deviationDirection == DeviationDirection.Forward ? "Forward" :
                                     deviationDirection == DeviationDirection.Left90 ? "Left 90°" : "Right 90°";
                Debug.Log($"[DEVIATION TEST] Started! Direction: {directionStr}, Speed: {deviationTestSpeed}, Duration: {deviationTestDuration}s");
            }
            else
            {
                Debug.LogWarning($"[DEVIATION TEST] Can only activate during FLYING state. Current: {currentState}");
            }
        }
    }

    void FixedUpdate()
    {
        if (!isInitialized || drone == null || mapPoints.Count == 0) return;

        // Handle deviation test mode
        if (isDeviationTestActive)
        {
            float elapsed = Time.time - deviationTestStartTime;
            if (elapsed >= deviationTestDuration)
            {
                isDeviationTestActive = false;
                Debug.Log($"[DEVIATION TEST] Completed! Resuming normal flight.");
            }
            else
            {
                float pitchCmd = 0;
                float rollCmd = 0;
                float yawCmd = 0;
                
                // First, handle yaw rotation if needed
                if (!deviationYawComplete)
                {
                    float currentYaw = transform.eulerAngles.y;
                    float yawError = Mathf.DeltaAngle(currentYaw, deviationTargetYaw);
                    
                    if (Mathf.Abs(yawError) > 2.0f) // 2 degree tolerance
                    {
                        // Apply yaw command
                        yawCmd = Mathf.Sign(yawError) * deviationYawSpeed;
                        yawCmd = Mathf.Clamp(yawCmd, -drone.maxYawTorqueCmd, drone.maxYawTorqueCmd);
                    }
                    else
                    {
                        // Yaw complete, start moving forward
                        deviationYawComplete = true;
                        Debug.Log($"[DEVIATION TEST] Yaw complete! Now moving forward.");
                    }
                }
                else
                {
                    // Yaw complete, move forward
                    pitchCmd = deviationTestSpeed;
                }
                
                drone.SetExternalCommand(pitchCmd, rollCmd, yawCmd);
                
                if (debugLogs && Time.time - lastLogTime > debugLogPeriod)
                {
                    lastLogTime = Time.time;
                    string phase = deviationYawComplete ? "Moving" : "Rotating";
                    string directionStr = deviationDirection == DeviationDirection.Forward ? "Forward" :
                                         deviationDirection == DeviationDirection.Left90 ? "Left 90°" : "Right 90°";
                    Debug.Log($"🧪 [DEVIATION TEST] {phase} ({directionStr}) {elapsed:F1}s / {deviationTestDuration:F1}s - Pitch:{pitchCmd:F1} Yaw:{yawCmd:F1}");
                }
                return; // Skip normal state handling
            }
        }

        if (Time.time - lastRosMsgTime > watchdogTimeout)
        {
            if (currentState != MissionState.LANDING && currentState != MissionState.COMPLETED)
            {
                Debug.LogError($"[WATCHDOG] ROS Timeout! ({Time.time - lastRosMsgTime:F1}s) -> Emergency Landing.");
                drone.SetExternalCommand(0, 0, 0);
                currentState = MissionState.LANDING;
                return;
            }
        }

        // Publish state periodically
        if (Time.time - lastStatePublishTime > statePublishInterval)
        {
            lastStatePublishTime = Time.time;
            PublishState();
        }

        switch (currentState)
        {
            case MissionState.WAITING_FOR_DATA: HandleWaitingState(); break;
            case MissionState.FLYING: HandleFlyingState(); break;
            case MissionState.HOVERING: HandleHoveringState(); break;
            case MissionState.BACKTRACKING: HandleBacktrackingState(); break;
            case MissionState.RELOCKING: HandleRelockingState(); break;
            case MissionState.ARRIVED: HandleArrivedState(); break;
            case MissionState.LANDING: HandleLandingState(); break;
        }
    }

    private void HandleWaitingState()
    {
        bool isDataGood = (rawSeq != -1) && (currentConf >= minConfidence);

        if (isDataGood && Mathf.Abs(currentMode - 0.0f) < 0.1f)
        {
            currentState = MissionState.FLYING;
            missionStartTime = Time.time;
            Debug.Log("[WAITING] Visual match established! Flight starting...");
        }
        else
        {
            if (Mathf.Abs(currentMode - 1.0f) < 0.1f) ChangeDB("MAP");
            drone.SetExternalCommand(0, 0, 0);

            if (debugLogs && Time.time - lastLogTime > debugLogPeriod)
            {
                lastLogTime = Time.time;
                Debug.Log($"[WAITING] Waiting for match... (Conf:{currentConf:F2})");
            }
        }
    }

    private void HandleFlyingState()
    {
        bool isDataBad = (rawSeq == -1) || (currentConf < minConfidence);
        string logStatus = isDataBad ? "BUFFERING" : "FLYING";

        if (isDataBad)
        {
            lostCounter++;
            if (lostCounter > lostTimeoutFrames)
            {
                Debug.LogWarning($"[LOST] Map lost! Starting recovery... ({lostCounter} frames)");
                currentState = MissionState.HOVERING;
                stateStartTime = Time.time;
                lostCounter = 0;
                drone.SetExternalCommand(0, 0, 0);
                ChangeDB("HISTORY");
                return;
            }
        }
        else lostCounter = 0;

        float pitchCmd = 0, rollCmd = 0, yawCmd = 0;
        string dirStr = "CENTER";
        float targetSeq = estimatedSeq + lookAhead;

        if (isDataBad)
        {
            pitchCmd = 5.0f; // Sabit güvenli hız (signal lost durumu)
            dirStr = "LOST";
        }
        else
        {
            if (Mathf.Abs(visualErrX) > deadband)
            {
                if (visualErrX > 0) { effectiveErrorX = visualErrX - deadband; dirStr = "RIGHT"; }
                else { effectiveErrorX = visualErrX + deadband; dirStr = "LEFT"; }
            }
            else effectiveErrorX = 0;

            rollCmd = effectiveErrorX * visualGainX;
            if (invertRoll) rollCmd = -rollCmd;

            if (estimatedSeq >= mapPoints.Count - 2)
            {
                currentState = MissionState.ARRIVED;
                Debug.Log("[FLYING] Target reached. Landing...");
                pitchCmd = 0;
            }
            else
            {
                // Sabit hız (adaptive speed kaldırıldı)
                pitchCmd = forwardSpeed;
            }

            Vector3 virtualPos = GetInterpolatedPoint(estimatedSeq);
            Vector3 targetPos = GetInterpolatedPoint(targetSeq);
            Vector3 pathVec = targetPos - virtualPos;
            
            if (pathVec.sqrMagnitude > 0.01f)
            {
                float targetHead = Mathf.Atan2(pathVec.x, pathVec.z) * Mathf.Rad2Deg;
                float yawErr = Mathf.DeltaAngle(transform.eulerAngles.y, targetHead);
                yawCmd = Mathf.Clamp(yawErr * yawGain, -drone.maxYawTorqueCmd, drone.maxYawTorqueCmd);
            }

            rollCmd = Mathf.Clamp(rollCmd, -20f, 20f);
        }

        drone.SetExternalCommand(pitchCmd, rollCmd, yawCmd);

        if (debugLogs && Time.time - lastLogTime > debugLogPeriod)
        {
            lastLogTime = Time.time;
            string prefix = (logStatus == "BUFFERING") ? "⚠️ " : "";
            Debug.Log($"{prefix}[{logStatus}] Seq:{estimatedSeq:F0} | Target:{targetSeq:F0} | Conf:{currentConf:F2} | Roll:{rollCmd:F1} | Yaw:{yawCmd:F1} | {dirStr}");
        }
    }

    private void HandleHoveringState()
    {
        drone.SetExternalCommand(0, 0, 0);
        if (Time.frameCount % 10 == 0) ChangeDB("HISTORY");

        if (Time.time - stateStartTime > hoverDuration)
        {
            Debug.Log("[HOVERING] Hover complete. Starting backtrack...");
            currentState = MissionState.BACKTRACKING;
            stateStartTime = Time.time;
            consistentSafeFrames = 0;
            lastFailedRelockSeq = -1;
        }
    }

    private void HandleBacktrackingState()
    {
        float rollCmd = 0;
        string dirStr = "CENTER";

        if (Mathf.Abs(currentMode - 1.0f) > 0.1f && Time.frameCount % 30 == 0) ChangeDB("HISTORY");

        if (rawSeq != -1 && currentConf > 0.0f)
        {
            float localEffectiveErr = 0;
            if (Mathf.Abs(visualErrX) > backtrackDeadband)
            {
                if (visualErrX > 0)
                {
                    localEffectiveErr = visualErrX - backtrackDeadband;
                    dirStr = "RIGHT";
                }
                else
                {
                    localEffectiveErr = visualErrX + backtrackDeadband;
                    dirStr = "LEFT";
                }
            }

            rollCmd = localEffectiveErr * backtrackVisualGainX;
            if (invertRoll) rollCmd = -rollCmd;
            rollCmd = Mathf.Clamp(rollCmd, -10f, 10f);
        }
        else dirStr = "LOST";

        float yawCmd = 0;
        if (Mathf.Abs(currentRotation) > backtrackRotationDeadband)
        {
            yawCmd = -currentRotation * backtrackYawGain;
            yawCmd = Mathf.Clamp(yawCmd, -drone.maxYawTorqueCmd, drone.maxYawTorqueCmd);
        }

        drone.SetExternalCommand(backtrackSpeed, rollCmd, yawCmd);

        bool isFrameSafe = (rawSeq != -1) && (currentRecConf >= 0.95f) && (Mathf.Abs(rawSeq - lastFailedRelockSeq) > 3.0f);

        if (isFrameSafe) consistentSafeFrames++;
        else consistentSafeFrames = 0;

        if (consistentSafeFrames >= 3)
        {
            Debug.Log($"[BACKTRACK] Safe point validated! (Seq:{rawSeq}) Attempting relock...");
            currentState = MissionState.RELOCKING;
            stateStartTime = Time.time;
            attemptedRelockSeq = rawSeq;
            ChangeDB("MAP");
            consistentSafeFrames = 0;
            return;
        }

        if (Time.time - stateStartTime > backtrackTimeout)
        {
            Debug.LogError($"[BACKTRACK] Timeout ({backtrackTimeout}s). Emergency landing.");
            currentState = MissionState.LANDING;
        }

        if (debugLogs && Time.time - lastLogTime > debugLogPeriod)
        {
            lastLogTime = Time.time;
            Debug.Log($"⏪ [BACKTRACK] Seq:{rawSeq:F0} | Conf:{currentConf:F2} | RecConf:{currentRecConf:F2} | Rot:{currentRotation:F1}° | Yaw:{yawCmd:F1} | Roll:{rollCmd:F1} | {dirStr} | Skip:{lastFailedRelockSeq:F0}");
        }
    }

    private void HandleRelockingState()
    {
        drone.SetExternalCommand(0, 0, 0);
        if (Time.frameCount % 10 == 0) ChangeDB("MAP");

        // Wait minimum time for DB to load and SIFT to process
        float elapsedTime = Time.time - stateStartTime;
        
        if (elapsedTime > relockWaitTime)
        {
            bool mapMatch = (rawSeq != -1) && (currentConf >= minConfidence) && (Mathf.Abs(currentMode - 0.0f) < 0.1f);

            if (mapMatch)
            {
                Debug.Log("✅ [RELOCK] Map locked! Resuming mission.");
                currentState = MissionState.FLYING;
                lostCounter = 0;
                recoveryEndTime = Time.time;
                lastFailedRelockSeq = -1;
            }
            else if (elapsedTime > relockAttemptDuration)
            {
                // Only give up after full attempt duration
                Debug.LogWarning($"❌ [RELOCK] Failed after {relockAttemptDuration}s. Seq {attemptedRelockSeq} blacklisted. Continuing backtrack.");
                currentState = MissionState.BACKTRACKING;
                stateStartTime = Time.time;
                consistentSafeFrames = 0;
                lastFailedRelockSeq = attemptedRelockSeq;
                ChangeDB("HISTORY");
            }
            else if (debugLogs && Time.time - lastLogTime > debugLogPeriod)
            {
                // Still trying...
                lastLogTime = Time.time;
                Debug.Log($"[RELOCK] Attempting... ({elapsedTime:F1}s / {relockAttemptDuration:F1}s) | Conf:{currentConf:F2}");
            }
        }
    }

    private void HandleArrivedState()
    {
        drone.SetExternalCommand(0, 0, 0);
        currentState = MissionState.LANDING;
    }

    private void HandleLandingState()
    {
        drone.SetExternalCommand(0, 0, 0);
        if (drone.targetAltitude > groundAltitude)
        {
            drone.targetAltitude -= descentSpeed * Time.fixedDeltaTime;
            drone.targetAltitude = Mathf.Max(drone.targetAltitude, groundAltitude);
            if (debugLogs && Time.time - lastLogTime > debugLogPeriod)
            {
                lastLogTime = Time.time;
                Debug.Log($"[LANDING] Altitude: {drone.targetAltitude:F2}m");
            }
        }
        else
        {
            currentState = MissionState.COMPLETED;
            CompleteMission();
        }
    }

    private void CompleteMission()
    {
        float duration = Time.time - missionStartTime;
        Debug.Log($"[COMPLETED] Mission complete! Duration: {duration:F2}s");
        if (missionCompleteUI != null) missionCompleteUI.ShowWithDuration(duration);
        drone.SetExternalCommand(0, 0, 0);
    }

    Vector3 GetInterpolatedPoint(float seqIndex)
    {
        seqIndex = Mathf.Clamp(seqIndex, 0, mapPoints.Count - 1.001f);
        int i = Mathf.FloorToInt(seqIndex);
        float t = seqIndex - i;
        return Vector3.Lerp(mapPoints[i], mapPoints[i + 1], t);
    }

    void LoadReferenceMap()
    {
        mapPoints.Clear();
        if (!File.Exists(referencesCsvPath)) return;
        try
        {
            string[] lines = File.ReadAllLines(referencesCsvPath);
            for (int i = 1; i < lines.Length; i++)
            {
                string[] parts = lines[i].Split(',');
                if (parts.Length >= 4)
                    mapPoints.Add(new Vector3(float.Parse(parts[1]), float.Parse(parts[2]), float.Parse(parts[3])));
            }
        }
        catch { }
    }

    public float GetLastMatchedSeq() { return rawSeq; }
    public float GetCurrentConfidence() { return currentConf; }

    public bool IsInRecoveryMode
    {
        get
        {
            bool isRecovering = (currentState == MissionState.HOVERING ||
                                 currentState == MissionState.BACKTRACKING ||
                                 currentState == MissionState.RELOCKING);
            bool isCoolingDown = (Time.time - recoveryEndTime < recoveryCooldown);
            return isRecovering || isCoolingDown;
        }
    }
}