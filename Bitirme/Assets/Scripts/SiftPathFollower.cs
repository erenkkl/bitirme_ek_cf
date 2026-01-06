using UnityEngine;
using System.Collections.Generic;
using System.IO;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class SiftPathFollower : MonoBehaviour
{
    // Mission state enum
    private enum MissionState
    {
        FLYING,
        ARRIVED,
        LANDING,
        COMPLETED
    }

    [Header("ROS Settings")]
    public string poseTopic = "/drone/sift_pose";
    public string avoidanceCommandTopic = "/drone/avoidance_command";
    public string referencesCsvPath = "/home/eren/bitirme_repo/bitirme_dataset/references.csv";

    [Header("Drone Components")]
    public DronePhysics drone;

    [Header("Navigation")]
    public float lookAhead = 4.0f;
    public float baseLookAhead = 4.0f; // Store original value

    [Header("Obstacle Avoidance Integration")]
    [Tooltip("Enable obstacle avoidance integration")]
    public bool enableAvoidance = true;

    [Tooltip("Blending factor for avoidance roll (0=ignore, 1=full)")]
    [Range(0f, 1f)]
    public float avoidanceBlendFactor = 0.7f;

    [Tooltip("Smoothing for avoidance transitions")]
    public float avoidanceSmoothness = 5.0f;

    [Header("Safety Filters")]
    public bool useSmoothing = true;
    public int seqWindowSize = 2;
    
    [Header("Control Settings")]
    public float forwardSpeed = 15.0f;    
    public float yawGain = 1.0f;         
    public float visualGainX = 0.05f;    // Lateral Düzeltme Gücü
    public bool invertRoll = true;      
    public float deadband = 10.0f;       // Titreşim Önleyici Eşik

    [Header("Landing Settings")]
    [Tooltip("Descent speed in m/s when landing")]
    public float descentSpeed = 5.0f;
    [Tooltip("Ground altitude threshold to consider mission completed")]
    public float groundAltitude = 0.5f;

    [Header("Mission UI")]
    public MissionCompleteUI missionCompleteUI;

    [Header("Debug")]
    public bool debugLogs = true;
    public float debugLogPeriod = 0.5f;

    // --- Internal State ---
    private ROSConnection ros;
    private List<Vector3> mapPoints = new List<Vector3>();
    private Queue<float> seqHistory = new Queue<float>();

    private float rawSeq = -1;
    private float visualErrX = 0;

    private float estimatedSeq = -1;
    private float effectiveErrorX = 0;
    private float lastLogTime;
    private bool isInitialized = false;

    // Mission state tracking
    private MissionState currentState = MissionState.FLYING;
    private float missionStartTime = 0f;
    private float flightDuration = 0f;

    // Avoidance command state
    private bool avoidanceActive = false;
    private float avoidanceRollAdjustment = 0f;
    private float avoidanceSpeedMultiplier = 1f;
    private int avoidanceMode = 0; // 0=normal, 1=avoiding, 2=recovering
    private float avoidanceLookAheadMult = 1f;

    // Smoothed avoidance values
    private float currentAvoidanceRoll = 0f;

    void Start()
    {
        if (drone == null) drone = GetComponent<DronePhysics>();
        if (drone != null) drone.useGPS = false; 

        LoadReferenceMap();

        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<Float32MultiArrayMsg>(poseTopic, OnPoseReceived);

        // Subscribe to avoidance commands
        if (enableAvoidance)
        {
            ros.Subscribe<Float32MultiArrayMsg>(avoidanceCommandTopic, OnAvoidanceCommandReceived);
            Debug.Log("[SiftPathFollower] Obstacle avoidance integration enabled.");
        }

        // Initialize mission tracking
        missionStartTime = Time.time;
        currentState = MissionState.FLYING;
        baseLookAhead = lookAhead; // Store original

        isInitialized = true;
        Debug.Log("[SiftPathFollower] Hazır. Mission started.");
    }

    void OnPoseReceived(Float32MultiArrayMsg msg)
    {
        if (msg.data.Length < 2) return;

        rawSeq = msg.data[0]; 
        visualErrX = msg.data[1];

        // KayÄ±p Durumu (-1)
        if (rawSeq == -1)
        {
            estimatedSeq = -1;
            seqHistory.Clear();
            return;
        }

        // Smoothing (YumuÅŸatma)
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

    void OnAvoidanceCommandReceived(Float32MultiArrayMsg msg)
    {
        if (msg.data.Length < 5) return;

        avoidanceActive = msg.data[0] > 0.5f;
        avoidanceRollAdjustment = msg.data[1];
        avoidanceSpeedMultiplier = msg.data[2];
        avoidanceMode = Mathf.RoundToInt(msg.data[3]);
        avoidanceLookAheadMult = msg.data[4];
    }

    void FixedUpdate()
    {
        if (!isInitialized || drone == null || mapPoints.Count == 0) return;

        // Handle different mission states
        switch (currentState)
        {
            case MissionState.FLYING:
                HandleFlyingState();
                break;

            case MissionState.ARRIVED:
                HandleArrivedState();
                break;

            case MissionState.LANDING:
                HandleLandingState();
                break;

            case MissionState.COMPLETED:
                // Mission complete - do nothing
                break;
        }
    }

    private void HandleFlyingState()
    {
        // --- 1. LOST DURUMU ---
        if (estimatedSeq == -1 || rawSeq == -1)
        {
            // NEW: If avoidance is active (especially RECOVERING), keep executing recovery commands
            // This helps drone return to path where SIFT can reacquire
            if (enableAvoidance && avoidanceActive && avoidanceMode == 2) // RECOVERING mode
            {
                // Continue with recovery commands even when SIFT lost
                // This allows drone to fly back toward path to reacquire SIFT
                if (debugLogs && Time.time - lastLogTime > debugLogPeriod)
                {
                    lastLogTime = Time.time;
                    Debug.LogWarning("[LOST+RECOVERING] SIFT lost but continuing recovery to reacquire path...");
                }
                // Don't return - continue to apply avoidance recovery commands below
            }
            else
            {
                // Not in recovery - safe to stop
                drone.SetExternalCommand(0, 0, 0);

                if (debugLogs && Time.time - lastLogTime > debugLogPeriod)
                {
                    lastLogTime = Time.time;
                    Debug.LogWarning("[LOST] Harita Kayip! Bekleniyor...");
                }
                return;
            }
        }

        float pitchCmd = 0;
        float rollCmd = 0;
        float yawCmd = 0;
        string stateTag = "FLYING";
        string dirStr = "MERKEZ";

        // --- AVOIDANCE INTEGRATION ---
        // Smooth avoidance roll transitions
        if (enableAvoidance && avoidanceActive)
        {
            currentAvoidanceRoll = Mathf.Lerp(currentAvoidanceRoll, avoidanceRollAdjustment,
                                               Time.fixedDeltaTime * avoidanceSmoothness);

            // Update look-ahead dynamically
            lookAhead = baseLookAhead * avoidanceLookAheadMult;

            // Update state tag for logging
            if (avoidanceMode == 1)
                stateTag = "AVOIDING";
            else if (avoidanceMode == 2)
                stateTag = "RECOVERING";
            else if (avoidanceMode == 3)
                stateTag = "EMERGENCY";
        }
        else
        {
            // Smoothly return to zero when avoidance deactivates
            currentAvoidanceRoll = Mathf.Lerp(currentAvoidanceRoll, 0f, Time.fixedDeltaTime * avoidanceSmoothness);
            lookAhead = baseLookAhead; // Reset to base
        }

        // --- 2. HESAPLAMALAR ---

        // NEW: If SIFT lost but in RECOVERING mode, provide basic movement to help reacquire
        bool siftLost = (estimatedSeq == -1 || rawSeq == -1);

        if (siftLost && enableAvoidance && avoidanceActive && avoidanceMode == 2)
        {
            // SIFT lost during recovery - use simple forward motion
            // Roll will be controlled by avoidance commands below
            pitchCmd = forwardSpeed * avoidanceSpeedMultiplier; // Keep moving forward slowly
            rollCmd = 0; // Will be overridden by avoidance blend
            yawCmd = 0; // Maintain current heading

            stateTag = "RECOVERING-BLIND";
            dirStr = "NO-SIFT";
        }
        else if (!siftLost)
        {
            // Normal SIFT-based navigation

            // Deadband & Yön Etiketi
            if (Mathf.Abs(visualErrX) > deadband)
            {
                if (visualErrX > 0)
                {
                    effectiveErrorX = visualErrX - deadband;
                    dirStr = "SAG (>>)";
                }
                else
                {
                    effectiveErrorX = visualErrX + deadband;
                    dirStr = "SOL (<<)";
                }
            }
            else
            {
                effectiveErrorX = 0;
                dirStr = "MERKEZ";
            }

            // Roll Komutu
            rollCmd = effectiveErrorX * visualGainX;
            if (invertRoll) rollCmd = -rollCmd;

            // ARRIVED Kontrolü & State Transition
            if (estimatedSeq >= mapPoints.Count - lookAhead)
            {
                stateTag = "ARRIVED";
                pitchCmd = 0;

                // Transition to ARRIVED state
                currentState = MissionState.ARRIVED;
                Debug.Log("[SiftPathFollower] Mission target reached. Transitioning to ARRIVED state.");
            }
            else
            {
                // Apply speed multiplier from avoidance (if active)
                float baseSpeed = forwardSpeed;
                if (enableAvoidance && avoidanceActive)
                {
                    baseSpeed *= avoidanceSpeedMultiplier;
                }
                pitchCmd = baseSpeed;
            }

            // Yaw Komutu
            Vector3 virtualPos = GetInterpolatedPoint(estimatedSeq);
            Vector3 targetPos = GetInterpolatedPoint(estimatedSeq + lookAhead);
            Vector3 pathVec = targetPos - virtualPos;

            if (pathVec.sqrMagnitude > 0.01f)
            {
                float targetHead = Mathf.Atan2(pathVec.x, pathVec.z) * Mathf.Rad2Deg;
                float currentYaw = transform.eulerAngles.y;
                float yawErr = Mathf.DeltaAngle(currentYaw, targetHead);
                yawCmd = Mathf.Clamp(yawErr * yawGain, -drone.maxYawTorqueCmd, drone.maxYawTorqueCmd);
            }
        }

        // --- BLEND SIFT + AVOIDANCE ROLL ---
        if (enableAvoidance && avoidanceActive)
        {
            // Blend SIFT correction with avoidance command
            float siftRoll = rollCmd;
            float blendedRoll = 0f;

            if (avoidanceMode == 1) // AVOIDING
            {
                // During avoidance, prioritize obstacle avoidance but keep some SIFT correction
                blendedRoll = siftRoll * (1f - avoidanceBlendFactor) + currentAvoidanceRoll * avoidanceBlendFactor;
            }
            else if (avoidanceMode == 2) // RECOVERING
            {
                // During recovery, blend more gently to help SIFT reacquire path
                blendedRoll = siftRoll * 0.7f + currentAvoidanceRoll * 0.3f;
            }
            else if (avoidanceMode == 3) // EMERGENCY STOP
            {
                // Emergency: zero roll, just stop
                blendedRoll = 0f;
                pitchCmd = 0f; // Override pitch too
            }
            else // NORMAL (shouldn't happen if avoidanceActive, but safe fallback)
            {
                blendedRoll = siftRoll;
            }

            rollCmd = blendedRoll;
        }

        // Komutları Uygula
        rollCmd = Mathf.Clamp(rollCmd, -20f, 20f);
        drone.SetExternalCommand(pitchCmd, rollCmd, yawCmd);

        // --- 3. LOGLAMA ---
        if (debugLogs && Time.time - lastLogTime > debugLogPeriod)
        {
            lastLogTime = Time.time;
            float targetSeq = estimatedSeq + lookAhead;

            if (enableAvoidance && avoidanceActive)
            {
                Debug.Log($"[{stateTag}] Seq:{estimatedSeq:F0} > Target:{targetSeq:F0} | {dirStr} Err:{visualErrX:F1} | " +
                          $"AvoidRoll:{currentAvoidanceRoll:F1}° SpeedMult:{avoidanceSpeedMultiplier:F2} | Cmd:(P{pitchCmd:F0}, R{rollCmd:F1})");
            }
            else
            {
                Debug.Log($"[{stateTag}] Seq:{estimatedSeq:F0} > Target:{targetSeq:F0} | {dirStr} Err:{visualErrX:F1} | Cmd:(P{pitchCmd:F0}, R{rollCmd:F1})");
            }
        }
    }

    private void HandleArrivedState()
    {
        // Stop all movement
        drone.SetExternalCommand(0, 0, 0);
        
        // Transition to landing after brief hover
        currentState = MissionState.LANDING;
        Debug.Log("[SiftPathFollower] Starting landing sequence.");
    }

    private void HandleLandingState()
    {
        // Keep drone stable (no pitch/roll/yaw)
        drone.SetExternalCommand(0, 0, 0);

        // Gradually descend by reducing target altitude
        if (drone.targetAltitude > groundAltitude)
        {
            drone.targetAltitude -= descentSpeed * Time.fixedDeltaTime;
            drone.targetAltitude = Mathf.Max(drone.targetAltitude, groundAltitude);

            if (debugLogs && Time.time - lastLogTime > debugLogPeriod)
            {
                lastLogTime = Time.time;
                Debug.Log($"[LANDING] Descending... Altitude: {transform.position.y:F2}m | Target: {drone.targetAltitude:F2}m");
            }
        }
        else
        {
            // Landing complete
            CompleteMission();
        }
    }

    private void CompleteMission()
    {
        currentState = MissionState.COMPLETED;
        
        // Calculate flight duration
        flightDuration = Time.time - missionStartTime;
        
        Debug.Log($"[SiftPathFollower] Mission COMPLETED! Flight duration: {flightDuration:F2} seconds");

        // Show mission complete UI
        if (missionCompleteUI != null)
        {
            missionCompleteUI.ShowWithDuration(flightDuration);
        }
        else
        {
            Debug.LogWarning("[SiftPathFollower] MissionCompleteUI reference is missing!");
        }

        // Disable drone control
        drone.SetExternalCommand(0, 0, 0);
        
        // Optionally disable this component
        // enabled = false;
    }

    Vector3 GetInterpolatedPoint(float seqIndex)
    {
        seqIndex = Mathf.Clamp(seqIndex, 0, mapPoints.Count - 1.001f);
        int i = Mathf.FloorToInt(seqIndex);
        float t = seqIndex - i;
        return Vector3.Lerp(mapPoints[i], mapPoints[i+1], t);
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
        catch {}
    }
}
