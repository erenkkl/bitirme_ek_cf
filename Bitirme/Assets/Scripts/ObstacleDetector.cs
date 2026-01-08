using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using System.Collections;

/// <summary>
/// Obstacle detection from front depth camera for static obstacle avoidance.
/// Analyzes depth image regions and publishes obstacle information.
/// Size-agnostic: Works with trees, buildings, cubes, etc.
/// </summary>
public class ObstacleDetector : MonoBehaviour
{
    [Header("ROS Topics")]
    [Tooltip("Input: Front depth camera topic (32FC1 encoding)")]
    public string depthInputTopic = "/drone/camera/front_depth";

    [Tooltip("Output: Obstacle information for avoidance node")]
    public string obstacleOutputTopic = "/drone/obstacle_info";

    [Header("Detection Parameters")]
    [Tooltip("Maximum detection range for STATIC obstacles in meters")]
    public float maxDetectionRange = 15.0f;

    [Tooltip("Maximum detection range for DYNAMIC obstacles (extended for early trajectory prediction)")]
    public float maxDynamicDetectionRange = 30.0f;

    [Tooltip("Critical range - urgent avoidance needed")]
    public float criticalRange = 8.0f;

    [Header("Dynamic Obstacle Detection")]
    [Tooltip("Velocity threshold to consider obstacle as moving (m/s)")]
    public float dynamicVelocityThreshold = 0.5f;

    [Tooltip("Number of frames to track for velocity estimation")]
    public int velocityHistorySize = 5;

    [Header("Trajectory Prediction (Dynamic Only)")]
    [Tooltip("Time horizon for trajectory prediction (seconds)")]
    public float predictionHorizon = 3.0f;

    [Tooltip("Lateral collision distance threshold (meters) - if predicted path comes this close, trigger avoidance")]
    public float lateralCollisionThreshold = 8.0f;

    [Tooltip("Minimum predicted time-to-collision to trigger early avoidance (seconds)")]
    public float minTimeToCollision = 5.0f;

    [Tooltip("Minimum obstacle occupancy (0-1) to trigger detection")]
    [Range(0.05f, 0.5f)]
    public float minOccupancy = 0.15f; // 15% of zone must be blocked

    [Tooltip("Depth variance threshold - helps reject noise")]
    public float depthVarianceThreshold = 0.5f;

    [Header("Zone Configuration")]
    [Tooltip("Center zone width as fraction of image width")]
    [Range(0.2f, 0.6f)]
    public float centerZoneWidth = 0.4f;

    [Tooltip("Vertical zone limits (0=bottom, 1=top). Ignore ground and sky.")]
    public float verticalMin = 0.3f;
    public float verticalMax = 0.8f;

    [Header("Filtering")]
    [Tooltip("Number of frames to average for stability")]
    public int temporalFilterSize = 3;

    [Tooltip("Processing rate (Hz)")]
    public int processingRate = 20;

    [Header("Debug")]
    public bool debugLogs = false;
    public bool visualizeZones = false;
    public float debugLogPeriod = 1.0f;

    // Internal state
    private ROSConnection ros;
    private float[] depthBuffer;
    private int imgWidth = 640;
    private int imgHeight = 480;

    // Zone boundaries (pixel coordinates)
    private int leftStart, leftEnd;
    private int centerStart, centerEnd;
    private int rightStart, rightEnd;
    private int vMin, vMax;

    // Temporal filtering
    private System.Collections.Generic.Queue<ObstacleData> history;

    // Dynamic obstacle tracking (NEW)
    private System.Collections.Generic.Queue<float> distanceHistory;
    private System.Collections.Generic.Queue<float> timeHistory;
    private float lastProcessTime = 0f;

    // Timing
    private float lastLogTime = 0f;
    private int frameCount = 0;

    // Data structure for obstacle info
    private struct ObstacleData
    {
        public bool hasObstacle;
        public float distance;
        public int preferredSide; // -1=left, 0=none, 1=right
        public float urgency;
        public float clearLeft;
        public float clearRight;
        public float velocity;              // m/s (negative = approaching, positive = receding)
        public bool isDynamic;              // true if obstacle is moving
        public float predictedClosestDist;  // NEW: Predicted closest approach distance (meters)
        public float timeToCollision;       // NEW: Estimated time until closest approach (seconds)
        public bool lateralThreat;          // NEW: True if side-impact predicted
    }

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        // Subscribe to depth camera
        ros.Subscribe<ImageMsg>(depthInputTopic, OnDepthImageReceived);

        // Register publisher
        ros.RegisterPublisher<Float32MultiArrayMsg>(obstacleOutputTopic);

        // Initialize history buffer
        history = new System.Collections.Generic.Queue<ObstacleData>();

        // Initialize velocity tracking (NEW)
        distanceHistory = new System.Collections.Generic.Queue<float>();
        timeHistory = new System.Collections.Generic.Queue<float>();
        lastProcessTime = Time.time;

        // Calculate zone boundaries
        UpdateZoneBoundaries();

        Debug.Log($"[ObstacleDetector] Started. Range: {maxDetectionRange}m, Occupancy: {minOccupancy*100}%");
        if (visualizeZones)
        {
            Debug.Log($"[ObstacleDetector] Zones: Left[{leftStart}-{leftEnd}] Center[{centerStart}-{centerEnd}] Right[{rightStart}-{rightEnd}]");
        }
    }

    void UpdateZoneBoundaries()
    {
        // Horizontal zones
        int centerWidth = Mathf.RoundToInt(imgWidth * centerZoneWidth);
        centerStart = (imgWidth - centerWidth) / 2;
        centerEnd = centerStart + centerWidth;

        leftStart = 0;
        leftEnd = centerStart;

        rightStart = centerEnd;
        rightEnd = imgWidth;

        // Vertical limits (avoid ground and sky)
        vMin = Mathf.RoundToInt(imgHeight * verticalMin);
        vMax = Mathf.RoundToInt(imgHeight * verticalMax);
    }

    void OnDepthImageReceived(ImageMsg msg)
    {
        // Validate message
        if (msg.encoding != "32FC1")
        {
            Debug.LogError($"[ObstacleDetector] Unexpected encoding: {msg.encoding}. Expected 32FC1.");
            return;
        }

        // Update dimensions if changed
        if (msg.width != imgWidth || msg.height != imgHeight)
        {
            imgWidth = (int)msg.width;
            imgHeight = (int)msg.height;
            UpdateZoneBoundaries();
            Debug.Log($"[ObstacleDetector] Resolution updated: {imgWidth}x{imgHeight}");
        }

        // Convert byte array to float array
        int pixelCount = imgWidth * imgHeight;
        if (depthBuffer == null || depthBuffer.Length != pixelCount)
        {
            depthBuffer = new float[pixelCount];
        }

        System.Buffer.BlockCopy(msg.data, 0, depthBuffer, 0, msg.data.Length);

        // Process depth data
        ProcessDepthData();
    }

    void ProcessDepthData()
    {
        frameCount++;

        // Analyze each zone
        ZoneAnalysis leftZone = AnalyzeZone(leftStart, leftEnd, "LEFT");
        ZoneAnalysis centerZone = AnalyzeZone(centerStart, centerEnd, "CENTER");
        ZoneAnalysis rightZone = AnalyzeZone(rightStart, rightEnd, "RIGHT");

        // Decision logic
        ObstacleData current = MakeDecision(leftZone, centerZone, rightZone);

        // Temporal filtering
        history.Enqueue(current);
        if (history.Count > temporalFilterSize)
            history.Dequeue();

        // Average filtered result
        ObstacleData filtered = AverageHistory();

        // Publish
        PublishObstacleInfo(filtered);

        // Debug logging
        if (debugLogs && Time.time - lastLogTime > debugLogPeriod)
        {
            lastLogTime = Time.time;
            if (filtered.hasObstacle)
            {
                string sideStr = filtered.preferredSide == -1 ? "LEFT" : (filtered.preferredSide == 1 ? "RIGHT" : "NONE");
                string typeStr = filtered.isDynamic ? "DYNAMIC" : "STATIC";
                string velStr = filtered.isDynamic ? $"Vel:{filtered.velocity:F1}m/s" : "";
                string lateralStr = filtered.lateralThreat ? $"⚠️LATERAL_THREAT TTC:{filtered.timeToCollision:F1}s" : "";
                Debug.Log($"[ObstacleDetector] {typeStr} OBSTACLE @ {filtered.distance:F1}m {velStr} {lateralStr} | Avoid: {sideStr} | Urgency: {filtered.urgency:F2} | ClearL:{filtered.clearLeft:F2} ClearR:{filtered.clearRight:F2}");
            }
            else if (frameCount % 100 == 0) // Occasional "all clear" log
            {
                Debug.Log($"[ObstacleDetector] All clear. L:{filtered.clearLeft:F2} C:OK R:{filtered.clearRight:F2}");
            }
        }
    }

    struct ZoneAnalysis
    {
        public float minDepth;
        public float medianDepth;
        public float occupancy;      // Fraction of pixels < threshold
        public float avgDepthBlocked; // Average depth of blocked pixels
        public bool isBlocked;
    }

    ZoneAnalysis AnalyzeZone(int xStart, int xEnd, string zoneName)
    {
        var result = new ZoneAnalysis();
        var depths = new System.Collections.Generic.List<float>();

        int totalPixels = 0;
        int blockedPixels = 0;
        float sumBlockedDepth = 0f;
        float minDepth = float.MaxValue;

        // Scan zone pixels (within vertical limits)
        for (int y = vMin; y < vMax; y++)
        {
            for (int x = xStart; x < xEnd; x++)
            {
                int idx = y * imgWidth + x;
                if (idx >= depthBuffer.Length) continue;

                float depth = depthBuffer[idx];

                // Ignore invalid depths
                if (depth <= 0.1f || depth > 100f || float.IsNaN(depth) || float.IsInfinity(depth))
                    continue;

                depths.Add(depth);
                totalPixels++;

                if (depth < minDepth)
                    minDepth = depth;

                // Check if blocked (use extended range for any obstacle detection first)
                // We'll filter by static/dynamic later
                if (depth < maxDynamicDetectionRange)
                {
                    blockedPixels++;
                    sumBlockedDepth += depth;
                }
            }
        }

        // Calculate statistics
        if (totalPixels > 0)
        {
            result.occupancy = (float)blockedPixels / totalPixels;
            result.minDepth = minDepth;

            if (blockedPixels > 0)
            {
                result.avgDepthBlocked = sumBlockedDepth / blockedPixels;
            }
            else
            {
                result.avgDepthBlocked = maxDetectionRange;
            }

            // Median depth
            if (depths.Count > 0)
            {
                depths.Sort();
                result.medianDepth = depths[depths.Count / 2];
            }

            // Is zone blocked?
            result.isBlocked = result.occupancy > minOccupancy;
        }
        else
        {
            // No valid data - assume clear
            result.minDepth = maxDetectionRange;
            result.medianDepth = maxDetectionRange;
            result.occupancy = 0f;
            result.avgDepthBlocked = maxDetectionRange;
            result.isBlocked = false;
        }

        if (visualizeZones && debugLogs && frameCount % 60 == 0)
        {
            Debug.Log($"[{zoneName}] Min:{result.minDepth:F1}m Med:{result.medianDepth:F1}m Occ:{result.occupancy*100:F0}% Blocked:{result.isBlocked}");
        }

        return result;
    }

    ObstacleData MakeDecision(ZoneAnalysis left, ZoneAnalysis center, ZoneAnalysis right)
    {
        var result = new ObstacleData();

        // Clearance scores (inverted occupancy)
        result.clearLeft = 1.0f - left.occupancy;
        result.clearRight = 1.0f - right.occupancy;

        // First, calculate velocity and dynamic detection for ANY detected obstacle
        // We use the minimum depth from all zones for initial velocity tracking
        float minDetectedDepth = Mathf.Min(left.minDepth, Mathf.Min(center.minDepth, right.minDepth));
        CalculateVelocity(minDetectedDepth, out result.velocity, out result.isDynamic);

        // NEW: Predict trajectory for dynamic obstacles (includes lateral threats)
        PredictTrajectory(left, center, right, result.velocity, result.isDynamic,
                         out result.predictedClosestDist, out result.timeToCollision, out result.lateralThreat);

        // Determine effective detection range based on obstacle type
        float effectiveRange = result.isDynamic ? maxDynamicDetectionRange : maxDetectionRange;

        // Check if center zone has obstacle (within range for obstacle type)
        bool centerBlocked = center.isBlocked && center.avgDepthBlocked < effectiveRange;

        // NEW: Also check for lateral threats that aren't yet in center
        bool lateralThreatDetected = result.lateralThreat && result.timeToCollision < minTimeToCollision;

        if (centerBlocked || lateralThreatDetected)
        {
            result.hasObstacle = true;

            // Use center distance if blocked, otherwise use lateral obstacle distance
            if (centerBlocked)
            {
                result.distance = center.avgDepthBlocked;
            }
            else
            {
                // Lateral threat - use the side's distance
                bool threatFromLeft = (left.occupancy > right.occupancy);
                result.distance = threatFromLeft ? left.avgDepthBlocked : right.avgDepthBlocked;
            }

            // Choose clearer side (with extra consideration for lateral threats)
            if (result.lateralThreat)
            {
                // If lateral threat from left, strongly prefer going right (and vice versa)
                bool threatFromLeft = (left.occupancy > right.occupancy);
                result.preferredSide = threatFromLeft ? 1 : -1; // Go opposite direction
            }
            else if (result.clearLeft > result.clearRight + 0.1f) // 10% hysteresis
            {
                result.preferredSide = -1; // LEFT
            }
            else if (result.clearRight > result.clearLeft + 0.1f)
            {
                result.preferredSide = 1; // RIGHT
            }
            else
            {
                // Equal clearance - choose based on which side is less blocked
                result.preferredSide = left.minDepth > right.minDepth ? -1 : 1;
            }

            // Calculate urgency based on distance and occupancy
            float distanceUrgency = 1.0f - Mathf.Clamp01((result.distance - criticalRange) / (effectiveRange - criticalRange));
            float occupancyUrgency = center.occupancy;
            result.urgency = Mathf.Max(distanceUrgency, occupancyUrgency);

            // NEW: Boost urgency if obstacle is approaching (dynamic)
            if (result.isDynamic && result.velocity < -0.5f) // Approaching faster than 0.5 m/s
            {
                float velocityUrgency = Mathf.Clamp01(-result.velocity / 5.0f); // Normalize by max expected speed
                result.urgency = Mathf.Max(result.urgency, velocityUrgency);
            }

            // NEW: Extra urgency boost for lateral threats
            if (result.lateralThreat)
            {
                float lateralUrgency = 1.0f - Mathf.Clamp01(result.timeToCollision / minTimeToCollision);
                result.urgency = Mathf.Max(result.urgency, lateralUrgency * 1.2f); // 20% boost for lateral threats
            }
        }
        else
        {
            // No obstacle in center and no lateral threat
            result.hasObstacle = false;
            result.distance = effectiveRange;
            result.preferredSide = 0;
            result.urgency = 0f;
            result.predictedClosestDist = 100f;
            result.timeToCollision = 100f;
            result.lateralThreat = false;
        }

        return result;
    }

    void CalculateVelocity(float currentDistance, out float velocity, out bool isDynamic)
    {
        // Add current measurement to history
        float currentTime = Time.time;
        distanceHistory.Enqueue(currentDistance);
        timeHistory.Enqueue(currentTime);

        // Maintain history size
        while (distanceHistory.Count > velocityHistorySize)
        {
            distanceHistory.Dequeue();
            timeHistory.Dequeue();
        }

        // Need at least 2 samples to calculate velocity
        if (distanceHistory.Count < 2)
        {
            velocity = 0f;
            isDynamic = false;
            return;
        }

        // Calculate velocity using linear regression over history
        float[] distances = distanceHistory.ToArray();
        float[] times = timeHistory.ToArray();

        // Simple velocity: (last_distance - first_distance) / (last_time - first_time)
        float deltaDistance = distances[distances.Length - 1] - distances[0];
        float deltaTime = times[times.Length - 1] - times[0];

        if (deltaTime > 0.001f) // Avoid division by zero
        {
            velocity = deltaDistance / deltaTime;

            // Mark as dynamic if sustained velocity above threshold
            isDynamic = Mathf.Abs(velocity) > dynamicVelocityThreshold;
        }
        else
        {
            velocity = 0f;
            isDynamic = false;
        }
    }

    /// <summary>
    /// NEW: Predict trajectory for dynamic obstacles to detect lateral (side) collisions
    /// Uses simple linear prediction: assume obstacle continues at current velocity
    /// </summary>
    void PredictTrajectory(ZoneAnalysis left, ZoneAnalysis center, ZoneAnalysis right,
                          float velocity, bool isDynamic,
                          out float predictedClosestDist, out float timeToCollision, out bool lateralThreat)
    {
        // Default values (no threat)
        predictedClosestDist = 100f;
        timeToCollision = 100f;
        lateralThreat = false;

        // Only predict for dynamic obstacles
        if (!isDynamic || Mathf.Abs(velocity) < dynamicVelocityThreshold)
            return;

        // Analyze lateral distribution to estimate obstacle's lateral position
        // If obstacle is more on left/right, it might cross our path

        float leftOccupancy = left.occupancy;
        float centerOccupancy = center.occupancy;
        float rightOccupancy = right.occupancy;

        // If obstacle is primarily in left or right zone (not center), check for lateral crossing
        bool obstacleOnLeft = (leftOccupancy > centerOccupancy) && (leftOccupancy > minOccupancy);
        bool obstacleOnRight = (rightOccupancy > centerOccupancy) && (rightOccupancy > minOccupancy);

        if (obstacleOnLeft || obstacleOnRight)
        {
            // Obstacle is lateral - estimate if it will cross into center
            float obstacleDistance = obstacleOnLeft ? left.avgDepthBlocked : right.avgDepthBlocked;

            // Simplified lateral threat assessment:
            // If obstacle is approaching (velocity < 0) and currently on the side,
            // it might cross our path as it gets closer

            if (velocity < -dynamicVelocityThreshold) // Approaching
            {
                // Estimate time until obstacle reaches our lateral position
                // Assume obstacle maintains current velocity
                timeToCollision = -obstacleDistance / velocity; // Negative velocity, so this is positive time

                // Predict closest distance during approach
                // For simplicity, assume obstacle will pass through center zone if it's moving toward us
                // In reality, we'd need lateral velocity, but we approximate from occupancy changes

                if (timeToCollision < minTimeToCollision && timeToCollision > 0)
                {
                    // Obstacle will be close soon
                    lateralThreat = true;
                    predictedClosestDist = lateralCollisionThreshold * 0.5f; // Conservative estimate

                    if (debugLogs)
                    {
                        string side = obstacleOnLeft ? "LEFT" : "RIGHT";
                        Debug.Log($"[ObstacleDetector] LATERAL THREAT from {side}! Dist:{obstacleDistance:F1}m, Vel:{velocity:F1}m/s, TTC:{timeToCollision:F1}s");
                    }
                }
            }
        }

        // Also check if center obstacle is approaching rapidly (head-on collision)
        if (center.isBlocked && velocity < -dynamicVelocityThreshold)
        {
            float obstacleDistance = center.avgDepthBlocked;
            timeToCollision = -obstacleDistance / velocity;
            predictedClosestDist = 0f; // Head-on collision predicted
            lateralThreat = false; // Not lateral, but frontal
        }
    }

    ObstacleData AverageHistory()
    {
        if (history.Count == 0)
        {
            return new ObstacleData
            {
                hasObstacle = false,
                distance = maxDetectionRange,
                preferredSide = 0,
                urgency = 0f,
                clearLeft = 1.0f,
                clearRight = 1.0f,
                velocity = 0f,
                isDynamic = false,
                predictedClosestDist = 100f,
                timeToCollision = 100f,
                lateralThreat = false
            };
        }

        // Voting for hasObstacle
        int obstacleVotes = 0;
        int dynamicVotes = 0;
        int lateralThreatVotes = 0;
        float sumDistance = 0f;
        int sumSide = 0;
        float sumUrgency = 0f;
        float sumClearLeft = 0f;
        float sumClearRight = 0f;
        float sumVelocity = 0f;
        float sumPredictedClosestDist = 0f;
        float sumTimeToCollision = 0f;

        foreach (var data in history)
        {
            if (data.hasObstacle) obstacleVotes++;
            if (data.isDynamic) dynamicVotes++;
            if (data.lateralThreat) lateralThreatVotes++;
            sumDistance += data.distance;
            sumSide += data.preferredSide;
            sumUrgency += data.urgency;
            sumClearLeft += data.clearLeft;
            sumClearRight += data.clearRight;
            sumVelocity += data.velocity;
            sumPredictedClosestDist += data.predictedClosestDist;
            sumTimeToCollision += data.timeToCollision;
        }

        int count = history.Count;
        bool finalObstacle = obstacleVotes > count / 2; // Majority vote
        bool finalDynamic = dynamicVotes > count / 2; // Majority vote for dynamic
        bool finalLateralThreat = lateralThreatVotes > count / 2; // Majority vote for lateral threat

        return new ObstacleData
        {
            hasObstacle = finalObstacle,
            distance = sumDistance / count,
            preferredSide = sumSide > 0 ? 1 : (sumSide < 0 ? -1 : 0),
            urgency = sumUrgency / count,
            clearLeft = sumClearLeft / count,
            clearRight = sumClearRight / count,
            velocity = sumVelocity / count,
            isDynamic = finalDynamic,
            predictedClosestDist = sumPredictedClosestDist / count,
            timeToCollision = sumTimeToCollision / count,
            lateralThreat = finalLateralThreat
        };
    }

    void PublishObstacleInfo(ObstacleData data)
    {
        var msg = new Float32MultiArrayMsg
        {
            data = new float[]
            {
                data.hasObstacle ? 1.0f : 0.0f,     // Index 0
                data.distance,                       // Index 1
                (float)data.preferredSide,          // Index 2
                data.urgency,                        // Index 3
                data.clearLeft,                      // Index 4
                data.clearRight,                     // Index 5
                data.velocity,                       // Index 6
                data.isDynamic ? 1.0f : 0.0f,       // Index 7
                data.predictedClosestDist,          // Index 8: NEW
                data.timeToCollision,               // Index 9: NEW
                data.lateralThreat ? 1.0f : 0.0f   // Index 10: NEW
            }
        };

        ros.Publish(obstacleOutputTopic, msg);
    }

    void OnDrawGizmos()
    {
        if (!visualizeZones || !Application.isPlaying)
            return;

        // Draw detection range sphere
        Gizmos.color = Color.yellow;
        Gizmos.DrawWireSphere(transform.position + transform.forward * maxDetectionRange, 2f);

        // Draw critical range sphere
        Gizmos.color = Color.red;
        Gizmos.DrawWireSphere(transform.position + transform.forward * criticalRange, 1.5f);
    }
}
