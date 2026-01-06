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
    [Tooltip("Maximum detection range in meters")]
    public float maxDetectionRange = 15.0f;

    [Tooltip("Critical range - urgent avoidance needed")]
    public float criticalRange = 8.0f;

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
                Debug.Log($"[ObstacleDetector] OBSTACLE @ {filtered.distance:F1}m | Avoid: {sideStr} | Urgency: {filtered.urgency:F2} | ClearL:{filtered.clearLeft:F2} ClearR:{filtered.clearRight:F2}");
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

                // Check if blocked
                if (depth < maxDetectionRange)
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

        // Check if center zone has obstacle
        if (center.isBlocked)
        {
            result.hasObstacle = true;
            result.distance = center.avgDepthBlocked;

            // Choose clearer side
            if (result.clearLeft > result.clearRight + 0.1f) // 10% hysteresis
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
            float distanceUrgency = 1.0f - Mathf.Clamp01((result.distance - criticalRange) / (maxDetectionRange - criticalRange));
            float occupancyUrgency = center.occupancy;
            result.urgency = Mathf.Max(distanceUrgency, occupancyUrgency);
        }
        else
        {
            // No obstacle in center
            result.hasObstacle = false;
            result.distance = maxDetectionRange;
            result.preferredSide = 0;
            result.urgency = 0f;
        }

        return result;
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
                clearRight = 1.0f
            };
        }

        // Voting for hasObstacle
        int obstacleVotes = 0;
        float sumDistance = 0f;
        int sumSide = 0;
        float sumUrgency = 0f;
        float sumClearLeft = 0f;
        float sumClearRight = 0f;

        foreach (var data in history)
        {
            if (data.hasObstacle) obstacleVotes++;
            sumDistance += data.distance;
            sumSide += data.preferredSide;
            sumUrgency += data.urgency;
            sumClearLeft += data.clearLeft;
            sumClearRight += data.clearRight;
        }

        int count = history.Count;
        bool finalObstacle = obstacleVotes > count / 2; // Majority vote

        return new ObstacleData
        {
            hasObstacle = finalObstacle,
            distance = sumDistance / count,
            preferredSide = sumSide > 0 ? 1 : (sumSide < 0 ? -1 : 0),
            urgency = sumUrgency / count,
            clearLeft = sumClearLeft / count,
            clearRight = sumClearRight / count
        };
    }

    void PublishObstacleInfo(ObstacleData data)
    {
        var msg = new Float32MultiArrayMsg
        {
            data = new float[]
            {
                data.hasObstacle ? 1.0f : 0.0f,
                data.distance,
                (float)data.preferredSide,
                data.urgency,
                data.clearLeft,
                data.clearRight
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
