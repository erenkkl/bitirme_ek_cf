# Dynamic Obstacle Avoidance - Implementation Guide

**Date:** 2026-01-05
**Feature:** Detection and avoidance of moving obstacles (drones)
**Approach:** Velocity estimation + enhanced urgency + recovery monitoring

---

## 🎯 **Overview**

This system extends static obstacle avoidance to handle **dynamic obstacles** (other drones moving in the environment). It detects obstacle velocity, boosts avoidance for approaching obstacles, and monitors recovery to re-avoid if obstacles move back into path.

---

## 🚁 **Supported Scenarios**

### **Your Use Case:**
- 2-3 drones moving on zigzag paths
- Constant altitude (30m)
- Speed: 3-4 m/s
- Spread out (won't encounter multiple simultaneously)

### **Detection Capabilities:**
- ✅ Approaching obstacles (negative velocity)
- ✅ Receding obstacles (positive velocity)
- ✅ Fast-approaching obstacles (> 2.5 m/s closing speed)
- ✅ Dynamic obstacles that turn around during recovery

---

## 📊 **How It Works**

### **1. Velocity Estimation** (ObstacleDetector.cs)

**Method:**
```csharp
// Track obstacle distance over last 5 frames (~0.25 seconds)
velocity = (current_distance - old_distance) / time_elapsed

// Negative velocity = approaching (distance decreasing)
// Positive velocity = receding (distance increasing)
```

**Classification:**
```csharp
isDynamic = abs(velocity) > 0.5 m/s  // Sustained movement
```

---

### **2. Enhanced Urgency** (Unity + ROS2)

**Unity Side (ObstacleDetector.cs):**
```csharp
if (isDynamic && velocity < -0.5 m/s) {  // Approaching
    velocityUrgency = abs(velocity) / 5.0  // Normalize
    urgency = max(urgency, velocityUrgency)
}
```

**ROS2 Side (obstacle_avoidance_node.py):**
```python
if is_dynamic and velocity < -1.0:  # Approaching
    roll_gain *= 1.3  # 30% boost

if velocity < -2.5:  # Fast approaching
    roll_gain *= 1.2  # Additional 20% boost
    speed *= 0.8     # Slow down to 48%
```

---

### **3. Recovery Monitoring**

**Problem:** Dynamic obstacle may turn around and approach again during recovery

**Solution:**
```python
if in RECOVERING:
    if obstacle_is_dynamic and velocity < -1.0 and distance < 22m:
        return to AVOIDING  # Re-avoid!
```

---

## 🔧 **Configuration Parameters**

### **Unity (ObstacleDetector.cs)**

| Parameter | Value | Description |
|-----------|-------|-------------|
| `maxDetectionRange` | 20.0m | Increased from 15m for dynamic obstacles |
| `dynamicVelocityThreshold` | 0.5 m/s | Min velocity to mark as "dynamic" |
| `velocityHistorySize` | 5 frames | Samples for velocity calculation |

### **ROS2 (obstacle_avoidance_node.py)**

| Parameter | Value | Description |
|-----------|-------|-------------|
| `DYNAMIC_URGENCY_BOOST` | 1.3 | Urgency multiplier for approaching obstacles |
| `DYNAMIC_ROLL_BOOST` | 1.2 | Roll gain boost for fast approach |
| `APPROACHING_VELOCITY_THRESHOLD` | -1.0 m/s | Velocity to consider "approaching" |
| `FAST_APPROACHING_THRESHOLD` | -2.5 m/s | Very fast approach threshold |

---

## 📡 **Message Format**

### **Updated `/drone/obstacle_info` (Float32MultiArray)**

| Index | Field | Type | Range | Description |
|-------|-------|------|-------|-------------|
| 0 | has_obstacle | bool | 0.0 or 1.0 | Obstacle detected |
| 1 | distance | float | 0-50 m | Distance to obstacle |
| 2 | preferred_side | int | -1, 0, 1 | Avoidance direction |
| 3 | urgency | float | 0.0-1.0 | Urgency level |
| 4 | clear_left | float | 0.0-1.0 | Left zone clearance |
| 5 | clear_right | float | 0.0-1.0 | Right zone clearance |
| 6 | **velocity** | **float** | **-10 to 10 m/s** | **Obstacle velocity (NEW)** |
| 7 | **is_dynamic** | **bool** | **0.0 or 1.0** | **Dynamic flag (NEW)** |

**Velocity Interpretation:**
- **Negative:** Obstacle approaching (distance decreasing)
- **Positive:** Obstacle receding (distance increasing)
- **Zero:** Static obstacle or no sustained movement

---

## 🎮 **Behavior Examples**

### **Scenario 1: Static Obstacle (Building)**
```
t=0s: Detect @ 18m, velocity = 0 m/s
      → Static obstacle
      → Normal avoidance: 12° roll, 60% speed

t=2s: Distance 22m+
      → Enter RECOVERY
      → Return to path
```

### **Scenario 2: Slow-Moving Drone (Receding)**
```
t=0s: Detect @ 18m, velocity = +2 m/s (moving away)
      → Dynamic but receding
      → Normal avoidance: 12° roll, 60% speed
      → Low urgency

t=3s: Distance 24m
      → Easily avoided
```

### **Scenario 3: Approaching Drone (Head-On)**
```
t=0s: Detect @ 18m, velocity = -3 m/s (approaching!)
      → Dynamic + approaching
      → Urgency boosted by 1.3x
      → Roll: 12° × 1.3 × 1.2 = 18.7° (boosted!)
      → Speed: 60% × 0.8 = 48% (extra slow)
      → Log: "[AVOIDING] FAST approaching obstacle! Vel:-3.0m/s"

t=2s: Distance 12m, velocity still -3 m/s
      → Continue aggressive avoidance

t=4s: Passed obstacle, distance 25m
      → Enter RECOVERY
```

### **Scenario 4: Drone Turns Around During Recovery**
```
t=0s: Avoided drone, entered RECOVERY

t=1s: Drone turns around, velocity = -3 m/s (coming back!)
      → Detect: dynamic + approaching + distance < 22m
      → Exit RECOVERY → Back to AVOIDING
      → Log: "[RECOVERY] Dynamic obstacle approaching again! Vel:-3.0m/s"

t=3s: Re-avoid with boosted roll
      → Success
```

---

## 🧪 **Testing Setup**

### **Create Moving Obstacle in Unity**

1. **Duplicate your drone prefab:**
   - Hierarchy → Right-click your drone → Duplicate
   - Rename to "ObstacleDrone1"

2. **Add simple movement script:**

```csharp
// SimpleDroneMovement.cs
using UnityEngine;

public class SimpleDroneMovement : MonoBehaviour
{
    public float speed = 3.0f;
    public float zigzagDistance = 20f;
    private float startZ;
    private int direction = 1;

    void Start()
    {
        startZ = transform.position.z;
    }

    void Update()
    {
        // Move back and forth in Z direction
        transform.position += Vector3.forward * direction * speed * Time.deltaTime;

        // Reverse at zigzag points
        if (transform.position.z > startZ + zigzagDistance)
            direction = -1;
        else if (transform.position.z < startZ - zigzagDistance)
            direction = 1;
    }
}
```

3. **Attach script to ObstacleDrone1**
4. **Position in your path:**
   - Place 30-40m ahead
   - Same altitude (30m)
   - Set speed = 3.5 m/s

5. **Add 2-3 more obstacle drones** spread along path

---

## 📈 **Expected Behavior**

### **Detection Range:**
- **Before:** 15m
- **After:** 20m (better for dynamic obstacles)

### **Avoidance Roll:**
| Scenario | Base Roll | Boosted Roll | Multipliers |
|----------|-----------|--------------|-------------|
| Static obstacle | 12° | 12° | 1.0x |
| Dynamic receding | 12° | 12° | 1.0x |
| Dynamic approaching (slow) | 12° | 15.6° | 1.3x |
| Dynamic approaching (fast) | 12° | 18.7° | 1.3x × 1.2x |

### **Speed Reduction:**
| Scenario | Speed |
|----------|-------|
| Static | 60% |
| Dynamic slow | 60% |
| Dynamic fast | 48% |

---

## 🔍 **Console Logs**

### **Unity Console:**

**Static Obstacle:**
```
[ObstacleDetector] STATIC OBSTACLE @ 12.3m | Avoid: RIGHT | Urgency: 0.65
```

**Dynamic Obstacle (Approaching):**
```
[ObstacleDetector] DYNAMIC OBSTACLE @ 15.2m Vel:-3.2m/s | Avoid: LEFT | Urgency: 0.82
```

**Dynamic Obstacle (Receding):**
```
[ObstacleDetector] DYNAMIC OBSTACLE @ 18.5m Vel:+2.1m/s | Avoid: RIGHT | Urgency: 0.45
```

---

### **ROS2 Terminal:**

**Normal Avoidance:**
```
[AVOIDING] STATIC  | Avoiding RIGHT | Dist: 12.3m | Urg: 0.65
```

**Dynamic Approaching:**
```
[AVOIDING] Dynamic obstacle approaching at -3.2m/s - boosting avoidance!
[AVOIDING] DYNAMIC Vel:-3.2m/s | Avoiding LEFT | Dist: 15.2m | Urg: 0.82
```

**Fast Approaching:**
```
[AVOIDING] FAST approaching obstacle! Vel:-3.5m/s - maximum avoidance!
[AVOIDING] DYNAMIC Vel:-3.5m/s | Avoiding RIGHT | Dist: 12.1m | Urg: 0.95
```

**Recovery Re-Avoid:**
```
[RECOVERING] Recovery progress | SIFT Conf: 0.72
[RECOVERY] Dynamic obstacle approaching again! Vel:-3.0m/s, returning to AVOIDING
[AVOIDING] DYNAMIC Vel:-3.0m/s | Avoiding LEFT | Dist: 18.5m
```

---

## ⚙️ **Tuning Guide**

### **If Avoidance Too Aggressive:**
```python
DYNAMIC_URGENCY_BOOST = 1.2       # Reduce from 1.3
DYNAMIC_ROLL_BOOST = 1.1          # Reduce from 1.2
```

### **If Not Aggressive Enough:**
```python
APPROACHING_VELOCITY_THRESHOLD = -0.5  # Trigger earlier
FAST_APPROACHING_THRESHOLD = -2.0      # Lower threshold
DYNAMIC_URGENCY_BOOST = 1.5            # Increase boost
```

### **If Too Many False Positives (Static Marked as Dynamic):**
```csharp
// In Unity ObstacleDetector
public float dynamicVelocityThreshold = 0.8f;  // Increase from 0.5
public int velocityHistorySize = 7;            // More samples (smoother)
```

### **If Missing Slow-Moving Obstacles:**
```csharp
public float dynamicVelocityThreshold = 0.3f;  // Decrease threshold
```

---

## 🚀 **How to Test**

### **1. Launch System:**

```bash
# Terminal 1: ROS endpoint
ros2 run ros_tcp_endpoint default_server_endpoint

# Terminal 2: SIFT localizer
ros2 run sift_localizer sift_localizer_node.py

# Terminal 3: Obstacle avoidance (UPDATED)
python3 /home/eren/bitirme_repo/ros2_ws/src/obstacle_avoidance_node.py

# Terminal 4: Unity
# Press Play
```

### **2. Monitor Detection:**

**Unity Console - Should see:**
```
[ObstacleDetector] Started. Range: 20.0m  ← Increased!
[ObstacleDetector] DYNAMIC OBSTACLE @ 18.2m Vel:-3.1m/s  ← Velocity shown!
```

**ROS2 Terminal - Should see:**
```
[AVOIDING] DYNAMIC Vel:-3.1m/s | Avoiding RIGHT  ← Type shown!
[AVOIDING] Dynamic obstacle approaching at -3.1m/s - boosting avoidance!  ← Boost applied!
```

### **3. Test Scenarios:**

**Test A: Static Obstacle**
- Place cube in path
- Should see: "STATIC OBSTACLE"
- Normal avoidance

**Test B: Slow-Moving Drone**
- Set obstacle drone speed = 2 m/s
- Should see: "DYNAMIC" but normal avoidance

**Test C: Fast-Approaching Drone**
- Set obstacle drone speed = 4 m/s
- Drone moving toward you
- Should see: "FAST approaching" + boosted avoidance

**Test D: Drone Turns During Recovery**
- Avoid drone
- Let it turn around
- Should see: Re-enter AVOIDING mode

---

## 🎯 **Success Criteria**

✅ **Dynamic obstacles detected** - Unity log shows "DYNAMIC"
✅ **Velocity calculated** - Vel: X.Xm/s shown in logs
✅ **Urgency boosted** - Higher urgency for approaching obstacles
✅ **Roll boosted** - Stronger avoidance for fast approach
✅ **Recovery monitoring** - Re-avoids if obstacle approaches again
✅ **Mission completes** - Drone successfully avoids 2-3 moving obstacles

---

## 📝 **Files Modified**

### **Unity:**
- ✅ **ObstacleDetector.cs**
  - Added velocity tracking (5-frame history)
  - Added dynamic detection logic
  - Increased detection range to 20m
  - Updated message format (8 fields now)

### **ROS2:**
- ✅ **obstacle_avoidance_node.py**
  - Added dynamic obstacle parameters
  - Added velocity-based urgency boost
  - Added recovery re-avoid logic
  - Enhanced logging for dynamic obstacles

### **No changes needed:**
- ✅ SiftPathFollower.cs
- ✅ DronePhysics.cs

---

## 🔄 **Upgrade Path**

**Current Implementation: Option 1 (Simple + Velocity)**
- ✅ Velocity estimation
- ✅ Dynamic detection
- ✅ Enhanced urgency
- ✅ Recovery monitoring

**Future Enhancement: Option 2 (Prediction)**
If needed later:
- Add collision prediction (time-to-collision)
- Add lateral velocity tracking
- Pre-emptive avoidance based on predicted trajectory

**Future Enhancement: Option 3 (Communication)**
If multi-drone system:
- Drones publish their positions
- Subscribe to other drones' topics
- Cooperative avoidance maneuvers

---

## ⚠️ **Limitations**

1. **Depth camera only** - No 360° awareness
2. **Single obstacle** - Tracks closest only
3. **No trajectory prediction** - Reacts to current velocity
4. **Assumes constant velocity** - May not anticipate turns
5. **Vision-dependent** - Requires visual detection

**For your use case (2-3 spread drones), these limitations are acceptable!**

---

## 📞 **Troubleshooting**

### **Issue: All obstacles marked as dynamic**
**Cause:** Depth camera noise
**Fix:** Increase `dynamicVelocityThreshold` to 0.8 m/s

### **Issue: Dynamic obstacles not detected**
**Cause:** Velocity too low or insufficient history
**Fix:**
- Lower `dynamicVelocityThreshold` to 0.3 m/s
- Increase `velocityHistorySize` to 7

### **Issue: Avoidance not aggressive enough for approaching drones**
**Cause:** Boost multipliers too low
**Fix:** Increase `DYNAMIC_URGENCY_BOOST` to 1.5 and `DYNAMIC_ROLL_BOOST` to 1.3

### **Issue: Recovery fails for dynamic obstacles**
**Cause:** Re-avoid threshold too strict
**Fix:** Change `APPROACHING_VELOCITY_THRESHOLD` from -1.0 to -0.5

---

**Status:** ✅ Ready for testing with moving drones
**Priority:** HIGH - Enables multi-drone scenarios
**Complexity:** MEDIUM - Builds on existing static avoidance
