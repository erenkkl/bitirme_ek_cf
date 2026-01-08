# Dynamic Obstacle Avoidance - Quick Start

**What's New:** Your drone can now detect and avoid **moving obstacles** (other drones)!

---

## 🚀 **Quick Test (5 Minutes)**

### **1. No Changes Needed in Unity!**

Just restart Unity - the updated `ObstacleDetector.cs` is ready.

### **2. Restart Python Node:**

```bash
cd /home/eren/bitirme_repo/ros2_ws/src
python3 obstacle_avoidance_node.py
```

### **3. Create Moving Obstacle:**

**Option A: Simple Moving Cube**
```csharp
// In Unity: Create > 3D Object > Cube
// Add this script to cube:

using UnityEngine;

public class MovingObstacle : MonoBehaviour
{
    void Update()
    {
        // Move back and forth at 3 m/s
        transform.position = new Vector3(
            0,
            30,
            Mathf.PingPong(Time.time * 3f, 40f)
        );
    }
}
```

**Option B: Use Existing Drone**
- Duplicate your drone
- Make it move in a simple pattern
- Place in your path

---

## 📊 **What to Look For**

### **Unity Console:**

**Before (Static):**
```
[ObstacleDetector] STATIC OBSTACLE @ 12.3m
```

**After (Dynamic):**
```
[ObstacleDetector] DYNAMIC OBSTACLE @ 15.2m Vel:-3.2m/s
```

### **ROS2 Terminal:**

**Approaching Obstacle:**
```
[AVOIDING] Dynamic obstacle approaching at -3.2m/s - boosting avoidance!
[AVOIDING] DYNAMIC Vel:-3.2m/s | Avoiding RIGHT
```

**Fast Approaching:**
```
[AVOIDING] FAST approaching obstacle! Vel:-3.5m/s - maximum avoidance!
```

**Recovery Re-Avoid:**
```
[RECOVERY] Dynamic obstacle approaching again! Vel:-3.0m/s, returning to AVOIDING
```

---

## ✅ **Key Features**

### **1. Velocity Detection**
- Negative velocity = approaching (distance decreasing)
- Positive velocity = receding (distance increasing)
- Zero = static obstacle

### **2. Boosted Avoidance**
- **Approaching (> 1 m/s):** 30% more aggressive
- **Fast approaching (> 2.5 m/s):** 56% more aggressive + slow to 48%

### **3. Recovery Monitoring**
- If obstacle turns around during recovery → re-avoid automatically

---

## 🎯 **Behavior Examples**

| Obstacle Type | Velocity | Roll | Speed | Notes |
|---------------|----------|------|-------|-------|
| Static building | 0 m/s | 12° | 60% | Normal |
| Moving away | +2 m/s | 12° | 60% | Normal |
| Approaching slow | -1.5 m/s | 15.6° | 60% | Boosted |
| Approaching fast | -3.5 m/s | 18.7° | 48% | Maximum |

---

## ⚙️ **Tuning (Optional)**

Edit `/home/eren/bitirme_repo/ros2_ws/src/obstacle_avoidance_node.py`:

```python
# Line 43-46: Adjust these if needed
DYNAMIC_URGENCY_BOOST = 1.3         # Default: 1.3 (30% boost)
DYNAMIC_ROLL_BOOST = 1.2            # Default: 1.2 (20% extra)
APPROACHING_VELOCITY_THRESHOLD = -1.0  # When to boost (m/s)
FAST_APPROACHING_THRESHOLD = -2.5    # When to maximize (m/s)
```

**More aggressive:**
- Increase boosts to 1.5 and 1.3
- Lower thresholds to -0.5 and -2.0

**Less aggressive:**
- Decrease boosts to 1.2 and 1.1
- Raise thresholds to -1.5 and -3.0

---

## 📝 **Testing Checklist**

- [ ] Static obstacle still works (cube in path)
- [ ] Moving obstacle detected as "DYNAMIC"
- [ ] Velocity shown in logs (Vel: X.Xm/s)
- [ ] Approaching obstacle gets boosted avoidance
- [ ] Fast approaching gets maximum avoidance
- [ ] Obstacle turning around triggers re-avoid
- [ ] Mission completes successfully

---

## 🐛 **Common Issues**

**All obstacles show "DYNAMIC" even static ones:**
- Depth camera noise
- Fix: In Unity Inspector, increase `Dynamic Velocity Threshold` to 0.8

**Dynamic obstacles not detected:**
- Velocity too low
- Fix: In Unity Inspector, decrease `Dynamic Velocity Threshold` to 0.3

**Not avoiding aggressively enough:**
- Boost too low
- Fix: In Python, increase `DYNAMIC_URGENCY_BOOST` to 1.5

---

## 📖 **Full Documentation**

See [DYNAMIC_OBSTACLE_AVOIDANCE.md](DYNAMIC_OBSTACLE_AVOIDANCE.md) for:
- Detailed implementation explanation
- Complete message format
- Advanced tuning guide
- Multiple test scenarios
- Troubleshooting guide

---

**Status:** ✅ Ready to test!
**Time to implement:** 0 minutes (already done!)
**Time to test:** 5 minutes
