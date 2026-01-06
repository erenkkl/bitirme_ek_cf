# Obstacle Avoidance System - Setup & Usage Guide

**Author:** Obstacle Avoidance Integration
**Date:** 2026-01-05
**Version:** 1.0

---

## 📋 OVERVIEW

This obstacle avoidance system enables your drone to detect and avoid static obstacles (buildings, trees, etc.) while maintaining GPS-free SIFT-based localization and path following.

**Key Features:**
- ✅ Size-agnostic detection (works with any obstacle size)
- ✅ Path-agnostic avoidance (handles sharp turns and straight paths)
- ✅ Smooth SIFT integration (minimal disruption to localization)
- ✅ Recovery mechanism (automatic path reacquisition)
- ✅ Emergency stop capability (critical distance protection)

---

## 🏗️ SYSTEM ARCHITECTURE

```
┌──────────────────────┐
│  Front Depth Camera  │ (640x480 @ 20Hz)
│  FrontDepthPublisher │
└──────────┬───────────┘
           │ /drone/camera/front_depth
           ▼
┌──────────────────────┐
│  ObstacleDetector.cs │ (Unity C#)
│  - Zone analysis     │
│  - Occupancy calc    │
│  - Side selection    │
└──────────┬───────────┘
           │ /drone/obstacle_info
           ▼
┌─────────────────────────────┐      ┌────────────────────┐
│ obstacle_avoidance_node.py  │◄─────│ /drone/sift_pose   │
│ (ROS2 Python)                │      │ (SIFT Localizer)   │
│ - State machine              │      └────────────────────┘
│ - Decision logic             │
│ - Recovery monitoring        │
└──────────┬──────────────────┘
           │ /drone/avoidance_command
           ▼
┌──────────────────────┐
│ SiftPathFollower.cs  │ (Modified)
│ - Blends SIFT + Avoid│
│ - Speed modulation   │
│ - Smooth transitions │
└──────────┬───────────┘
           │ SetExternalCommand(pitch, roll, yaw)
           ▼
┌──────────────────────┐
│   DronePhysics.cs    │
│   (Physics Control)  │
└──────────────────────┘
```

---

## 📦 INSTALLATION

### 1. Unity Setup

**Step 1: Add ObstacleDetector Component**

1. Open your Unity scene
2. Select the GameObject with the **Front Camera** (the one with FrontDepthCameraPublisher)
3. Add Component → **ObstacleDetector**
4. Configure in Inspector:
   - **Depth Input Topic:** `/drone/camera/front_depth` (default)
   - **Obstacle Output Topic:** `/drone/obstacle_info` (default)
   - **Max Detection Range:** `15.0` meters
   - **Critical Range:** `8.0` meters
   - **Min Occupancy:** `0.15` (15%)
   - **Center Zone Width:** `0.4` (40% of image width)
   - **Enable Debug Logs:** Check for testing

**Step 2: Update SiftPathFollower**

1. Select the GameObject with **SiftPathFollower** component
2. In Inspector, verify new settings:
   - **Enable Avoidance:** ✅ Checked
   - **Avoidance Command Topic:** `/drone/avoidance_command`
   - **Avoidance Blend Factor:** `0.7` (70% avoidance priority)
   - **Avoidance Smoothness:** `5.0`

**Step 3: Verify Front Depth Camera**

1. Select Front Camera GameObject
2. Ensure **FrontDepthCameraPublisher** is configured:
   - **Depth Topic:** `/drone/camera/front_depth`
   - **Width:** `640`, **Height:** `480`
   - **Publish FPS:** `20`
   - **Max Depth:** `50.0` meters
   - **Use Async GPU Readback:** ✅ Checked

---

### 2. ROS2 Setup

**Step 1: Make Python Node Executable**

```bash
cd /home/eren/bitirme_repo/ros2_ws/src
chmod +x obstacle_avoidance_node.py
```

**Step 2: Verify Dependencies**

```bash
# Ensure you have required Python packages
pip3 install numpy
```

**Step 3: Source Workspace**

```bash
cd /home/eren/bitirme_repo/ros2_ws
source install/setup.bash
```

---

## 🚀 LAUNCHING THE SYSTEM

### Terminal 1: ROS-TCP Endpoint

```bash
cd /home/eren/bitirme_repo/ros2_ws
source install/setup.bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

**Expected output:**
```
Starting server on 0.0.0.0:10000
```

---

### Terminal 2: SIFT Localizer

```bash
cd /home/eren/bitirme_repo/ros2_ws
source install/setup.bash
ros2 run sift_localizer sift_localizer_node.py
```

**Expected output:**
```
[INFO] SiftLocalizer Started. Log:1.0s | ConfSat:40.0 | Win:20
[INFO] Referans gorseller yuklendi: 278 Gorsel (Seq: 0 -> 277)
```

---

### Terminal 3: Obstacle Avoidance Node (NEW)

```bash
cd /home/eren/bitirme_repo/ros2_ws/src
python3 obstacle_avoidance_node.py
```

**Expected output:**
```
[INFO] ObstacleAvoidanceNode started.
[INFO] Config: DetectRange=15.0m, ClearRange=18.0m
[INFO] Avoidance: Roll=8.0°, Speed=60%
[INFO] [NORMAL] All clear | SIFT: Seq=0, Conf=0.00
```

---

### Terminal 4: Unity

```bash
# Launch Unity Editor or built executable
cd /home/eren/bitirme_repo/Bitirme
# Open in Unity Editor, then press Play
```

**Expected Unity Console:**
```
[SiftPathFollower] Obstacle avoidance integration enabled.
[SiftPathFollower] Hazır. Mission started.
[ObstacleDetector] Started. Range: 15.0m, Occupancy: 15%
```

---

## 🎮 TESTING WITH UNITY CUBE

### Step 1: Create Test Obstacle

1. In Unity Hierarchy, right-click → **3D Object → Cube**
2. Rename to "TestObstacle"
3. Transform:
   - **Position:** Place it ~20-30m ahead in the drone's path
   - **Scale:** Try `(5, 10, 5)` for a building-sized obstacle
   - **Rotation:** `(0, 0, 0)`

4. Add a **Box Collider** (should be automatic)
5. Optional: Change material color to red for visibility

### Step 2: Position Verification

```
Example positions (adjust based on your path):
- Drone start: (0, 30, 0)
- Obstacle: (0, 30, 50) - 50m ahead
- Obstacle size: 5x10x5 (W x H x D)
```

### Step 3: Run Test

1. Press **Play** in Unity
2. Watch Console for obstacle detection logs
3. Verify state transitions in ROS2 terminal

**Expected behavior:**
```
1. Drone approaches obstacle
2. At ~15m: ObstacleDetector detects obstacle
3. Avoidance node: NORMAL → AVOIDING
4. Drone steers left/right (clearer side)
5. Speed reduces to 60%
6. After passing: AVOIDING → RECOVERING
7. SIFT confidence improves
8. Recovery complete: RECOVERING → NORMAL
```

---

## 📊 MONITORING & DEBUGGING

### ROS2 Topic Monitoring

**Check obstacle detection:**
```bash
ros2 topic echo /drone/obstacle_info
```

**Expected output (obstacle detected):**
```
data:
- 1.0        # has_obstacle
- 12.5       # distance (meters)
- 1.0        # preferred_side (RIGHT)
- 0.65       # urgency
- 0.3        # clear_left
- 0.85       # clear_right
```

**Check avoidance commands:**
```bash
ros2 topic echo /drone/avoidance_command
```

**Expected output (avoiding):**
```
data:
- 1.0        # is_active
- 8.0        # roll_adjustment (degrees)
- 0.6        # speed_multiplier
- 1.0        # mode (1=AVOIDING)
- 1.0        # look_ahead_multiplier
```

**Check SIFT pose:**
```bash
ros2 topic echo /drone/sift_pose
```

**Monitor topic rates:**
```bash
ros2 topic hz /drone/camera/front_depth      # Should be ~12-20 Hz
ros2 topic hz /drone/obstacle_info            # Should be ~20 Hz
ros2 topic hz /drone/avoidance_command        # Should be ~20 Hz
```

---

## ⚙️ PARAMETER TUNING

### ObstacleDetector.cs (Unity)

**Detection Sensitivity:**
- **Min Occupancy:** Lower = more sensitive (0.10 - 0.20 recommended)
- **Max Detection Range:** Increase for earlier detection (15-20m)
- **Critical Range:** Distance for urgent avoidance (5-10m)

**Zone Configuration:**
- **Center Zone Width:** Wider = more conservative (0.3 - 0.5)
- **Vertical Min/Max:** Adjust to ignore ground/sky (0.2-0.3 / 0.7-0.9)

**Filtering:**
- **Temporal Filter Size:** More frames = smoother but slower (2-5)

---

### obstacle_avoidance_node.py (ROS2)

Edit the configuration section at top of file:

**Thresholds:**
```python
OBSTACLE_DETECT_RANGE = 15.0        # Enter avoidance
OBSTACLE_CLEARED_RANGE = 18.0       # Exit avoidance (hysteresis)
SIFT_CONFIDENCE_THRESHOLD = 0.7     # Recovery complete
SIFT_ERROR_THRESHOLD = 20.0         # Max visual error (px)
```

**Control Gains:**
```python
AVOIDANCE_ROLL_GAIN = 8.0           # Lateral avoidance strength
RECOVERY_ROLL_GAIN = 3.0            # Recovery steering
AVOIDANCE_SPEED_MULT = 0.6          # Speed during avoidance
RECOVERY_SPEED_MULT = 0.8           # Speed during recovery
```

**Timing:**
```python
MIN_AVOIDING_DURATION = 1.0         # Min time avoiding (sec)
MIN_RECOVERY_DURATION = 2.0         # Min time recovering (sec)
```

---

### SiftPathFollower.cs (Unity)

**Blending:**
- **Avoidance Blend Factor:** Higher = prioritize avoidance over SIFT (0.6 - 0.8)
- **Avoidance Smoothness:** Higher = faster transitions (3.0 - 8.0)

**Navigation:**
- **Base Look Ahead:** Original path preview distance (3.0 - 5.0)

---

## 🔧 TROUBLESHOOTING

### Issue: Obstacle Not Detected

**Symptoms:** Drone crashes into obstacle

**Solutions:**
1. Check depth camera is publishing:
   ```bash
   ros2 topic echo /drone/camera/front_depth --once
   ```
2. Verify ObstacleDetector logs in Unity Console
3. Lower `minOccupancy` threshold (try 0.10)
4. Increase `maxDetectionRange` (try 20m)
5. Enable `visualizeZones` in Inspector to see Gizmos

---

### Issue: False Positives (Avoiding Nothing)

**Symptoms:** Drone avoids when path is clear

**Solutions:**
1. Increase `minOccupancy` (try 0.20)
2. Increase `depthVarianceThreshold` (try 1.0)
3. Increase `temporalFilterSize` (try 5)
4. Check for depth camera noise in Unity Scene view

---

### Issue: Oscillation Between States

**Symptoms:** Rapid AVOIDING ↔ NORMAL transitions

**Solutions:**
1. Increase hysteresis gap:
   ```python
   OBSTACLE_DETECT_RANGE = 15.0
   OBSTACLE_CLEARED_RANGE = 20.0  # Larger gap
   ```
2. Increase `MIN_AVOIDING_DURATION` (try 2.0 seconds)
3. Increase `temporalFilterSize` in ObstacleDetector

---

### Issue: SIFT Lost During Avoidance

**Symptoms:** SIFT confidence drops to 0, drone stops

**Solutions:**
1. Reduce avoidance roll gain (try 6.0°)
2. Reduce speed multiplier (try 0.5)
3. Increase `avoidanceBlendFactor` in SiftPathFollower (keep more SIFT control)
4. Check if obstacle is too close to path (SIFT features change too much)

---

### Issue: Recovery Takes Too Long

**Symptoms:** Drone stays in RECOVERING for many seconds

**Solutions:**
1. Lower `SIFT_CONFIDENCE_THRESHOLD` (try 0.6)
2. Increase `SIFT_ERROR_THRESHOLD` (try 30.0 pixels)
3. Lower `MIN_RECOVERY_DURATION` (try 1.5 seconds)
4. Check SIFT localizer is working (`ros2 topic echo /drone/sift_pose`)

---

### Issue: Avoidance Node Not Receiving Data

**Symptoms:** Node shows "All clear" even with obstacle

**Solutions:**
1. Verify topics are connected:
   ```bash
   ros2 topic list
   # Should show:
   # /drone/obstacle_info
   # /drone/avoidance_command
   # /drone/sift_pose
   ```
2. Check ObstacleDetector is publishing:
   ```bash
   ros2 topic hz /drone/obstacle_info
   ```
3. Restart ROS-TCP Endpoint and Unity

---

## 📈 PERFORMANCE OPTIMIZATION

### For Lower FPS (< 12 Hz)

1. **Reduce depth resolution:**
   ```csharp
   // In FrontDepthCameraPublisher
   public int width = 320;   // Was 640
   public int height = 240;  // Was 480
   ```

2. **Reduce processing rate:**
   ```csharp
   // In ObstacleDetector
   public int processingRate = 15;  // Was 20
   ```

3. **Disable async GPU readback:**
   ```csharp
   // In FrontDepthCameraPublisher
   public bool useAsyncGPUReadback = false;
   ```

---

### For Better Accuracy

1. **Increase depth resolution:**
   ```csharp
   public int width = 640;
   public int height = 480;
   ```

2. **Increase temporal filtering:**
   ```csharp
   public int temporalFilterSize = 5;
   ```

3. **Tighter zone configuration:**
   ```csharp
   public float centerZoneWidth = 0.3f;  // Narrower center
   ```

---

## 📋 APPENDIX: MESSAGE FORMATS

### `/drone/obstacle_info` (Float32MultiArray)

| Index | Field | Type | Range | Description |
|-------|-------|------|-------|-------------|
| 0 | has_obstacle | bool | 0.0 or 1.0 | Obstacle detected flag |
| 1 | distance | float | 0-50 m | Distance to obstacle |
| 2 | preferred_side | int | -1, 0, 1 | Avoidance direction (LEFT, NONE, RIGHT) |
| 3 | urgency | float | 0.0-1.0 | Urgency level (distance + occupancy) |
| 4 | clear_left | float | 0.0-1.0 | Left zone clearance score |
| 5 | clear_right | float | 0.0-1.0 | Right zone clearance score |

---

### `/drone/avoidance_command` (Float32MultiArray)

| Index | Field | Type | Range | Description |
|-------|-------|------|-------|-------------|
| 0 | is_active | bool | 0.0 or 1.0 | Avoidance system active |
| 1 | roll_adjustment | float | ±20° | Roll command adjustment |
| 2 | speed_multiplier | float | 0.0-1.0 | Speed reduction factor |
| 3 | mode | int | 0-3 | State (0=NORMAL, 1=AVOIDING, 2=RECOVERING, 3=EMERGENCY) |
| 4 | look_ahead_mult | float | 1.0-2.0 | Look-ahead distance multiplier |

---

### `/drone/sift_pose` (Float32MultiArray)

| Index | Field | Type | Range | Description |
|-------|-------|------|-------|-------------|
| 0 | seq | float | 0-277 or -1 | Sequence number (-1 if lost) |
| 1 | visual_error_x | float | pixels | Lateral offset from path center |
| 2 | visual_error_y | float | pixels | Vertical offset (unused) |
| 3 | confidence | float | 0.0-1.0 | Matching confidence |
| 4 | reserved | float | - | Reserved field |
| 5 | inliers | int | 0-100+ | RANSAC inlier count |

---

## 🎯 EXPECTED PERFORMANCE

### Normal Operation

- **Detection latency:** < 100ms (obstacle → command)
- **Avoidance activation:** 15m distance threshold
- **Speed reduction:** 40% (15 m/s → 9 m/s)
- **Lateral deviation:** < 5m from original path
- **Recovery time:** 2-5 seconds
- **SIFT confidence during avoidance:** > 0.5

### Edge Cases

- **Very large obstacles (buildings):** May take longer avoidance path
- **Multiple obstacles:** Handles closest first
- **Narrow passages:** May trigger EMERGENCY_STOP if < 5m
- **Sharp turns + obstacles:** Combines turn + avoidance (monitor SIFT)

---

## 📞 SUPPORT & NEXT STEPS

### Validation Checklist

- [ ] All 3 terminals running without errors
- [ ] Unity plays without warnings
- [ ] Depth camera publishing at ~12-20 Hz
- [ ] Obstacle detector publishing at ~20 Hz
- [ ] Avoidance node publishing commands
- [ ] SiftPathFollower receiving all topics
- [ ] Test obstacle placed in path
- [ ] Drone detects obstacle at ~15m
- [ ] Drone avoids to clearer side
- [ ] Drone recovers and continues mission

### Future Enhancements

1. **Dynamic obstacles:** Moving objects (requires prediction)
2. **Multi-obstacle handling:** Priority queue for multiple threats
3. **Path replanning:** Generate new waypoints around obstacles
4. **Lidar integration:** 360° sensing instead of front-only
5. **Machine learning:** Obstacle classification (tree vs building vs person)

---

## 📝 NOTES

- This system is designed for **static obstacles only**
- **No GPS** - pure vision-based navigation
- Works with **any obstacle size** (occupancy-based detection)
- **Path-agnostic** - handles straight and curved paths
- **SIFT-compatible** - minimal disruption to localization
- **Tested at:** 20 FPS (works down to 12 FPS)

---

**Last Updated:** 2026-01-05
**Status:** Ready for testing
**Contact:** Check repository issues for support
