# Obstacle Avoidance - Quick Start Guide

## 🚀 5-MINUTE SETUP

### 1. Unity Setup (2 minutes)

**Front Camera (ObstacleDetector):**
1. Add `ObstacleDetector.cs` component to Front Camera GameObject
2. Leave default settings for first test

**Drone (SiftPathFollower):**
1. Select drone GameObject with `SiftPathFollower`
2. Check: **Enable Avoidance** ✅

### 2. ROS2 Setup (1 minute)

```bash
cd /home/eren/bitirme_repo/ros2_ws/src
chmod +x obstacle_avoidance_node.py
```

### 3. Launch (2 minutes)

**Terminal 1:**
```bash
cd /home/eren/bitirme_repo/ros2_ws
source install/setup.bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

**Terminal 2:**
```bash
cd /home/eren/bitirme_repo/ros2_ws/src
python3 sift_localizer_node.py
```

**Terminal 3 (NEW):**
```bash
cd /home/eren/bitirme_repo/ros2_ws/src
python3 obstacle_avoidance_node.py
```

**Terminal 4:**
```bash
# Open Unity and press Play
```

---

## 🧪 QUICK TEST

### Create Test Obstacle in Unity

1. Right-click Hierarchy → 3D Object → Cube
2. Position: Place 30m ahead in drone's path
3. Scale: `(5, 10, 5)`
4. Press Play

### Expected Behavior

```
Distance 20m → Monitoring
Distance 15m → AVOIDING (steers left/right)
Distance 5m  → Passing obstacle (speed 60%)
Distance 18m → RECOVERING (steering back to path)
Distance 25m → NORMAL (path reacquired)
```

---

## 📊 VERIFY IT'S WORKING

**Check obstacle detection:**
```bash
ros2 topic echo /drone/obstacle_info
```

**Should see:**
```
data: [1.0, 12.5, 1.0, 0.65, 0.3, 0.85]
       ^^^  ^^^^  ^^^
       has  dist  side
```

**Check Unity Console:**
```
[ObstacleDetector] OBSTACLE @ 12.3m | Avoid: RIGHT
[AVOIDING] Avoiding RIGHT | Dist: 12.3m
[RECOVERING] Recovery progress | SIFT Conf: 0.75
[NORMAL] All clear
```

---

## 🔧 COMMON FIXES

**"No obstacle detected":**
- Lower `Min Occupancy` to `0.10` in ObstacleDetector Inspector

**"False alarms":**
- Raise `Min Occupancy` to `0.20`

**"Drone oscillates":**
- Edit `obstacle_avoidance_node.py`:
  ```python
  OBSTACLE_CLEARED_RANGE = 20.0  # Increase from 18.0
  ```

**"SIFT lost during avoidance":**
- Edit `obstacle_avoidance_node.py`:
  ```python
  AVOIDANCE_ROLL_GAIN = 6.0  # Reduce from 8.0
  AVOIDANCE_SPEED_MULT = 0.5  # Reduce from 0.6
  ```

---

## 📖 FULL DOCUMENTATION

See [OBSTACLE_AVOIDANCE_SETUP.md](OBSTACLE_AVOIDANCE_SETUP.md) for:
- Complete parameter tuning guide
- Troubleshooting steps
- Performance optimization
- Message format details

---

## ✅ VALIDATION CHECKLIST

- [ ] 3 ROS2 terminals running
- [ ] Unity playing without errors
- [ ] `/drone/obstacle_info` publishing
- [ ] `/drone/avoidance_command` publishing
- [ ] Test obstacle in scene
- [ ] Drone avoids successfully
- [ ] Drone recovers to path

**Ready to fly! 🚁**
