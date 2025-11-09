# 🎯 Sensor Fusion Strategy: ArUco-Dominant Fusion

## 📖 Overview

Chiến lược sensor fusion này ưu tiên **ArUco markers như ground truth tuyệt đối** khi được phát hiện, và sử dụng **AMCL như fallback** khi không nhìn thấy markers.

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Sensor Fusion Flow                        │
└─────────────────────────────────────────────────────────────┘

ArUco Detected?
    │
    ├─ YES → 🎯 ArUco (ABSOLUTE TRUTH)
    │         - Covariance: 0.001 - 0.005 (very low)
    │         - Rejection threshold: 100.0 (never reject)
    │         - Weight: 95%+ in fusion
    │
    └─ NO  → 📡 AMCL (Fallback)
              - Covariance: 0.2 - 0.35 (moderate)
              - Rejection threshold: 5.0 (can drift a bit)
              - Weight: 100% when ArUco absent

           + 🚗 Wheel Odometry (Velocity prediction)
              - Only for smooth interpolation
              - vx, vy, vyaw (no position/orientation)
```

## 🔧 Key Configuration Parameters

### 1. ArUco Covariance (aruco_detector_sim.cpp)

```cpp
// VERY LOW covariance = HIGH trust
double var_xy = 0.001 + dist * 0.0005;   // Position: ~0.001-0.003
double var_rot = 0.005 + dist * 0.002;   // Orientation: ~0.005-0.015

// Compare with AMCL: 0.2-0.35
// → ArUco is 100x more trusted!
```

**Rationale**: 
- Chúng ta **BIẾT CHÍNH XÁC** vị trí markers trong map
- Detection noise là tối thiểu (OpenCV ArUco rất accurate)
- Covariance chỉ tăng nhẹ theo khoảng cách (accounting for camera noise)

### 2. EKF Rejection Thresholds (ekf_map_fusion.yaml)

```yaml
# AMCL: Can be rejected if too different from filter state
pose0_rejection_threshold: 5.0

# ArUco: NEVER reject (trust absolutely)
pose1_rejection_threshold: 100.0
```

**Rationale**:
- AMCL có thể drift → cho phép EKF reject nếu quá khác biệt
- ArUco là ground truth → không bao giờ reject, luôn chấp nhận correction

### 3. Process Noise Covariance (ekf_map_fusion.yaml)

```yaml
# HIGH process noise = trust measurements more than predictions
process_noise_covariance:
  - Position (x, y): 0.5    # 10x higher than default
  - Yaw: 0.3                # 5x higher than default
  - Velocity: 0.1           # Moderate for smooth prediction
```

**Rationale**:
- Cao → EKF tin vào measurements (AMCL, ArUco) hơn là prediction model
- Khi ArUco xuất hiện với covariance rất thấp, nó sẽ "pull" state estimate mạnh mẽ về đúng vị trí

## 📊 Expected Behavior

### Scenario 1: ArUco Visible ✅

```
Robot at: (actual)     → (5.0, 3.0, 90°)
AMCL estimate:         → (5.5, 3.2, 95°)    [drifted]
ArUco detection:       → (5.0, 3.0, 90°)    [ground truth]

Fused output:          → (5.05, 3.02, 90.5°)  [~95% ArUco, ~5% AMCL]
```

**Result**: Fusion follows ArUco almost exactly, minor smoothing from AMCL

### Scenario 2: ArUco Lost ❌

```
Last ArUco:            → (5.0, 3.0, 90°)  [2 seconds ago]
Current AMCL:          → (6.2, 3.5, 95°)  [continuous updates]
Wheel odom velocity:   → vx=0.5 m/s, vyaw=0.1 rad/s

Fused output:          → (6.2, 3.5, 95°)  [100% AMCL + velocity prediction]
```

**Result**: Seamlessly transitions to AMCL, no discontinuity

### Scenario 3: ArUco Re-appears After Drift 🔄

```
Current estimate:      → (8.0, 5.0, 120°)  [AMCL drifted significantly]
ArUco detection:       → (7.0, 4.5, 110°)  [ground truth after 30 seconds]

Fused output (t=0):    → (8.0, 5.0, 120°)
Fused output (t=1):    → (7.1, 4.55, 111°)  [rapid correction]
Fused output (t=2):    → (7.05, 4.52, 110.5°)  [converged to ArUco]
```

**Result**: Quick correction (~1-2 seconds) when ArUco re-appears, no matter how much AMCL drifted

## 🎛️ Tuning Guide

### If fusion follows AMCL too much (ignores ArUco):

1. ✅ **Decrease ArUco covariance** (already at 0.001, very low)
2. ✅ **Increase process noise** (already at 0.5, high)
3. ✅ **Increase pose1_rejection_threshold** (already at 100.0, very high)
4. ⚠️ Check if ArUco is actually publishing (topic hz)

### If fusion is too jumpy when ArUco appears:

1. Add **smooth transition period** (not implemented yet)
2. Decrease process noise slightly (0.5 → 0.3)
3. Add velocity damping

### If AMCL is not good enough as fallback:

1. Improve AMCL parameters (particle count, update rates)
2. Add IMU fusion to wheel odometry
3. Consider adding more ArUco markers in environment

## 📈 Performance Metrics

Monitor with `fusion_monitor.py`:

```bash
# Expected metrics when ArUco detected:
ArUco vs Fused:   < 0.1m position,  < 5° orientation  ✅
AMCL vs Fused:    < 0.5m position,  < 10° orientation ✅

# Expected metrics when ArUco lost:
AMCL vs Fused:    < 0.2m position,  < 5° orientation  ✅
```

## 🚀 Usage

```bash
# Terminal 1: Launch simulation
ros2 launch mobile_robot launch_sim.launch.py

# Terminal 2: Launch fusion localization
ros2 launch agv_localization localization_fusion.launch.py

# Terminal 3: Monitor fusion quality
ros2 run agv_localization fusion_monitor.py

# Terminal 4: Visualize in RViz
rviz2 -d $(ros2 pkg prefix agv_localization)/share/agv_localization/rviz/fusion.rviz
```

## 🔍 Debugging

### Check ArUco covariance:
```bash
ros2 topic echo /aruco/pose_with_covariance --once
# Look at covariance[0] (x), covariance[7] (y), covariance[35] (yaw)
# Should be 0.001-0.005 range
```

### Check AMCL covariance:
```bash
ros2 topic echo /amcl_pose --once
# Look at covariance[0] (x), covariance[7] (y), covariance[35] (yaw)
# Should be 0.1-0.5 range (much higher than ArUco)
```

### Verify EKF is subscribing:
```bash
ros2 topic info /amcl_pose --verbose
ros2 topic info /aruco/pose_with_covariance --verbose
# Both should show "ekf_filter_node" as subscriber
```

### Check fusion output:
```bash
ros2 topic echo /odometry/map_fused
# This is the final fused estimate
```

## 📚 References

- [robot_localization documentation](http://docs.ros.org/en/melodic/api/robot_localization/html/index.html)
- [ArUco marker detection](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)
- [AMCL (Adaptive Monte Carlo Localization)](http://wiki.ros.org/amcl)
- [Extended Kalman Filter (EKF)](https://en.wikipedia.org/wiki/Extended_Kalman_filter)

## ✍️ Author

Thuong Tran Dinh - October 27, 2025

## 🎯 Conclusion

Chiến lược này đảm bảo:
- ✅ **Độ chính xác cao** khi có ArUco (ground truth)
- ✅ **Không bị mất tích** khi mất ArUco (AMCL fallback)
- ✅ **Smooth transitions** giữa các nguồn sensor
- ✅ **Robust** với AMCL drift và sensor noise

**Key insight**: ArUco không phải "một trong các sensors" mà là **GROUND TRUTH**, nên covariance phải cực kỳ thấp để EKF tin tưởng tuyệt đối!
