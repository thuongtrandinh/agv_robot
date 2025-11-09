# 🎯 Sensor Fusion: AMCL + ArUco với EKF

## Tổng quan

Hệ thống localization fusion kết hợp 3 nguồn thông tin:
1. **AMCL**: Localization liên tục từ LiDAR + map (có drift)
2. **ArUco Markers**: Absolute position khi detect marker (chính xác nhưng không liên tục)
3. **EKF (Extended Kalman Filter)**: Fusion 2 nguồn trên để có localization tối ưu

## Kiến trúc hệ thống

```
┌─────────────────────────────────────────────────────┐
│                    Map Frame                         │
└─────────────────────────────────────────────────────┘
                         ↑
         ┌───────────────┴───────────────┐
         │                               │
    ┌────┴────┐                    ┌─────┴─────┐
    │  AMCL   │                    │   ArUco   │
    │ (LiDAR) │                    │ (Camera)  │
    └────┬────┘                    └─────┬─────┘
         │                               │
         │ /amcl_pose                    │ /aruco/pose_with_covariance
         │ (continuous)                  │ (when marker detected)
         │                               │
         └───────────────┬───────────────┘
                         │
                  ┌──────┴──────┐
                  │   EKF Node  │
                  │  (robot_    │
                  │ localization)│
                  └──────┬──────┘
                         │
                 /odometry/map_fused  ← Fused optimal pose
```

## Cách hoạt động

### 1. AMCL (Continuous Global Localization)
- **Input**: LiDAR scan + static map
- **Method**: Particle filter
- **Output**: `/amcl_pose` (PoseWithCovarianceStamped)
- **Ưu điểm**: Liên tục, độ trễ thấp
- **Nhược điểm**: Có drift tích lũy, đôi khi jump khi relocalize

### 2. ArUco Marker Detection (Absolute Corrections)
- **Input**: Camera image
- **Method**: Computer vision (detect ArUco markers với known positions)
- **Output**: `/aruco/pose_with_covariance` (PoseWithCovarianceStamped)
- **Ưu điểm**: Absolute position, không drift
- **Nhược điểm**: Chỉ hoạt động khi thấy marker, phụ thuộc ánh sáng

### 3. EKF Fusion
- **Input**: AMCL pose + ArUco pose (khi có)
- **Method**: Extended Kalman Filter với covariance-based weighting
- **Output**: `/odometry/map_fused` (Odometry message)
- **Cơ chế**:
  - Khi KHÔNG thấy marker: tin tưởng AMCL 100%
  - Khi THẤY marker: weight dựa trên covariance
    - ArUco gần → covariance nhỏ → trust ArUco nhiều hơn
    - ArUco xa → covariance lớn → trust AMCL nhiều hơn

## Covariance Strategy

### AMCL Covariance
- Từ AMCL tự tính dựa trên particle distribution
- Lớn khi: robot mới start, relocalize, map ambiguous
- Nhỏ khi: tracking ổn định lâu

### ArUco Covariance (dynamic)
```cpp
double var_xy = 0.01 + distance * 0.005;   // Position variance
double var_z = 0.02 + distance * 0.01;
double var_rot = 0.05 + distance * 0.02;   // Rotation variance
```
- **Gần marker** (0.5m): var_xy ≈ 0.01 → rất tin tưởng
- **Xa marker** (5m): var_xy ≈ 0.035 → ít tin tưởng hơn

### EKF Decision Making
EKF tự động weight dựa trên covariance:
```
weight_amcl = 1 / (1 + cov_amcl)
weight_aruco = 1 / (1 + cov_aruco)

fused_pose = (weight_amcl * amcl_pose + weight_aruco * aruco_pose) / (weight_amcl + weight_aruco)
```

## Cách sử dụng

### 1. Launch hệ thống fusion
```bash
cd ~/ros2_ws
source install/setup.bash

# Launch tất cả: Map Server + AMCL + ArUco + EKF Fusion
ros2 launch agv_localization localization_fusion.launch.py map_name:=small_house
```

### 2. Set initial pose (nếu cần)
```bash
# Trong RViz, click "2D Pose Estimate" để set initial pose cho AMCL
```

### 3. Monitor topics
```bash
# AMCL pose
ros2 topic echo /amcl_pose

# ArUco pose (only when marker detected)
ros2 topic echo /aruco/pose_with_covariance

# Fused pose (output của EKF)
ros2 topic echo /odometry/map_fused
```

### 4. Visualize trong RViz
Thêm các display:
- **TF**: Xem transform tree
- **PoseWithCovariance**: 
  - Topic: `/amcl_pose` (màu đỏ)
  - Topic: `/aruco/pose_with_covariance` (màu xanh)
- **Odometry**: 
  - Topic: `/odometry/map_fused` (màu vàng - fused result)

## Setup ArUco Markers

### Bước 1: In markers
```bash
# Generate markers (nếu chưa có)
cd ~/ros2_ws
# Marker ID 0-249 với dictionary DICT_6X6_250
# Kích thước: 17.3cm x 17.3cm
```

### Bước 2: Đặt markers trong map
- Gắn markers lên tường/sàn ở các vị trí biết trước
- Lưu vị trí trong file: `maps/aruco_markers/aruco_markers.yaml`

Format YAML:
```yaml
markers:
  - id: 0
    position:
      x: 2.5
      y: 1.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
  - id: 1
    position:
      x: -1.5
      y: 3.0
      z: 0.0
    ...
```

### Bước 3: Camera calibration
Đảm bảo camera đã calibrated và publish camera_info đúng.

## Tuning Parameters

### EKF Config: `config/ekf_map_fusion.yaml`

#### Tăng trust ArUco hơn:
```yaml
# Giảm process noise của position
process_noise_covariance[0,0]: 0.01  # x (giảm từ 0.05)
process_noise_covariance[1,1]: 0.01  # y
```

#### Tăng trust AMCL hơn:
```yaml
# Tăng process noise
process_noise_covariance[0,0]: 0.1   # x (tăng từ 0.05)
```

#### Tăng tốc độ convergence:
```yaml
frequency: 50.0  # tăng từ 30Hz
```

### ArUco Covariance Tuning

Trong `aruco_detector_sim.cpp`:
```cpp
// Tin tưởng ArUco nhiều hơn (giảm covariance)
double var_xy = 0.005 + distance * 0.003;  // giảm coefficients

// Tin tưởng ArUco ít hơn (tăng covariance)
double var_xy = 0.02 + distance * 0.01;    // tăng coefficients
```

## Troubleshooting

### 1. Fused pose không smooth
**Nguyên nhân**: EKF frequency thấp hoặc process noise quá lớn
**Giải pháp**:
```yaml
frequency: 50.0
process_noise_covariance: giảm các giá trị xuống
```

### 2. Fused pose không theo ArUco khi detect marker
**Nguyên nhân**: ArUco covariance quá lớn
**Giải pháp**: Giảm covariance coefficients trong code

### 3. Fused pose jump khi detect marker
**Nguyên nhân**: AMCL drift quá xa, ArUco correction quá mạnh
**Giải pháp**: 
- Cải thiện AMCL tuning
- Tăng ArUco covariance để smooth hơn
- Tăng process noise để EKF chấp nhận jump

### 4. ArUco không detect
**Kiểm tra**:
```bash
ros2 topic hz /aruco/pose_with_covariance  # Check publishing rate
ros2 topic echo /zed2/left/image_raw       # Check camera working
```

## Performance Metrics

### Expected Performance:
- **Position accuracy**: 
  - Without ArUco: ±5-10cm (AMCL only)
  - With ArUco near marker (<1m): ±1-2cm
  - With ArUco far from marker (>3m): ±3-5cm
- **Orientation accuracy**: ±2-5 degrees
- **Update rate**: 30Hz (EKF output)

## Tham khảo

- [robot_localization Documentation](http://docs.ros.org/en/humble/p/robot_localization/)
- [AMCL Parameters](https://navigation.ros.org/configuration/packages/configuring-amcl.html)
- [ArUco Marker Detection](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)

---
**Author**: Thuong Tran Dinh  
**Date**: October 27, 2025  
**Version**: 1.0
