# Trajectory Publisher - Hướng Dẫn Sử Dụng

**Module:** `trajectory_publisher.py`  
**Author:** Thuong Tran Dinh  
**Date:** October 26, 2025

---

## 📖 Tổng Quan

Module này sinh ra các quỹ đạo tham chiếu (reference trajectories) và publish lên topic `/trajectory` để **Fuzzy Controller** có thể theo dõi.

**Dựa trên code MATLAB:** `trajectory_reference.m`

---

## 🎯 Các Loại Quỹ Đạo

### 1️⃣ **Hình Tròn (Circle)**
```python
trajectory_type = 1
```

**Phương trình:**
```
x(t) = R * cos(ω*t)
y(t) = R * sin(ω*t)
θ(t) = ω*t + π/2
```

**Tham số:**
- `R = 2.0` m - Bán kính
- `ω = 0.2` rad/s - Tốc độ góc
- `center = [0, 0]` - Tâm quỹ đạo

**Đặc điểm:**
- ✅ Quỹ đạo mượt mà, liên tục
- ✅ Tốc độ tuyến tính không đổi: `v = R*ω = 0.4 m/s`
- ✅ Tốc độ góc không đổi: `ω = 0.2 rad/s`
- ✅ Phù hợp test controller cơ bản

---

### 2️⃣ **Hình Vuông (Square)**
```python
trajectory_type = 2
```

**Phương trình:**
```
Cạnh 1: x = side/2,           y = -side/2 → side/2,     θ = π/2
Cạnh 2: x = side/2 → -side/2, y = side/2,               θ = π
Cạnh 3: x = -side/2,          y = side/2 → -side/2,     θ = -π/2
Cạnh 4: x = -side/2 → side/2, y = -side/2,              θ = 0
```

**Tham số:**
- `side = 4.0` m - Chiều dài cạnh
- `T_side = 20` s - Thời gian đi hết một cạnh
- `v = side/T_side = 0.2 m/s` - Tốc độ di chuyển

**Đặc điểm:**
- ⚠️ **Có góc sắc** tại 4 đỉnh (thách thức cho controller)
- ✅ Test khả năng xoay 90° tại mỗi góc
- ✅ Test khả năng duy trì hướng thẳng trên mỗi cạnh
- 🎯 **Khó hơn** hình tròn - yêu cầu controller tốt

---

### 3️⃣ **Hình Số 8 (Figure-8 / Lemniscate)**
```python
trajectory_type = 3  # MẶC ĐỊNH
```

**Phương trình (Lemniscate of Gerono):**
```
x(t) = A * cos(ω*t)
y(t) = A * sin(2*ω*t) / 2
θ(t) = atan2(dy/dt, dx/dt)
```

**Đạo hàm (để tính góc tiếp tuyến):**
```
dx/dt = -A*ω * sin(ω*t)
dy/dt = A*ω * cos(2*ω*t)
```

**Tham số:**
- `A = 2.0` m - Biên độ (nửa chiều rộng)
- `ω = 0.2` rad/s - Tốc độ góc

**Đặc điểm:**
- ✅ **Quỹ đạo phức tạp nhất** - hình số 8
- ✅ Có điểm giao nhau ở tâm (0, 0)
- ✅ Tốc độ và gia tốc thay đổi liên tục
- ✅ Test đầy đủ khả năng controller
- 🎯 **Khuyến nghị** để test performance thực tế

---

## 🚀 Cách Sử Dụng

### **1. Build Package**

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select agv_trajectory_tracking
source install/setup.bash
```

### **2. Chạy Trajectory Publisher**

#### **Chạy với quỹ đạo mặc định (Figure-8):**
```bash
ros2 run agv_trajectory_tracking trajectory_publisher
```

#### **Chọn quỹ đạo hình tròn:**
```bash
ros2 run agv_trajectory_tracking trajectory_publisher \
  --ros-args -p trajectory_type:=1
```

#### **Chọn quỹ đạo hình vuông:**
```bash
ros2 run agv_trajectory_tracking trajectory_publisher \
  --ros-args -p trajectory_type:=2
```

#### **Chọn quỹ đạo hình số 8:**
```bash
ros2 run agv_trajectory_tracking trajectory_publisher \
  --ros-args -p trajectory_type:=3
```

### **3. Output Mong Đợi**

```
[INFO] [trajectory_publisher]: Trajectory Publisher started!
[INFO] [trajectory_publisher]:   Type: Figure-8 (Lemniscate)
[INFO] [trajectory_publisher]:   Publishing at 10.0 Hz
[INFO] [trajectory_publisher]:   Path points: 200
[INFO] [trajectory_publisher]: Published trajectory | t=0.20s | Current ref: x=1.960, y=0.159, θ=-4.7°
[INFO] [trajectory_publisher]: Published trajectory | t=2.20s | Current ref: x=1.627, y=0.691, θ=-24.5°
...
```

---

## ⚙️ Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `trajectory_type` | int | `3` | Loại quỹ đạo (1:Circle, 2:Square, 3:Figure-8) |
| `publish_rate` | float | `10.0` | Tần số publish (Hz) |
| `path_points` | int | `200` | Số điểm trong path preview |
| `preview_time` | float | `20.0` | Thời gian preview (giây) |

### **Thay Đổi Parameters:**

```bash
ros2 run agv_trajectory_tracking trajectory_publisher \
  --ros-args \
  -p trajectory_type:=1 \
  -p publish_rate:=20.0 \
  -p path_points:=300 \
  -p preview_time:=30.0
```

---

## 📡 Topics

### **Published:**

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/trajectory` | `nav_msgs/Path` | 10 Hz | Quỹ đạo tham chiếu (200 điểm) |

### **Message Structure:**

```python
Path:
  header:
    stamp: ros_time
    frame_id: "map"
  poses: [PoseStamped × 200]
    - header:
        stamp: ros_time
        frame_id: "map"
      pose:
        position:
          x: float  # [m]
          y: float  # [m]
          z: 0.0
        orientation:
          x, y, z, w: quaternion  # từ theta_ref
```

---

## 🔧 Kết Hợp Với Fuzzy Controller

### **Terminal 1: Start Trajectory Publisher**
```bash
ros2 run agv_trajectory_tracking trajectory_publisher \
  --ros-args -p trajectory_type:=3
```

### **Terminal 2: Start Fuzzy Controller**
```bash
ros2 run agv_trajectory_tracking fuzzy_trajectory_controller
```

### **Terminal 3: Monitor (Optional)**
```bash
# Xem trajectory
ros2 topic echo /trajectory

# Xem velocity commands
ros2 topic echo /diff_cont/cmd_vel

# Xem robot pose
ros2 topic echo /amcl_pose
```

---

## 📊 Visualization trong RViz

### **1. Mở RViz**
```bash
rviz2
```

### **2. Thêm Display cho Trajectory**

**Add → By topic → /trajectory → Path**

**Settings:**
- Topic: `/trajectory`
- Color: Red (hoặc màu nổi bật)
- Alpha: 1.0
- Line Width: 0.05

### **3. Thêm Display cho Robot Pose**

**Add → By topic → /amcl_pose → PoseWithCovariance**

**Settings:**
- Topic: `/amcl_pose`
- Shaft Length: 0.5
- Shaft Radius: 0.05
- Head Length: 0.2
- Head Radius: 0.1

### **4. Fixed Frame**
```
Global Options → Fixed Frame: "map"
```

---

## 🧮 So Sánh Code MATLAB vs Python

| Feature | MATLAB Code | Python Code | Note |
|---------|-------------|-------------|------|
| **Hình tròn** | ✅ | ✅ | Giống hệt |
| **Hình vuông** | ✅ | ✅ | Giống hệt |
| **Hình số 8** | ✅ | ✅ | Giống hệt |
| **wrapToPi** | `wrapToPi()` | `normalize_angle()` | Tương đương |
| **Quaternion** | Không có | ✅ `quaternion_from_euler()` | Thêm cho ROS2 |
| **Path message** | Không có | ✅ `nav_msgs/Path` | Thêm cho ROS2 |
| **Time update** | Input `t` | ✅ `self.current_time += dt` | Auto increment |

---

## 📐 Chi Tiết Toán Học

### **Hình Tròn**
```
Chu kỳ: T = 2π/ω = 31.4 s
Chu vi: C = 2πR = 12.57 m
Tốc độ: v = C/T = 0.4 m/s
```

### **Hình Vuông**
```
Chu kỳ: T = 4 × T_side = 80 s
Chu vi: C = 4 × side = 16 m
Tốc độ: v = C/T = 0.2 m/s
```

### **Hình Số 8**
```
Chu kỳ: T = 2π/ω = 31.4 s
Chiều rộng: W = 2A = 4 m
Chiều cao: H = A = 2 m
Tốc độ: Thay đổi (0.1 - 0.6 m/s)
```

---

## 🐛 Troubleshooting

### **Problem: Topic /trajectory không có data**
```bash
# Kiểm tra node đang chạy
ros2 node list | grep trajectory

# Kiểm tra topic
ros2 topic list | grep trajectory

# Echo topic
ros2 topic echo /trajectory
```

### **Problem: Controller không theo trajectory**
```bash
# Kiểm tra controller có subscribe /trajectory không
ros2 node info /fuzzy_trajectory_controller

# Kiểm tra AMCL có publish pose không
ros2 topic hz /amcl_pose

# Kiểm tra cmd_vel có được publish không
ros2 topic hz /diff_cont/cmd_vel
```

### **Problem: Quỹ đạo không đúng**
- ✅ Kiểm tra `trajectory_type` parameter
- ✅ Kiểm tra frame_id = "map" trong Path message
- ✅ Verify quaternion calculation (orientation)

---

## 🔬 Testing Plan

### **Test 1: Hình Tròn (Dễ)**
```bash
# Easy test - smooth circular motion
ros2 run agv_trajectory_tracking trajectory_publisher \
  --ros-args -p trajectory_type:=1
```
**Expected:** Robot di chuyển tròn mượt mà, không rung

### **Test 2: Hình Vuông (Trung bình)**
```bash
# Medium test - sharp corners
ros2 run agv_trajectory_tracking trajectory_publisher \
  --ros-args -p trajectory_type:=2
```
**Expected:** Robot xoay 90° tại mỗi góc, có thể overshoot

### **Test 3: Hình Số 8 (Khó)**
```bash
# Hard test - complex trajectory
ros2 run agv_trajectory_tracking trajectory_publisher \
  --ros-args -p trajectory_type:=3
```
**Expected:** Robot theo hình số 8, test đầy đủ controller

---

## 📈 Performance Metrics

### **Đo lường chất lượng tracking:**

1. **Distance Error (e_D)**: Khoảng cách từ robot đến trajectory
   - Target: < 0.15 m

2. **Angle Error (e_Theta)**: Góc chênh lệch hướng
   - Target: < 15°

3. **Smoothness**: Độ mượt của cmd_vel
   - Target: Không rung (no oscillation)

4. **Goal Reaching**: Đạt được goal point
   - Target: < 0.15 m

---

## 📝 Next Steps

- [ ] Test với từng loại trajectory
- [ ] Tune fuzzy rules dựa trên tracking error
- [ ] Record bag file để phân tích
- [ ] Plot trajectory vs actual path
- [ ] Measure performance metrics

---

## 🎓 Kết Luận

**Trajectory Publisher** đã được implement đầy đủ theo code MATLAB với:

✅ **3 loại quỹ đạo** - Circle, Square, Figure-8  
✅ **ROS2 native** - Publish nav_msgs/Path  
✅ **Parametric design** - Dễ dàng thay đổi trajectory  
✅ **Preview path** - 200 điểm, 20 giây trước  
✅ **Map frame** - Khớp với AMCL pose  

Sẵn sàng để test với Fuzzy Controller! 🚀
