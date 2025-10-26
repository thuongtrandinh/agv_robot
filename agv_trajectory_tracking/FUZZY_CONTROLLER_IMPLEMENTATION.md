# Fuzzy Trajectory Controller Implementation

**Author:** Thuong Tran Dinh  
**Date:** October 26, 2025  
**Package:** `agv_trajectory_tracking`

---

## 📋 Tổng Quan

File này mô tả chi tiết việc triển khai **Fuzzy Logic Trajectory Tracking Controller** cho AGV (Automated Guided Vehicle) **KHÔNG sử dụng thư viện ngoài** (pure Python implementation).

---

## 🎯 Mục Tiêu

Xây dựng bộ điều khiển fuzzy logic để robot tự động theo dõi quỹ đạo (trajectory tracking) với các yêu cầu:

1. ✅ **Không phụ thuộc thư viện fuzzy** (scikit-fuzzy, skfuzzy, etc.)
2. ✅ **Triển khai hoàn toàn bằng Python thuần**
3. ✅ **Tích hợp với ROS2 Humble**
4. ✅ **Sử dụng AMCL localization** cho pose estimation
5. ✅ **Điều khiển differential drive robot**

---

## 🔧 Những Thay Đổi Đã Thực Hiện

### 1. **Sửa Lỗi Subscribe Topic (CRITICAL FIX)**

#### ❌ **Lỗi ban đầu:**
```python
from nav_msgs.msg import Odometry

self.odom_sub = self.create_subscription(Odometry, '/amcl_pose', 
                                          self.odom_callback, 10)
```

**Vấn đề:**
- Topic `/amcl_pose` có message type là `PoseWithCovarianceStamped`
- Nhưng code đang subscribe với type `Odometry` ❌
- **Type mismatch** → Node sẽ không nhận được dữ liệu!

#### ✅ **Sau khi sửa:**
```python
from geometry_msgs.msg import PoseWithCovarianceStamped

self.pose_sub = self.create_subscription(
    PoseWithCovarianceStamped, '/amcl_pose', 
    self.amcl_pose_callback, 10)
```

**Lý do chọn `/amcl_pose`:**
- ✅ Topic này có sẵn trong hệ thống (đã verify với `ros2 topic list`)
- ✅ Cung cấp pose trong **map frame** (toàn cục)
- ✅ Được localized bởi AMCL → **chính xác hơn raw odometry**
- ✅ Phù hợp với trajectory tracking (trajectory cũng trong map frame)

---

### 2. **Đổi Tên Callback Function**

#### ❌ **Trước:**
```python
def odom_callback(self, msg):
    """Update robot pose"""
    # ... code xử lý Odometry message
```

#### ✅ **Sau:**
```python
def amcl_pose_callback(self, msg):
    """Update robot pose from AMCL localization
    
    AMCL provides pose estimation in the map frame, which is more
    accurate than raw odometry as it corrects for drift using
    particle filter localization.
    """
    self.robot_x = msg.pose.pose.position.x
    self.robot_y = msg.pose.pose.position.y
    orientation_q = msg.pose.pose.orientation
    _, _, self.robot_theta = quaternion_to_euler(
        orientation_q.x, orientation_q.y, 
        orientation_q.z, orientation_q.w
    )
```

**Thay đổi:**
- ✅ Tên function rõ ràng hơn: `amcl_pose_callback`
- ✅ Docstring giải thích tại sao dùng AMCL
- ✅ Xử lý đúng structure của `PoseWithCovarianceStamped`:
  - `msg.pose.pose.position` (nested structure)
  - `msg.pose.pose.orientation`

---

### 3. **Cập Nhật Import Statements**

#### ❌ **Trước:**
```python
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import TwistStamped
```

#### ✅ **Sau:**
```python
from nav_msgs.msg import Path
from geometry_msgs.msg import TwistStamped, PoseWithCovarianceStamped
```

**Thay đổi:**
- ✅ Xóa `Odometry` (không dùng)
- ✅ Thêm `PoseWithCovarianceStamped`
- ✅ Giữ code gọn gàng, chỉ import những gì cần thiết

---

## 🏗️ Kiến Trúc Hệ Thống

### **ROS2 Topics**

```
┌─────────────────────────────────────────────────┐
│          Fuzzy Trajectory Controller            │
│                                                  │
│  Inputs:                                         │
│    📥 /amcl_pose (PoseWithCovarianceStamped)    │
│       └─> Robot position in map frame           │
│    📥 /trajectory (Path)                         │
│       └─> Reference trajectory to follow        │
│                                                  │
│  Output:                                         │
│    📤 /diff_cont/cmd_vel (TwistStamped)         │
│       └─> Velocity commands for robot           │
└─────────────────────────────────────────────────┘
```

### **Data Flow**

```
AMCL Localization ──┐
                    │
                    ├──> Fuzzy Controller ──> Velocity Commands
                    │         ↓                      ↓
Reference Path ─────┘    Error Calculation    Differential Drive
                                                   Robot
```

---

## 🧮 Fuzzy Logic Implementation

### **Inputs**

1. **e_D (Distance Error)**: Khoảng cách từ robot đến trajectory (meters)
   - Range: [-2.0, 2.0] m
   - Membership Functions: NB, NM, NS, ZE, PS, PM, PB

2. **e_Theta (Angle Error)**: Chênh lệch góc giữa robot và trajectory (degrees)
   - Range: [-180, 180] deg
   - Membership Functions: NB, NM, NS, ZE, PS, PM, PB

### **Outputs**

1. **VL (Left Wheel Velocity)**: m/s
2. **VR (Right Wheel Velocity)**: m/s
   - Range: [-0.5, 0.5] m/s
   - Membership Functions: NB, NM, NS, ZE, PS, PM, PB

### **Membership Functions**

- **Triangular (trimf)**: Cho các giá trị trung gian
- **Trapezoidal (trapmf)**: Cho các giá trị biên (NB, PB)

### **Fuzzy Rules**

Tổng cộng: **49 rules** (7x7 rule table)

Ví dụ:
```
IF e_D is NB AND e_Theta is ZE THEN VL is PB AND VR is PB
IF e_D is ZE AND e_Theta is ZE THEN VL is PS AND VR is PS
```

### **Inference Method**

- **Fuzzification**: Tính membership degrees
- **Rule Evaluation**: Min (AND operation)
- **Aggregation**: Max (combine rules)
- **Defuzzification**: Weighted average (centroid method)

---

## 📊 So Sánh: AMCL Pose vs Raw Odometry

| Tiêu chí | Raw Odometry | AMCL Pose |
|----------|--------------|-----------|
| **Frame** | `odom` (local) | `map` (global) |
| **Độ chính xác** | Bị drift theo thời gian | Corrected bởi particle filter |
| **Phù hợp cho** | Short-term motion | Long-term navigation |
| **Độ trễ** | Thấp | Trung bình (do localization) |
| **Trajectory tracking** | ❌ Không phù hợp | ✅ **Phù hợp** |

**Lý do chọn AMCL:**
- Trajectory thường được định nghĩa trong **map frame**
- AMCL loại bỏ **cumulative drift** của odometry
- Chính xác hơn cho **long-distance tracking**

---

## 🔄 Message Structure

### **PoseWithCovarianceStamped**
```python
# /amcl_pose message structure
msg.header.stamp              # timestamp
msg.header.frame_id           # "map"
msg.pose.pose.position.x      # robot x in map frame
msg.pose.pose.position.y      # robot y in map frame
msg.pose.pose.orientation.w   # quaternion w
msg.pose.pose.orientation.x   # quaternion x
msg.pose.pose.orientation.y   # quaternion y
msg.pose.pose.orientation.z   # quaternion z
msg.pose.covariance[36]       # 6x6 covariance matrix
```

### **TwistStamped** (Output)
```python
cmd_vel.header.stamp          # timestamp
cmd_vel.header.frame_id       # "base_link"
cmd_vel.twist.linear.x        # forward velocity (m/s)
cmd_vel.twist.angular.z       # rotation velocity (rad/s)
```

---

## 🚀 Usage

### **Build Package**
```bash
cd ~/ros2_ws
source install/setup.bash
colcon build --packages-select agv_trajectory_tracking
```

### **Run Controller**
```bash
source install/setup.bash
ros2 run agv_trajectory_tracking fuzzy_trajectory_controller
```

### **Expected Output**
```
[INFO] [fuzzy_trajectory_controller]: Initializing fuzzy system...
[INFO] [fuzzy_trajectory_controller]: Created 49 fuzzy rules
[INFO] [fuzzy_trajectory_controller]: Fuzzy system ready!
[INFO] [fuzzy_trajectory_controller]: Controller started (Pure Python Fuzzy)!
```

### **Monitor Topics**
```bash
# Check if controller is receiving pose
ros2 topic echo /amcl_pose

# Check if controller is publishing commands
ros2 topic echo /diff_cont/cmd_vel

# Publish test trajectory
ros2 topic pub /trajectory nav_msgs/msg/Path "{...}"
```

---

## ⚙️ Parameters

```python
wheel_base: 0.46              # Distance between wheels (m)
max_linear_vel: 1.0           # Max forward velocity (m/s)
max_angular_vel: 1.0          # Max rotation velocity (rad/s)
control_frequency: 20.0       # Control loop frequency (Hz)
goal_tolerance: 0.15          # Goal reached threshold (m)
```

**Cách thay đổi parameters:**
```bash
ros2 run agv_trajectory_tracking fuzzy_trajectory_controller \
  --ros-args \
  -p wheel_base:=0.5 \
  -p max_linear_vel:=0.8 \
  -p control_frequency:=30.0
```

---

## 🐛 Troubleshooting

### **Problem: Controller không nhận được pose**
```bash
# Kiểm tra topic có tồn tại không
ros2 topic list | grep amcl_pose

# Kiểm tra message type
ros2 topic info /amcl_pose

# Echo để xem data
ros2 topic echo /amcl_pose
```

### **Problem: Robot không di chuyển**
```bash
# Kiểm tra cmd_vel có được publish không
ros2 topic echo /diff_cont/cmd_vel

# Kiểm tra trajectory có được nhận không
ros2 topic echo /trajectory
```

### **Problem: Build failed**
```bash
# Clean build
rm -rf build/agv_trajectory_tracking install/agv_trajectory_tracking

# Rebuild
colcon build --packages-select agv_trajectory_tracking
```

---

## 📝 Code Quality

### ✅ **Improvements Made**

1. **Type Safety**: Correct message types for all topics
2. **Documentation**: Clear docstrings explaining AMCL choice
3. **Naming**: Function names reflect actual purpose
4. **Imports**: Only import what's needed
5. **Pure Python**: No external fuzzy library dependencies

### ✅ **Best Practices Followed**

- ✅ ROS2 naming conventions
- ✅ Proper frame_id in published messages
- ✅ Throttled logging (1 Hz) to avoid spam
- ✅ Parameter-based configuration
- ✅ Graceful shutdown (KeyboardInterrupt handling)

---

## 🔬 Testing Checklist

- [ ] Node starts without errors
- [ ] Subscribes to `/amcl_pose` successfully
- [ ] Receives trajectory from `/trajectory`
- [ ] Publishes velocity commands to `/diff_cont/cmd_vel`
- [ ] Fuzzy inference produces reasonable outputs
- [ ] Robot follows trajectory in simulation
- [ ] Goal detection works correctly
- [ ] Controller stops at goal

---

## 📚 References

### **ROS2 Message Types**
- `geometry_msgs/PoseWithCovarianceStamped`
- `geometry_msgs/TwistStamped`
- `nav_msgs/Path`

### **Topics Used**
- `/amcl_pose` - Robot localization (input)
- `/trajectory` - Reference path (input)
- `/diff_cont/cmd_vel` - Velocity commands (output)

### **Coordinate Frames**
- `map` - Global fixed frame
- `odom` - Local odometry frame
- `base_footprint` - Robot base frame
- `base_link` - Robot center frame

---

## 📅 Change Log

### **Version 1.0.1 - October 26, 2025**

**FIXED:**
- ✅ Corrected message type: `Odometry` → `PoseWithCovarianceStamped`
- ✅ Renamed callback: `odom_callback` → `amcl_pose_callback`
- ✅ Updated imports to match actual usage
- ✅ Added detailed documentation

**REASON:**
- Topic `/amcl_pose` uses `PoseWithCovarianceStamped`, not `Odometry`
- Previous code had type mismatch → no data received
- Now correctly subscribes to AMCL localization

---

## 👨‍💻 Author Notes

**Tại sao không dùng `/diff_cont/odom`?**

Khi kiểm tra `ros2 topic list`, không thấy topic `/diff_cont/odom` trong hệ thống. Các topics có sẵn liên quan đến localization:
- ✅ `/amcl_pose` - AMCL localization output
- ❌ `/diff_cont/odom` - Không tồn tại

Do đó, quyết định sử dụng `/amcl_pose` là lựa chọn đúng đắn và phù hợp với hệ thống hiện tại.

**Tại sao KHÔNG dùng thư viện fuzzy?**

1. **Dependency issues**: scikit-fuzzy có vấn đề với NumPy version
2. **Simplicity**: Pure Python dễ debug và maintain hơn
3. **Performance**: Custom implementation nhanh cho 49 rules
4. **Learning**: Hiểu rõ cách fuzzy logic hoạt động
5. **Portability**: Không cần install external packages

---

## 🎓 Kết Luận

Bộ điều khiển fuzzy trajectory tracking đã được triển khai thành công với:

✅ **Pure Python implementation** (không dùng thư viện ngoài)  
✅ **Correct topic subscription** (PoseWithCovarianceStamped)  
✅ **AMCL localization integration** (accurate pose estimation)  
✅ **49 fuzzy rules** (7x7 rule table)  
✅ **Differential drive kinematics** (wheel velocities → twist)  
✅ **ROS2 Humble compatibility**  

Controller sẵn sàng để test với robot thực tế! 🚀
