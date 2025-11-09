# AGV Robot - DDMR

Robot AGV sử dụng hệ thống Differential Drive Mobile Robot (DDMR) với ROS2 Humble.

## Packages
- `mobile_robot`: Mô phỏng robot trong Gazebo
- `agv_controller`: Điều khiển robot bằng bàn phím
- `agv_localization`: Định vị và lọc dữ liệu sensor (AMCL + Map Server)
- `agv_mapping_with_knowns_poses`: Mapping với vị trí đã biết
- `agv_trajectory_tracking`: Điều khiển theo quỹ đạo với Fuzzy Logic Controller

## Cách sử dụng

### 1. Chạy simulation tự động (Khuyên dùng)

Script `run_sim.sh` sẽ tự động khởi chạy tất cả các thành phần cần thiết trong 3 terminal riêng biệt:
1. **Gazebo Simulation**: Mô phỏng robot và môi trường
2. **Global Localization**: Định vị robot với AMCL và Map Server
3. **Trajectory Tracking**: Điều khiển robot theo quỹ đạo với Fuzzy Controller

#### Cách chạy đơn giản:
```bash
cd /home/thuong/ros2_ws
./run_sim.sh
```

#### Cách chạy với tùy chọn:
```bash
# Spawn robot tại vị trí (5, -2) với quỹ đạo hình vuông
./run_sim.sh -x 5.0 -y -2.0 -t 2

# Spawn robot tại vị trí (0, 0) với quỹ đạo hình tròn bán kính 3m
./run_sim.sh -x 0.0 -y 0.0 -t 1 -r 3.0

# Sử dụng world khác
./run_sim.sh -w small_warehouse.world -x 10.0 -y 5.0

# Quỹ đạo hình số 8
./run_sim.sh -t 3 -x 0.0 -y 0.0
```

#### Các tùy chọn:
| Tùy chọn | Mô tả | Giá trị mặc định |
|----------|-------|------------------|
| `-x, --x-pos` | Vị trí spawn X của robot | 0.0 |
| `-y, --y-pos` | Vị trí spawn Y của robot | 0.0 |
| `-w, --world` | File world (small_house.world, small_warehouse.world, room_20x20.world) | small_house.world |
| `-t, --trajectory` | Loại quỹ đạo: 1=Tròn, 2=Vuông, 3=Số 8 | 2 |
| `-r, --radius` | Bán kính đường tròn (mét) | 2.0 |
| `-h, --help` | Hiển thị trợ giúp | - |

#### Sau khi chạy:
- 3 terminal sẽ tự động mở ra
- Gazebo sẽ khởi động với robot tại vị trí đã chỉ định
- AMCL sẽ tự động định vị robot
- Robot sẽ bắt đầu di chuyển theo quỹ đạo đã chọn
- Để visualize, chạy: `rviz2`

### 2. Dừng simulation

Script `kill_sim.sh` sẽ dừng **TẤT CẢ** các tiến trình liên quan đến simulation:

```bash
cd /home/thuong/ros2_ws
./kill_sim.sh
```

Script này sẽ dừng:
- ✅ Tất cả terminal windows đã mở
- ✅ Gazebo (gzserver, gzclient)
- ✅ Tất cả ROS2 launch files
- ✅ Tất cả ROS2 nodes (robot_state_publisher, controllers, AMCL, map_server, trajectory nodes...)
- ✅ RViz2 (nếu đang chạy)
- ✅ Matplotlib plotting windows
- ✅ Dọn dẹp shared memory
- ✅ Khởi động lại ROS2 daemon

#### Khi nào cần dùng:
- Sau khi xong việc với simulation
- Khi muốn restart simulation với cấu hình mới
- Khi gặp lỗi và cần reset toàn bộ hệ thống
- Trước khi tắt máy để tránh zombie processes

#### Lưu ý:
- Script sẽ **force kill** (-9) tất cả processes
- Nếu vẫn còn process chạy, script sẽ hiển thị cảnh báo
- Có thể cần chạy lại `./kill_sim.sh` nếu lần đầu không dừng hết

### 3. Chạy từng thành phần riêng lẻ (Manual)

Nếu muốn chạy từng phần riêng biệt:

#### Chỉ chạy Gazebo:
```bash
ros2 launch mobile_robot launch_sim.launch.py x_pos:=0.0 y_pos:=0.0 world:=small_house.world
```

#### Chỉ chạy Localization:
```bash
ros2 launch agv_localization global_localization.launch.py x_pos:=0.0 y_pos:=0.0 map_name:=small_house
```

#### Chỉ chạy Trajectory Tracking:
```bash
ros2 launch agv_trajectory_tracking trajectory_tracking.launch.py trajectory_type:=2 center_x:=0.0 center_y:=0.0 radius:=2.0
```

### 4. Điều khiển bằng bàn phím (Teleop)
```bash
ros2 launch agv_controller teleop.launch.py
```

## Sensors
- LIDAR: RPLidar A1
- IMU: 6-DOF
- UWB: Ultra-Wideband cho định vị indoor

## How to run rplidar
```bash
ros2 launch rplidar_ros rplidar_a2m8_launch.py serial_baudrate:=256000
```

## Troubleshooting

### Simulation không khởi động:
1. Chạy `./kill_sim.sh` để dọn dẹp
2. Chờ 5 giây
3. Chạy lại `./run_sim.sh`

### Robot không di chuyển:
- Kiểm tra AMCL đã localize chưa (xem trong RViz)
- Kiểm tra trajectory publisher node đã chạy chưa: `ros2 node list`
- Kiểm tra topic /cmd_vel: `ros2 topic echo /cmd_vel`

### Gazebo bị treo:
```bash
./kill_sim.sh
# Nếu vẫn chưa dừng:
killall -9 gzserver gzclient
```

### Cannot find map file:
- Đảm bảo file map tồn tại trong `src/agv_localization/maps/`
- Tên map phải khớp với world: small_house.world → small_house.yaml