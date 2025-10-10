# AGV Robot - DDMR

Robot AGV sử dụng hệ thống Differential Drive Mobile Robot (DDMR) với ROS2 Humble.

## Packages
- `mobile_robot`: Mô phỏng robot trong Gazebo
- `agv_controller`: Điều khiển robot bằng bàn phím
- `agv_localization`: Định vị và lọc dữ liệu sensor
- `agv_mapping_with_knowns_poses`: Mapping với vị trí đã biết

## Cách sử dụng

### Chạy simulation
```bash
ros2 launch mobile_robot launch_sim.launch.py
```

### Điều khiển bằng bàn phím
```bash
ros2 launch agv_controller teleop.launch.py
```

## Sensors
- LIDAR: RPLidar A1
- IMU: 6-DOF
- UWB: Ultra-Wideband cho định vị indoor

## How to run rplidar
``` bash
ros2 launch rplidar_ros rplidar_a2m8_launch.py serial_baudrate:=256000
```