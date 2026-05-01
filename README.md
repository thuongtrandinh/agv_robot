# ROS2 AGV Workspace - Package Overview

Goal: Summarize what each package in the `src` folder does for interview presentation.
Video demo: https://drive.google.com/drive/folders/1oOBvTOSlMDj5cZtJ1IEq_ZiKAU9VEebP?usp=sharing

## System overview
- AGV platform running on real hardware and simulation (Gazebo)
- Main sensors: LIDAR (RPLIDAR) and stereo camera ZED2 (ArUco)
- Localization: EKF (robot_localization) + AMCL (nav2)
- Mapping: SLAM Toolbox + map saver
- Experimental tracking: fuzzy trajectory tracking

## Package summary

| Package | Main role | Typical outputs / responsibilities |
|---|---|---|
| agv_bringup | Launch aggregation for real robot and simulation | Starts robot_state_publisher, controller, LIDAR, ZED2, EKF, AMCL, SLAM, RViz |
| agv_controller | Manual keyboard control | Publishes `cmd_vel` to `/diff_cont/cmd_vel` |
| agv_localization | Localization configuration (EKF + AMCL) | EKF fuses IMU + odom, AMCL publishes map->odom, map_server |
| agv_mapping_with_knowns_poses | Real-time mapping (SLAM) | SLAM Toolbox + map_saver_server, map configs |
| agv_planner | Global + local planning | Global path on `/global_path`, local planner outputs `cmd_vel` and avoids obstacles |
| agv_trajectory_tracking | Experimental fuzzy trajectory tracking | Trajectory publisher + fuzzy controller + plot/visualizer |
| agv_zed2 | ZED2 stereo and ArUco processing | ArUco detector + aruco mapper, marker pose |
| mobile_robot | Robot model, simulation, STM32 odom | URDF/xacro, Gazebo launch, controller, odom, scan republisher |
| rplidar_ros | LIDAR driver | Publishes `/scan` for RPLIDAR models |

## Package details (talking points)

### agv_bringup
- Purpose: One-click launch for real and simulated systems.
- Use cases: mapping, localization, full system bringup.
- Key launch files:
  - [agv_bringup/launch/real_robot.launch.py](agv_bringup/launch/real_robot.launch.py)
  - [agv_bringup/launch/simulated_robot.launch.py](agv_bringup/launch/simulated_robot.launch.py)
  - [agv_bringup/launch/mapping_realtime.launch.py](agv_bringup/launch/mapping_realtime.launch.py)
  - [agv_bringup/launch/localization_realtime.launch.py](agv_bringup/launch/localization_realtime.launch.py)

### agv_controller
- Purpose: Keyboard teleoperation using W/A/S/D.
- Output: TwistStamped on `/diff_cont/cmd_vel`.
- Main file: [agv_controller/src/KeyboardInput.cpp](agv_controller/src/KeyboardInput.cpp)

### agv_localization
- Purpose: Global and local localization with EKF + AMCL.
- Stack: `robot_localization` (EKF) + `nav2_amcl` + `nav2_map_server`.
- Key launch file:
  - [agv_localization/launch/real_time_localization.launch.py](agv_localization/launch/real_time_localization.launch.py)

### agv_mapping_with_knowns_poses
- Purpose: Real-time mapping using SLAM Toolbox and map saving.
- Talking point: EKF stabilizes odom before SLAM starts.
- Key launch file:
  - [agv_mapping_with_knowns_poses/launch/real_time_mapping.launch.py](agv_mapping_with_knowns_poses/launch/real_time_mapping.launch.py)

### agv_planner
- Global planner: builds a global path on the map with inflation and smoothing.
- Local planner: tracks the path, smooth braking, and obstacle avoidance from LaserScan.
- Nodes:
  - `agv_global_planner` (path from `/goal_pose` -> `/global_path`)
  - `agv_local_planner` (path + scan -> `cmd_vel`)
- Main files:
  - [agv_planner/agv_planner/global_planner.py](agv_planner/agv_planner/global_planner.py)
  - [agv_planner/agv_planner/local_planner.py](agv_planner/agv_planner/local_planner.py)

### agv_trajectory_tracking
- Purpose: Fuzzy tracking for benchmark trajectories (circle, square, figure-8).
- Components:
  - Trajectory publisher (reference path)
  - Fuzzy controller (trajectory tracking)
  - Plot/visualizer (real-time view)
- Key launch files:
  - [agv_trajectory_tracking/launch/trajectory_tracking.launch.py](agv_trajectory_tracking/launch/trajectory_tracking.launch.py)
  - [agv_trajectory_tracking/launch/real_time_trajectory_tracking.launch.py](agv_trajectory_tracking/launch/real_time_trajectory_tracking.launch.py)

### agv_zed2
- Purpose: ZED2 stereo processing and ArUco pose estimation.
- Main nodes: `aruco_detector` and `aruco_mapper`.
- Technical note: [agv_zed2/README_STEREO_60FPS.md](agv_zed2/README_STEREO_60FPS.md)

### mobile_robot
- Purpose: Robot model, Gazebo simulation, controller, STM32 odom pipeline.
- Main components:
  - URDF/xacro + robot_state_publisher
  - ros2_control + diff_drive_controller
  - `stm32_odom` and `scan_repub` (topic alignment for real hardware)
- Key launch files:
  - [mobile_robot/launch/launch_sim.launch.py](mobile_robot/launch/launch_sim.launch.py)
  - [mobile_robot/launch/model_real_time.launch.py](mobile_robot/launch/model_real_time.launch.py)

### rplidar_ros
- Purpose: RPLIDAR driver (A1/A2/A3/S1/S2/S3/T1/C1).
- More info: [rplidar_ros/README.md](rplidar_ros/README.md)

## Interview flow (system pipeline)
1. Sensor inputs: LIDAR -> `/scan`, IMU + odom -> EKF, ZED2 -> ArUco.
2. Localization: EKF for smooth odom, AMCL for map->odom, with map_server.
3. Mapping: SLAM Toolbox + map_saver, or localization on a prebuilt map.
4. Trajectory tracking: fuzzy controller for benchmark paths.
