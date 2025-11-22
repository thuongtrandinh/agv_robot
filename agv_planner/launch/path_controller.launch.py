#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ==========================
    # Declare Launch Arguments
    # ==========================
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',  # Real-time operation (đồng bộ với workspace)
        description='Use simulation clock'
    )

    # ==========================
    # Global Planner Node (Delay để odometry và /scan ổn định)
    # ==========================
    global_planner = TimerAction(
        period=5.0,  # Wait 5s for odometry and scan to be ready
        actions=[
            Node(
                package='agv_planner',
                executable='global_planner',
                name='agv_global_planner',
                output='screen',
                parameters=[
                    {'use_sim_time': LaunchConfiguration('use_sim_time')},
                    {'max_iter': 2000},
                    {'step_len': 0.10},
                    {'goal_sample_rate': 0.15},
                    {'search_radius': 0.5},
                    {'enable_smoothing': True},
                    {'path_resolution': 0.08},
                    {'robot_radius': 0.23},  # width 46cm/2
                    {'safety_margin': 0.05},
                    # Giảm tần số publish path nếu cần
                    {'publish_rate': 5.0}  # 5Hz thay vì real-time
                ]
            )
        ]
    )

    # ==========================
    # Local Planner Node (Delay sau global planner)
    # ==========================
    local_planner = TimerAction(
        period=7.0,  # Wait 7s to ensure global planner is ready
        actions=[
            Node(
                package='agv_planner',
                executable='local_planner',
                name='agv_local_planner',
                output='screen',
                parameters=[
                    {'use_sim_time': LaunchConfiguration('use_sim_time')},
                    # Velocity limits - giảm tốc độ cho an toàn
                    {'max_v': 0.4},
                    {'max_w': 0.8},  # Giảm tốc độ góc
                    {'max_accel': 0.4},
                    {'max_decel': 1.0},
                    {'max_angular_accel': 1.2},
                    {'control_rate': 10.0},  # 10Hz - đồng bộ với yêu cầu
                    # Robot geometry
                    {'robot_radius': 0.23},  # width 46cm/2
                    # Hardware delay compensation
                    {'hardware_delay': 0.15},  # STM32 PID processing delay (s)
                    {'enable_predictive': True},  # Enable predictive control
                    # Tracking gains - giảm để ổn định hơn
                    {'track.k_p': 1.2},
                    {'track.k_d': 1.5},
                    {'track.k_w': 0.4},
                    # Braking parameters
                    {'brake.start_dist': 0.6},
                    {'brake.safety_factor': 0.7},
                    # Goal region
                    {'goal.capture_radius': 0.25},
                    # Evasion parameters - giảm độ nhạy
                    {'evasion.clearance': 0.4},
                    {'evasion.lookahead': 0.8},
                    # Reduce sensor processing frequency
                    {'scan_throttle': 10.0}  # Process scan at 10Hz max
                ],
                remappings=[
                    # No remapping needed - subscribe to both /amcl_pose and /odometry/filtered directly
                    # Subscribe từ /scan throttled (5Hz)
                    ('/scan', '/scan'),
                    # Publish cmd_vel
                    ('/cmd_vel', '/diff_cont/cmd_vel')
                ]
            )
        ]
    )

    # ==========================
    # Return LaunchDescription
    # ==========================
    return LaunchDescription([
        use_sim_time_arg,
        global_planner,
        local_planner,
    ])
