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
        default_value='false',  # Set to false for real-time operation
        description='Use simulation clock'
    )

    # ==========================
    # Global Planner Node
    # ==========================
    global_planner = TimerAction(
        period=2.0,  # Delay to ensure topics and transforms are ready
        actions=[
            Node(
                package='agv_planner',
                executable='global_planner',
                name='agv_global_planner',
                output='screen',
                parameters=[
                    {'use_sim_time': LaunchConfiguration('use_sim_time')},
                    {'max_iter': 2000},
                    {'step_len': 0.15},
                    {'goal_sample_rate': 0.15},
                    {'search_radius': 1.0},
                    {'enable_smoothing': True},
                    {'path_resolution': 0.1},
                    {'robot_radius': 0.25},
                    {'safety_margin': 0.05},
                ]
            )
        ]
    )

    # ==========================
    # Local Planner Node
    # ==========================
    local_planner = TimerAction(
        period=4.0,  # Start after global planner for synchronization
        actions=[
            Node(
                package='agv_planner',
                executable='local_planner',
                name='agv_local_planner',
                output='screen',
                parameters=[
                    {'use_sim_time': LaunchConfiguration('use_sim_time')},
                    # Velocity limits
                    {'max_v': 0.8},
                    {'max_w': 0.8},
                    {'max_accel': 0.8},
                    {'max_decel': 2.0},
                    {'max_angular_accel': 2.0},
                    {'control_rate': 20.0},
                    # Robot geometry
                    {'robot_radius': 0.25},
                    # Tracking gains
                    {'track.k_p': 1.5},
                    {'track.k_d': 2.0},
                    {'track.k_w': 0.5},
                    # Braking parameters
                    {'brake.start_dist': 1.0},
                    {'brake.safety_factor': 0.6},
                    # Goal region
                    {'goal.capture_radius': 0.5},
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
