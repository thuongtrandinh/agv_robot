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
                    {'step_len': 0.10},  # Smaller steps for tight 2x4m map
                    {'goal_sample_rate': 0.15},
                    {'search_radius': 0.5},  # Smaller search for compact space
                    {'enable_smoothing': True},
                    {'path_resolution': 0.08},  # Finer resolution for small map
                    {'robot_radius': 0.21},  # Robot width 38cm / 2 + margin
                    {'safety_margin': 0.03},  # Tighter margin for 2x4m space
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
                    # Velocity limits - conservative for small space
                    {'max_v': 0.35},  # Match trajectory tracking max velocity
                    {'max_w': 1.0},  # Moderate turning for tight corners
                    {'max_accel': 0.5},  # Smooth acceleration
                    {'max_decel': 1.5},  # Safe deceleration
                    {'max_angular_accel': 1.5},
                    {'control_rate': 10.0},  # Match system frequency
                    # Robot geometry
                    {'robot_radius': 0.21},  # Actual robot: width 38cm/2 + margin
                    # Tracking gains
                    {'track.k_p': 1.5},
                    {'track.k_d': 2.0},
                    {'track.k_w': 0.5},
                    # Braking parameters - shorter for small map
                    {'brake.start_dist': 0.5},  # Start braking at 0.5m
                    {'brake.safety_factor': 0.6},
                    # Goal region - tighter for precision
                    {'goal.capture_radius': 0.3},  # Smaller goal radius
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
