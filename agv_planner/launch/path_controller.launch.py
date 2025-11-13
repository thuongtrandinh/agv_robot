#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ==========================
    # Declare Launch Arguments
    # ==========================
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    # ==========================
    # Global Planner Node
    # ==========================
    global_planner = Node(
        package='agv_planner',
        executable='global_planner',
        name='agv_global_planner',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'max_iter': 3000},
            {'step_len': 0.5},
            {'goal_sample_rate': 0.1},
            {'search_radius': 2.0},
            {'enable_smoothing': True},
            {'path_resolution': 0.1},
        ]
    )

    # ==========================
    # Local Planner Node
    # ==========================
    local_planner = Node(
        package='agv_planner',
        executable='local_planner',
        name='agv_local_planner',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},

            # Velocity limits
            {'max_v': 1.0},
            {'max_w': 1.0},
            {'max_accel': 1.0},
            {'max_decel': 3.0},
            {'max_angular_accel': 3.0},
            {'control_rate': 20.0},

            # Robot geometry
            {'robot_radius': 0.35},

            # Tracking gains
            {'track.k_p': 1.8},
            {'track.k_d': 2.5},
            {'track.k_w': 0.4},

            # Braking parameters
            {'brake.start_dist': 2.0},
            {'brake.safety_factor': 0.6},

            # Goal region
            {'goal.capture_radius': 0.8},
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
