#!/usr/bin/env python3
"""
Launch file for AGV Trajectory Tracking System

Launches:
1. trajectory_publisher - Generates reference trajectory
2. fuzzy_trajectory_controller - Controls robot to follow trajectory
3. trajectory_plotter - Real-time visualization with matplotlib

Author: Thuong Tran Dinh
Date: November 2, 2025
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    # Declare arguments
    trajectory_type_arg = DeclareLaunchArgument(
        'trajectory_type',
        default_value='2',
        description='Trajectory type: 1=Circle, 2=Square, 3=Figure-8'
    )
    
    center_x_arg = DeclareLaunchArgument(
        'center_x',
        default_value='5.0',
        description='X coordinate of trajectory center'
    )
    
    center_y_arg = DeclareLaunchArgument(
        'center_y',
        default_value='-2.0',
        description='Y coordinate of trajectory center'
    )
    
    radius_arg = DeclareLaunchArgument(
        'radius',
        default_value='5.0',
        description='Circle radius for trajectory (meters)'
    )
    
    max_linear_vel_arg = DeclareLaunchArgument(
        'max_linear_vel',
        default_value='0.4',  # Reduced for safer and smoother tracking
        description='Maximum robot linear velocity (m/s) - Optimal: 0.4'
    )
    
    max_angular_vel_arg = DeclareLaunchArgument(
        'max_angular_vel',
        default_value='1.5',  # Increased from 1.0 for sharper turns
        description='Maximum angular velocity (rad/s)'
    )
    
    trajectory_speed_arg = DeclareLaunchArgument(
        'trajectory_speed',
        default_value='0.3',  # m/s - Reduced for smoother tracking
        description='Trajectory reference speed (m/s) - Optimal: 0.3'
    )
    
    enable_traj_publish_arg = DeclareLaunchArgument(
        'enable_traj_publish',
        default_value='true',
        description='Enable/disable trajectory publishing'
    )
    
    verbose_logging_arg = DeclareLaunchArgument(
        'verbose_logging',
        default_value='true',
        description='Enable detailed velocity logging (v, omega, vL, vR)'
    )
    
    # Get launch configurations
    trajectory_type = LaunchConfiguration('trajectory_type')
    center_x = LaunchConfiguration('center_x')
    center_y = LaunchConfiguration('center_y')
    radius = LaunchConfiguration('radius')
    max_linear_vel = LaunchConfiguration('max_linear_vel')
    max_angular_vel = LaunchConfiguration('max_angular_vel')
    trajectory_speed = LaunchConfiguration('trajectory_speed')
    ramp_time = LaunchConfiguration('ramp_time')
    enable_traj_publish = LaunchConfiguration('enable_traj_publish')
    verbose_logging = LaunchConfiguration('verbose_logging')
    
    # Node 1: Trajectory Publisher
    trajectory_publisher_node = Node(
        package='agv_trajectory_tracking',
        executable='trajectory_publisher',
        name='trajectory_publisher',
        output='screen',
        parameters=[{
            'trajectory_type': trajectory_type,
            'center_x': center_x,
            'center_y': center_y,
            'radius': radius,
            'publish_rate': 20.0,  # FIXED at 20Hz for optimal real-time tracking
            'path_points': 200,
            'preview_time': 10.0,
            'enable_publish': enable_traj_publish,  # Control trajectory publishing
            'trajectory_speed': trajectory_speed,  # Configurable trajectory speed
            'ramp_time': ramp_time,  # Smooth speed ramping time
        }],
    )
    
    # Node 2: Fuzzy Trajectory Controller
    fuzzy_controller_node = Node(
        package='agv_trajectory_tracking',
        executable='fuzzy_trajectory_controller',
        name='fuzzy_trajectory_controller',
        output='screen',
        parameters=[{
            'wheel_base': 0.46,
            'max_linear_vel': max_linear_vel,
            'max_angular_vel': max_angular_vel,
            'control_frequency': 20.0,
            'goal_tolerance': 0.10,  # Reduced from 0.15 for tighter tracking
            'enable_path_publish': True,  # Always enable path publishing to controller
            'verbose_logging': verbose_logging,  # Control detailed velocity logging
        }],
    )
    
    # Node 3: Trajectory Plotter (Matplotlib visualization)
    trajectory_plotter_node = Node(
        package='agv_trajectory_tracking',
        executable='trajectory_plotter',
        name='trajectory_plotter',
        output='screen',
        parameters=[{
            'max_history': 2000,
            'update_interval': 25,  # ms - Fast real-time update (40 Hz) for smooth plotting
            'publish_rate': 20.0,  # Fixed to match trajectory publisher
        }],
    )
    
    return LaunchDescription([
        # Launch arguments
        trajectory_type_arg,
        center_x_arg,
        center_y_arg,
        radius_arg,
        max_linear_vel_arg,
        max_angular_vel_arg,
        trajectory_speed_arg,  # New: control trajectory speed
        ramp_time_arg,  # New: smooth speed ramping
        enable_traj_publish_arg,
        verbose_logging_arg,
        
        # Nodes
        trajectory_publisher_node,
        fuzzy_controller_node,
        trajectory_plotter_node,
    ])
