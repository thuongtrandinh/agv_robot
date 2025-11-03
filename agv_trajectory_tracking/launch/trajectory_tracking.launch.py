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
        default_value='1',
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
        default_value='1.0',
        description='Maximum linear velocity (m/s)'
    )
    
    max_angular_vel_arg = DeclareLaunchArgument(
        'max_angular_vel',
        default_value='1.0',
        description='Maximum angular velocity (rad/s)'
    )
    
    # Get launch configurations
    trajectory_type = LaunchConfiguration('trajectory_type')
    center_x = LaunchConfiguration('center_x')
    center_y = LaunchConfiguration('center_y')
    radius = LaunchConfiguration('radius')
    max_linear_vel = LaunchConfiguration('max_linear_vel')
    max_angular_vel = LaunchConfiguration('max_angular_vel')
    
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
            'publish_rate': 10.0,
            'path_points': 200,
            'preview_time': 20.0,
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
            'goal_tolerance': 0.15,
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
            'update_interval': 100,  # ms
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
        
        # Nodes
        trajectory_publisher_node,
        fuzzy_controller_node,
        trajectory_plotter_node,
    ])
