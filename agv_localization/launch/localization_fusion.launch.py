"""
Launch file for fused localization: AMCL + ArUco with EKF
Author: Thuong Tran Dinh
Date: October 27, 2025

Architecture:
1. AMCL: Provides continuous global localization using LiDAR + map
2. ArUco Detector: Provides absolute position corrections when markers are detected
3. EKF (robot_localization): Fuses AMCL + ArUco for optimal localization

TF Tree:
map -> [EKF fusion] -> base_footprint_ekf
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    pkg_dir = get_package_share_directory('agv_localization')
    
    # Launch arguments
    map_name = LaunchConfiguration('map_name')
    use_sim_time = LaunchConfiguration('use_sim_time')
    amcl_config = LaunchConfiguration('amcl_config')
    
    # AMCL lifecycle nodes
    lifecycle_nodes = ["map_server", "amcl"]
    
    # Map path
    map_path = PathJoinSubstitution([
        get_package_share_directory("agv_mapping_with_knowns_poses"),
        "maps",
        map_name,
        "map.yaml"
    ])
    
    # EKF config for fusion
    ekf_config_file = os.path.join(pkg_dir, 'config', 'ekf_map_fusion.yaml')
    
    return LaunchDescription([
        # ============================================
        # Launch Arguments
        # ============================================
        DeclareLaunchArgument(
            'map_name',
            default_value='small_house',
            description='Name of the map to load'),
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'),
        
        DeclareLaunchArgument(
            'amcl_config',
            default_value=os.path.join(pkg_dir, 'config', 'amcl.yaml'),
            description='Full path to AMCL yaml file to load'),
        
        # ============================================
        # 1. Map Server
        # ============================================
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {'yaml_filename': map_path},
                {'use_sim_time': use_sim_time}
            ],
        ),
        
        # ============================================
        # 2. AMCL - Adaptive Monte Carlo Localization
        # ============================================
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            emulate_tty=True,
            parameters=[
                amcl_config,
                {'use_sim_time': use_sim_time},
            ],
        ),
        
        # ============================================
        # 3. Lifecycle Manager for Map Server + AMCL
        # ============================================
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[
                {'node_names': lifecycle_nodes},
                {'use_sim_time': use_sim_time},
                {'autostart': True}
            ],
        ),
        
        # ============================================
        # 4. ArUco Detector - With debug logs
        # ============================================
        Node(
            package='agv_localization',
            executable='aruco_detector_sim',
            name='aruco_detector',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'marker_size': 0.173,
                'camera_frame_id': 'zed2_left_camera_frame',
                'image_topic': '/zed2/left/image_raw',
                'camera_info_topic': '/zed2/left/camera_info'
            }],
        ),
        
        # ============================================
        # 5. EKF Node - Fusion of AMCL + ArUco
        # ============================================
        # NOTE: robot_localization expects parameters under "ekf_filter_node" namespace
        # We need to remap the node name to match config file namespace
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',  # CHANGED: Match namespace in YAML file
            output='screen',
            parameters=[ekf_config_file, {'use_sim_time': use_sim_time}],
            remappings=[
                ('/odometry/filtered', '/odometry/map_fused'),  # Output fused pose
            ]
        ),
        
        # ============================================
        # 6. Fusion Monitor - Compare and visualize fusion quality
        # ============================================
        Node(
            package='agv_localization',
            executable='fusion_monitor.py',
            name='fusion_monitor',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
