import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    """Setup function to handle map-specific configurations"""
    
    map_name_str = "lab208b3"  # Fixed to use the lab208b3 map
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    launch_rviz_str = context.launch_configurations.get('launch_rviz', 'true')
    launch_rviz = launch_rviz_str.lower() == 'true'
    
    # Get spawn position from arguments
    init_x = float(context.launch_configurations.get('x_pos', '0'))
    init_y = float(context.launch_configurations.get('y_pos', '0'))
    init_yaw = float(context.launch_configurations.get('yaw', '0.0'))
    
    pkg_dir = get_package_share_directory("agv_localization")
    mobile_robot_pkg = get_package_share_directory("mobile_robot")
    
    # Use the AMCL configuration for lab208b3
    amcl_config_file = os.path.join(pkg_dir, "config", "amcl.yaml")
    
    # RViz config file from mobile_robot package
    rviz_config_file = os.path.join(mobile_robot_pkg, 'rviz', 'rviz2.rviz')
    
    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=["-d", rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    print(f"✅ Loading map: {map_name_str}")
    print(f"✅ AMCL config: {amcl_config_file}")
    print(f"✅ Initial pose: x={init_x}, y={init_y}, yaw={init_yaw}")
    
    # 🔧 FIX: Use absolute path instead of PathJoinSubstitution
    map_pkg_dir = get_package_share_directory("agv_mapping_with_knowns_poses")
    map_path = os.path.join(map_pkg_dir, "maps", map_name_str, "lab208b3.yaml")
    
    print(f"✅ Map file: {map_path}")
    print(f"✅ Map exists: {os.path.exists(map_path)}")

    # EKF for sensor fusion (high frequency for smooth tracking)
    ekf_config_file = os.path.join(pkg_dir, "config", "ekf.yaml")
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            ekf_config_file,
            {"use_sim_time": use_sim_time}
        ],
    )
    
    nav2_map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {"yaml_filename": map_path},
            {"use_sim_time": use_sim_time}
        ],
    )

    # AMCL node - no delay needed, lifecycle manager handles activation
    nav2_amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        emulate_tty=True,
        parameters=[
            amcl_config_file,
            {"use_sim_time": use_sim_time},
            # Override initial pose from launch arguments
            {"set_initial_pose": True},
            {"initial_pose.x": init_x},
            {"initial_pose.y": init_y},
            {"initial_pose.z": 0.0},
            {"initial_pose.yaw": init_yaw},
        ]
        # No remapping needed - AMCL uses TF (odom->base_footprint) from EKF
    )

    # ArUco Localizer - Uses saved marker positions to correct robot pose
    aruco_localizer = Node(
        package="agv_zed2",
        executable="aruco_localizer",
        name="aruco_localizer",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"map_frame": "map"},
            {"camera_frame_id": "zed2_left_camera_frame"},
            {"marker_topic": "aruco/markers"},
            {"marker_map_file": "aruco_map_positions.yaml"},
            {"position_variance": 0.0001},
            {"orientation_variance": 0.001},
            {"distance_scaling": 0.00005}
        ],
        # Remap ArUco pose to AMCL's initialpose topic for automatic correction
        # remappings=[
        #     ("/aruco/pose_with_covariance", "/initialpose")
        # ]
    )

    lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {"node_names": ["map_server", "amcl"]},
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"bond_timeout": 30.0},  # Increase timeout to prevent warnings
            {"bond_respawn_max_duration": 30.0},
            {"attempt_respawn_reconnection": True}  # Auto-reconnect if needed
        ],
    )

    nodes_to_launch = [
        ekf_node,
        nav2_map_server,
        nav2_amcl,
        aruco_localizer,  # ArUco localization for drift correction
        lifecycle
    ]
    
    # Conditionally add RViz2
    if launch_rviz:
        nodes_to_launch.append(rviz_node)
    
    return nodes_to_launch


def generate_launch_description():
    
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false"
    )
    
    x_pos_arg = DeclareLaunchArgument(
        "x_pos",
        default_value="0",
        description="Initial X position of robot (must match spawn position!)"
    )
    
    y_pos_arg = DeclareLaunchArgument(
        "y_pos",
        default_value="0",
        description="Initial Y position of robot (must match spawn position!)"
    )
    
    yaw_arg = DeclareLaunchArgument(
        "yaw",
        default_value="0.0",
        description="Initial yaw orientation in radians"
    )
    
    launch_rviz_arg = DeclareLaunchArgument(
        "launch_rviz",
        default_value="true",
        description="Launch RViz2 for visualization"
    )

    return LaunchDescription([
        use_sim_time_arg,
        x_pos_arg,
        y_pos_arg,
        yaw_arg,
        launch_rviz_arg,
        OpaqueFunction(function=launch_setup),
    ])