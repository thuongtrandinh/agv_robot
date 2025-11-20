import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def launch_setup(context, *args, **kwargs):
    """Setup function to handle map-specific configurations"""
    
    map_name_str = "lab208b3"  # Fixed to use the lab208b3 map
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    
    # Get spawn position from arguments
    init_x = float(context.launch_configurations['x_pos'])
    init_y = float(context.launch_configurations['y_pos'])
    init_yaw = float(context.launch_configurations['yaw'])
    
    lifecycle_nodes = ["map_server", "amcl"]
    
    pkg_dir = get_package_share_directory("agv_localization")
    
    # EKF config file for sensor fusion (IMU + Encoder Odometry)
    ekf_local_config_file = os.path.join(pkg_dir, "config", "ekf.yaml")
    
    # Use the AMCL configuration for lab208b3
    amcl_config_file = os.path.join(pkg_dir, "config", "amcl.yaml")
    
    print(f"✅ Loading map: {map_name_str}")
    print(f"✅ AMCL config: {amcl_config_file}")
    print(f"✅ Initial pose: x={init_x}, y={init_y}, yaw={init_yaw}")
    
    map_path = PathJoinSubstitution([
        get_package_share_directory("agv_mapping_with_knowns_poses"),
        "maps",
        map_name_str,
        "lab208b3.yaml"
    ])

    # Static transform to align map and odom frames at startup
    static_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_publisher",
        arguments=["0", "0", "0", "0", "0", "3.14159", "map", "odom"],
        parameters=[{"use_sim_time": use_sim_time}],
    )
    
    # Static transform for laser frame to ensure transforms are available
    static_transform_laser = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="laser_to_base_link_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "laser", "base_link"],
        parameters=[{"use_sim_time": use_sim_time}],
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

    # Delay AMCL activation to ensure transforms are available
    delayed_amcl = TimerAction(
        period=5.0,  # Delay AMCL activation by 5 seconds
        actions=[
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                output="screen",
                emulate_tty=True,
                parameters=[
                    amcl_config_file,
                    {"use_sim_time": use_sim_time},
                    {"set_initial_pose": True},
                    {"initial_pose.x": init_x},
                    {"initial_pose.y": init_y},
                    {"initial_pose.z": 0.0},
                    {"initial_pose.yaw": init_yaw},
                ]
            )
        ]
    )

    # Delay lifecycle manager to ensure all nodes are ready
    delayed_lifecycle_manager = TimerAction(
        period=7.0,  # Delay lifecycle manager by 7 seconds
        actions=[
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization",
                output="screen",
                parameters=[
                    {"node_names": lifecycle_nodes},
                    {"use_sim_time": use_sim_time},
                    {"autostart": True}
                ],
            )
        ]
    )
    
    # ==========================================
    # SENSOR FUSION: Single EKF (Standard Pipeline)
    # ==========================================
    imu_republisher = Node(
        package="agv_localization",
        executable="imu_republisher",
        name="imu_republisher",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"input_topic": "/imu"},
            {"output_topic": "/imu_with_covariance"},
            {"orientation_covariance": 0.01},
            {"angular_velocity_covariance": 0.01},
            {"linear_acceleration_covariance": 0.02}
        ]
    )
    
    odom_republisher = Node(
        package='agv_localization',
        executable='odom_republisher',
        name='odom_republisher',
        output='screen',
        parameters=[
            {'input_odom_topic': '/diff_cont/odom'},
            {'output_odom_topic': '/diff_cont/odom_with_covariance'},
        ]
    )
    
    ekf_filter = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            ekf_local_config_file,
            {"use_sim_time": use_sim_time}
        ]
    )
    
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
            {"distance_scaling": 0.00005},
        ],
    )

    return [
        static_transform,
        static_transform_laser,  # Add static transform for laser frame
        nav2_map_server,
        delayed_amcl,  # Use delayed AMCL activation
        delayed_lifecycle_manager,  # Delay lifecycle manager
        imu_republisher,
        odom_republisher,
        ekf_filter,
        aruco_localizer,
    ]


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

    return LaunchDescription([
        use_sim_time_arg,
        x_pos_arg,
        y_pos_arg,
        yaw_arg,
        OpaqueFunction(function=launch_setup),
    ])