import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def launch_setup(context, *args, **kwargs):
    """Setup function to handle map-specific configurations"""
    
    map_name_str = context.launch_configurations['map_name']
    use_sim_time = LaunchConfiguration("use_sim_time")
    
    # Get spawn position from arguments
    init_x = float(context.launch_configurations['x_pos'])
    init_y = float(context.launch_configurations['y_pos'])
    init_yaw = float(context.launch_configurations['yaw'])
    
    lifecycle_nodes = ["map_server", "amcl"]
    
    pkg_dir = get_package_share_directory("agv_localization")
    
    # EKF config file for sensor fusion (IMU + Encoder Odometry)
    # Standard pipeline: [Encoder + IMU] → EKF → /odometry/filtered
    ekf_local_config_file = os.path.join(pkg_dir, "config", "ekf.yaml")
    
    # Select AMCL config based on map
    if map_name_str == "small_warehouse":
        amcl_config_file = os.path.join(pkg_dir, "config", "amcl_warehouse.yaml")
    else:  # small_house or default
        amcl_config_file = os.path.join(pkg_dir, "config", "amcl.yaml")
    
    print(f"✅ Loading map: {map_name_str}")
    print(f"✅ AMCL config: {amcl_config_file}")
    print(f"✅ Initial pose: x={init_x}, y={init_y}, yaw={init_yaw}")
    
    map_path = PathJoinSubstitution([
        get_package_share_directory("agv_mapping_with_knowns_poses"),
        "maps",
        map_name_str,
        "map.yaml"
    ])

    
    # Static transform to align map and odom frames at startup
    # This ensures LIDAR scan aligns with map initially
    static_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
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
        ],
        remappings=[
            # Subscribe to EKF filtered output (IMU + Encoder fusion)
            ("/odom", "/odometry/filtered"),
        ]
    )

    nav2_lifecycle_manager = Node(
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
    
    # ==========================================
    # SENSOR FUSION: Single EKF (Standard Pipeline)
    # ==========================================
    # Pipeline: [Encoder + IMU] → EKF → /odometry/filtered → AMCL → map
    # TF: map → odom → base_footprint
    
    # 1. IMU Republisher - Add covariance to Gazebo IMU data
    # TUNED for LOCALIZATION: Lower covariance for better accuracy
    imu_republisher = Node(
        package="agv_localization",
        executable="imu_republisher",
        name="imu_republisher",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"input_topic": "/imu"},
            {"output_topic": "/imu_with_covariance"},
            {"orientation_covariance": 0.02},           # Lower than SLAM
            {"angular_velocity_covariance": 0.03},      # Lower than SLAM
            {"linear_acceleration_covariance": 0.06}    # Lower than SLAM
        ]
    )
    
    # 2. Odometry Republisher - Add covariance to diff_drive_controller odometry
    # TUNED for LOCALIZATION: Lower covariance for better accuracy
    odom_republisher = Node(
        package="agv_localization",
        executable="odom_republisher",
        name="odom_republisher",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"input_topic": "/diff_cont/odom"},
            {"output_topic": "/diff_cont/odom_with_covariance"},
            {"pose_x_covariance": 0.002},               # Lower than SLAM
            {"pose_y_covariance": 0.002},               # Lower than SLAM
            {"pose_yaw_covariance": 0.02},              # Lower than SLAM
            {"twist_vx_covariance": 0.005},             # Lower than SLAM
            {"twist_vy_covariance": 0.005},             # Lower than SLAM
            {"twist_vyaw_covariance": 0.02}             # Lower than SLAM
        ]
    )
    
    # 3. EKF Node - Fuse IMU + Wheel Odometry
    # Publishes /odometry/filtered (odom → base_footprint transform)
    ekf_filter = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            ekf_local_config_file,
            {"use_sim_time": use_sim_time}
        ]
        # Default output: /odometry/filtered (no remapping needed)
    )

    return [
        static_transform,
        nav2_map_server,
        nav2_amcl,
        nav2_lifecycle_manager,
        # Sensor Fusion nodes (Standard Pipeline)
        imu_republisher,  # Add covariance to IMU
        odom_republisher, # Add covariance to encoder odom
        ekf_filter,       # EKF: Fuse IMU + Encoder → /odometry/filtered
    ]


def generate_launch_description():
    
    map_name_arg = DeclareLaunchArgument(
        "map_name",
        default_value="small_house",
        description="Map name to load. Options: small_house, small_warehouse, room_20x20",
        choices=["small_house", "small_warehouse", "room_20x20"]
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true"
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
        map_name_arg,
        use_sim_time_arg,
        x_pos_arg,
        y_pos_arg,
        yaw_arg,
        OpaqueFunction(function=launch_setup),
    ])