import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_config = LaunchConfiguration("slam_config")

    ros_distro = os.environ["ROS_DISTRO"]
    lifecycle_nodes = ["map_saver_server"]
    if ros_distro != "humble":
        lifecycle_nodes.append("slam_toolbox")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time if true"
    )

    slam_config_arg = DeclareLaunchArgument(
        "slam_config",
        default_value=os.path.join(
            get_package_share_directory("agv_mapping_with_knowns_poses"),
            "config",
            "slam_toolbox.yaml"
        ),
        description="Full path to slam yaml file to load"
    )
    
    # ==========================================
    # SENSOR FUSION: EKF for reducing odometry drift
    # ==========================================
    # Pipeline: [IMU + Encoder] → EKF → /odometry/filtered → SLAM Toolbox
    
    ekf_config_file = os.path.join(
        get_package_share_directory("agv_localization"),
        "config",
        "ekf.yaml"
    )
    
    # 1. IMU Republisher - Add covariance to Gazebo IMU
    # TUNED: High covariance to reduce jerky motion during SLAM
    imu_republisher = Node(
        package="agv_localization",
        executable="imu_republisher",
        name="imu_republisher",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"input_topic": "/imu"},
            {"output_topic": "/imu_with_covariance"},
            {"orientation_covariance": 0.1},            # Increased from 0.05
            {"angular_velocity_covariance": 0.15},      # Increased from 0.08
            {"linear_acceleration_covariance": 0.25}    # Increased from 0.15
        ]
    )
    
    # 2. Odometry Republisher - Add covariance to wheel odometry
    # TUNED: High covariance to reduce jerky motion during SLAM
    odom_republisher = Node(
        package="agv_localization",
        executable="odom_republisher",
        name="odom_republisher",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"input_topic": "/diff_cont/odom"},
            {"output_topic": "/diff_cont/odom_with_covariance"},
            {"pose_x_covariance": 0.05},                # Increased from 0.01
            {"pose_y_covariance": 0.05},                # Increased from 0.01
            {"pose_yaw_covariance": 0.1},               # Increased from 0.05
            {"twist_vx_covariance": 0.05},              # Increased from 0.01
            {"twist_vy_covariance": 0.05},              # Increased from 0.01
            {"twist_vyaw_covariance": 0.1}              # Increased from 0.05
        ]
    )
    
    # 3. EKF Node - Fuse IMU + Encoder to reduce drift
    ekf_filter = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            ekf_config_file,
            {"use_sim_time": use_sim_time}
        ]
        # Publishes /odometry/filtered (much better than raw encoder!)
    )
    
    # ==========================================
    # SLAM NODES
    # ==========================================
    
    nav2_map_saver = Node(
        package="nav2_map_server",
        executable="map_saver_server",
        name="map_saver_server",
        output="screen",
        parameters=[
            {"save_map_timeout": 5.0},
            {"use_sim_time": use_sim_time},
            {"free_thresh_default": 0.196},
            {"occupied_thresh_default": 0.65},
        ],
    )

    # Add a delay to ensure transforms and sensor data are initialized
    slam_toolbox = TimerAction(
        period=5.0,  # Delay for 5 seconds
        actions=[
            Node(
                package="slam_toolbox",
                executable="sync_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[
                    slam_config,
                    {"use_sim_time": use_sim_time},
                    {"queue_size": 50}  # Increase queue size to prevent message drops
                ],
                remappings=[
                    # CRITICAL: Use filtered odometry instead of raw /odom
                    ("/odom", "/odometry/filtered"),
                ]
            )
        ]
    )

    # ============================
    #  ArUco Mapper (from agv_zed2)
    #  Saves discovered markers to agv_mapping_with_knowns_poses/maps
    # ============================
    aruco_mapper = Node(
        package="agv_zed2",
        executable="aruco_mapper",
        name="aruco_mapper",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"map_frame": "map"},
            {"camera_frame_id": "zed2_left_camera_frame"},
            {"marker_topic": "aruco/markers"},
            {"marker_size": 0.18},
            {"max_marker_distance": 3.0},
            {"max_jump_distance": 0.30},
        ],
    )

    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes},
            {"use_sim_time": use_sim_time},
            {"autostart": True}
        ],
    )

    # Add a delay to AMCL initialization
    amcl_node = TimerAction(
        period=5.0,  # Delay for 5 seconds
        actions=[
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"initial_pose_x": 0.0},
                    {"initial_pose_y": 0.0},
                    {"initial_pose_a": 3.14159},  # Set RPY yaw to Pi
                ]
            )
        ]
    )

    # Ensure EKF and SLAM Toolbox are initialized first
    nodes = [
        use_sim_time_arg,
        slam_config_arg,
        imu_republisher,
        odom_republisher,
        ekf_filter,
        slam_toolbox,
        amcl_node,
        nav2_map_saver,
        aruco_mapper,
        nav2_lifecycle_manager,
    ]

    return LaunchDescription(nodes)
