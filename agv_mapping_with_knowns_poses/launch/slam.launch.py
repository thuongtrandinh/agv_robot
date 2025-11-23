import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    slam_config = LaunchConfiguration("slam_config")

    ros_distro = os.environ.get("ROS_DISTRO", "humble")
    lifecycle_nodes = ["map_saver_server"]
    if ros_distro != "humble":
        lifecycle_nodes.append("slam_toolbox")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
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
    
    # EKF config
    ekf_config_file = os.path.join(
        get_package_share_directory("agv_localization"),
        "config",
        "ekf.yaml"
    )
    
    # IMU Republisher (reduced covariance for stability)
    imu_republisher = Node(
        package="agv_localization",
        executable="imu_republisher",
        name="imu_republisher",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"input_topic": "/imu"},
            {"output_topic": "/imu_with_covariance"},
            {"orientation_covariance": 0.08},
            {"angular_velocity_covariance": 0.12},
            {"linear_acceleration_covariance": 0.20}
        ]
    )
    
    # Odometry Republisher (reduced covariance for stability)
    odom_republisher = Node(
        package="agv_localization",
        executable="odom_republisher",
        name="odom_republisher",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"input_topic": "/diff_cont/odom"},
            {"output_topic": "/diff_cont/odom_with_covariance"},
            {"pose_x_covariance": 0.03},
            {"pose_y_covariance": 0.03},
            {"pose_yaw_covariance": 0.08},
            {"twist_vx_covariance": 0.03},
            {"twist_vy_covariance": 0.03},
            {"twist_vyaw_covariance": 0.08}
        ]
    )
    
    # EKF Node (high frequency to match stm32_odom rate)
    ekf_filter = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            ekf_config_file,
            {"use_sim_time": use_sim_time},
            {"frequency": 50.0}  # Match stm32_odom rate (~57Hz)
        ]
    )
    
    # Map Saver
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

    # SLAM Toolbox (delayed to ensure /scan and TF are stable)
    # Reduced delay since high-frequency odometry stabilizes faster
    slam_toolbox_delayed = TimerAction(
        period=5.0,  # 5 seconds sufficient with 50Hz odometry
        actions=[
            Node(
                package="slam_toolbox",
                executable="sync_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[
                    slam_config,
                    {"use_sim_time": use_sim_time}
                ],
                remappings=[
                    ("/odom", "/odometry/filtered"),
                    ("/scan", "/scan")  # Use throttled scan
                ]
            )
        ]
    )

    # ArUco Mapper
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

    # Lifecycle Manager
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

    return LaunchDescription([
        use_sim_time_arg,
        slam_config_arg,
        imu_republisher,
        odom_republisher,
        ekf_filter,
        nav2_map_saver,
        slam_toolbox_delayed,
        aruco_mapper,
        nav2_lifecycle_manager,
    ])
