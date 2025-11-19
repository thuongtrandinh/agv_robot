import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
    import xacro

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    # ============================
    #  PATHS
    # ============================
    pkg_robot = get_package_share_directory('mobile_robot')
    pkg_zed2  = get_package_share_directory('agv_zed2')
    pkg_mapping = get_package_share_directory('agv_mapping_with_knowns_poses')
    pkg_localization = get_package_share_directory('agv_localization')

    urdf_file = os.path.join(pkg_robot, 'urdf/robot_real_time.urdf.xacro')
    controller_yaml = os.path.join(pkg_robot, 'config/robot_controller.yaml')
    ekf_config_file = os.path.join(pkg_localization, 'config', 'ekf.yaml')
    slam_config_file = os.path.join(pkg_mapping, 'config', 'slam_toolbox.yaml')

    # ============================
    #  LOAD URDF
    # ============================
    robot_description = xacro.process_file(
        urdf_file,
        mappings={'sim_mode': 'false'}
    ).toxml()

    # 1) robot_state_publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # 2) controller_manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            controller_yaml,
            {'robot_description': robot_description}
        ],
        output='screen'
    )

    # 3) Joint State Broadcaster
    spawner_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad'],
        output='screen'
    )

    # 4) motor_odom
    motor_odom = Node(
        package='mobile_robot',
        executable='motor_odom',
        name='motor_odom',
        output='screen',
        parameters=[
            {'wheel_radius': 0.05},
            {'wheel_separation': 0.46},
            {'left_index': 1},
            {'right_index': 0},
            {'feedback_is_linear_velocity': True},
            {'imu_topic': '/imu'},
            {'motor_topic': '/motor_feedback'},
            {'odom_topic': '/diff_cont/odom'},
            {'odom_frame': 'odom'},
            {'base_frame': 'base_footprint'},
        ]
    )

    # 5) LIDAR
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar',
        parameters=[
            {'serial_port': '/dev/ttyUSB1'},
            {'serial_baudrate': 256000},
            {'frame_id': 'laser'},
            {'angle_compensate': True},
        ],
        output='screen'
    )

    # 6) ArUco Detector
    aruco_detector = Node(
        package='agv_zed2',
        executable='aruco_detector',
        name='aruco_detector',
        output='screen',
        parameters=[
            {'device': '/dev/video0'},
            {'camera_frame': 'zed2_left_camera_frame'},
            {'calib_pkg': 'agv_zed2'},
            {'calib_file': 'zed2_calibration_vga.yaml'},
            {'marker_size': 0.173},
        ]
    )

    # 7) IMU Republisher
    imu_republisher = Node(
        package="agv_localization",
        executable="imu_republisher",
        name="imu_republisher",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"input_topic": "/imu"},
            {"output_topic": "/imu_with_covariance"},
            {"orientation_covariance": 0.1},
            {"angular_velocity_covariance": 0.15},
            {"linear_acceleration_covariance": 0.25}
        ]
    )

    # 8) Odometry Republisher
    odom_republisher = Node(
        package="agv_localization",
        executable="odom_republisher",
        name="odom_republisher",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"input_topic": "/diff_cont/odom"},
            {"output_topic": "/diff_cont/odom_with_covariance"},
            {"pose_x_covariance": 0.05},
            {"pose_y_covariance": 0.05},
            {"pose_yaw_covariance": 0.1},
            {"twist_vx_covariance": 0.05},
            {"twist_vy_covariance": 0.05},
            {"twist_vyaw_covariance": 0.1}
        ]
    )

    # 9) EKF Node
    ekf_filter = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            ekf_config_file,
            {"use_sim_time": use_sim_time}
        ]
    )

    # 10) Mapping Node (SLAM Toolbox)
    slam_toolbox = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="slam_toolbox",
                executable="sync_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[
                    slam_config_file,
                    {"use_sim_time": use_sim_time},
                ],
                remappings=[
                    ("/odom", "/odometry/filtered"),
                ]
            )
        ]
    )

    # 11) ArUco Mapper
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
            {"marker_size": 0.173},
            {"max_marker_distance": 3.0},
            {"max_jump_distance": 0.30},
        ]
    )

    # 12) Map Saver
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

    # 13) Lifecycle Manager
    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        output="screen",
        parameters=[
            {"node_names": ["map_saver_server", "slam_toolbox"]},
            {"use_sim_time": use_sim_time},
            {"autostart": True}
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        robot_state_pub,
        controller_manager,
        spawner_jsb,
        motor_odom,
        rplidar_node,
        aruco_detector,
        imu_republisher,
        odom_republisher,
        ekf_filter,
        slam_toolbox,
        aruco_mapper,
        nav2_map_saver,
        nav2_lifecycle_manager
    ])