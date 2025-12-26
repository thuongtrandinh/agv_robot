import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    map_name_str = "lab208b3"
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    
    init_x = float(context.launch_configurations.get('x_pos', '0'))
    init_y = float(context.launch_configurations.get('y_pos', '0'))
    init_yaw = float(context.launch_configurations.get('yaw', '0.0'))
    
    pkg_dir = get_package_share_directory("agv_localization")
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

    # ===========================
    # EKF LOCAL (odom frame fusion: IMU + wheel odometry)
    # ===========================
    ekf_local_config = os.path.join(pkg_dir, "config", "ekf_local.yaml")
    ekf_local_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_local_node",
        output="screen",
        parameters=[
            ekf_local_config,
            {"use_sim_time": use_sim_time}
        ],
        remappings=[
            ("/odometry/filtered", "/odometry/local")  # Output: /odometry/local
        ]
    )
    
    # ===========================
    # EKF GLOBAL (map frame fusion: local EKF + AMCL)
    # ===========================
    ekf_global_config = os.path.join(pkg_dir, "config", "ekf_global.yaml")
    ekf_global_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_global_node",
        output="screen",
        parameters=[
            ekf_global_config,
            {"use_sim_time": use_sim_time}
        ],
        remappings=[
            ("/odometry/filtered", "/odometry/global")  # Output: /odometry/global
        ]
    )
    
    # ===========================
    # Map Server
    # ===========================
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

    # ===========================
    # AMCL (delayed to ensure /scan and TF are ready)
    # ===========================
    amcl_delayed = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                output="screen",
                parameters=[
                    amcl_config_file,
                    {"use_sim_time": use_sim_time},
                ],
                remappings=[
                    ("/scan", "/scan"),
                    # AMCL sử dụng /odometry/local để estimate pose
                    ("/odom", "/odometry/local")
                ]
            )
        ]
    )

    # ===========================
    # Lifecycle Manager
    # ===========================
    lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {"node_names": ["map_server", "amcl"]},
            {"use_sim_time": use_sim_time},
            {"autostart": True}
        ],
    )

    # ===========================
    # ArUco Localizer (Global pose correction from known markers)
    # ===========================
    aruco_localizer = Node(
        package="agv_zed2",
        executable="aruco_localizer",
        name="aruco_localizer",
        output="screen",
        parameters=[
            {"map_frame": "map"},
            {"camera_frame_id": "zed2_left_camera_frame"},
            {"base_frame_id": "base_footprint"},
            {"marker_map_file": "aruco_map_positions.yaml"},
            {"position_variance": 0.0001},
            {"orientation_variance": 0.001},
            {"distance_scaling": 0.00005},
            {"publish_frequency": 10.0}
        ]
    )

    return [
        ekf_local_node,
        ekf_global_node,
        nav2_map_server,
        amcl_delayed,
        aruco_localizer,  # ArUco-based global localization
        lifecycle
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("x_pos", default_value="0"),
        DeclareLaunchArgument("y_pos", default_value="0"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        OpaqueFunction(function=launch_setup),
    ])