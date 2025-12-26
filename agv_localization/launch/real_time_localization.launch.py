import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    map_name_str = "lab208b3"
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    
    init_x = float(context.launch_configurations.geet('x_pos', '0'))
    init_y = float(context.launch_configurations.get('y_pos', '0'))
    init_yaw = float(context.launch_configurations.get('yaw', '0.0'))
    
    pkg_dir = get_package_share_directory("agv_localization")
    amcl_config_file = os.path.join(pkg_dir, "config", "amcl.yaml")
    ekf_config_file = os.path.join(pkg_dir, "config", "ekf.yaml")
    
    print(f"✅ Loading map: {map_name_str}")
    print(f"✅ AMCL config: {amcl_config_file}")
    print(f"✅ EKF config: {ekf_config_file}")
    print(f"✅ Initial pose: x={init_x}, y={init_y}, yaw={init_yaw}")
    
    map_path = PathJoinSubstitution([
        get_package_share_directory("agv_mapping_with_knowns_poses"),
        "maps",
        map_name_str,
        "lab208b3.yaml"
    ])

    # ===========================
    # EKF Node (Fuses IMU + Wheel Odometry)
    # Output: /odometry/filtered, TF: odom -> base_footprint
    # ===========================
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            ekf_config_file,
            {"use_sim_time": use_sim_time}
        ],
        remappings=[
            ("/odometry/filtered", "/odometry/filtered")
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
    # Provides global localization: map -> odom TF
    # ===========================
    amcl_delayed = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                output="screen",
                parameters=[
                    amcl_config_file,
                    {"use_sim_time": use_sim_time},
                    {
                        "initial_pose.x": init_x,
                        "initial_pose.y": init_y,
                        "initial_pose.yaw": init_yaw,
                    }
                ],
                remappings=[
                    ("/scan", "/scan"),
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

    return [
        ekf_node,
        nav2_map_server,
        amcl_delayed,
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