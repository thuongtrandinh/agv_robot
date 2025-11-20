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
    
    # Get spawn position from arguments
    init_x = float(context.launch_configurations.get('x_pos', '0'))
    init_y = float(context.launch_configurations.get('y_pos', '0'))
    init_yaw = float(context.launch_configurations.get('yaw', '0.0'))
    
    pkg_dir = get_package_share_directory("agv_localization")
    
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
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen"
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
    amcl_delayed = TimerAction(
        period=5.0,  # Delay AMCL activation by 5 seconds
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
                remappings=[("/scan", "/scan")]
            )
        ]
    )

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
        static_transform,
        nav2_map_server,
        amcl_delayed,
        lifecycle
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