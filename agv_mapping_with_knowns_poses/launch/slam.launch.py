import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    slam_config = LaunchConfiguration("slam_config", default=os.path.join(
        get_package_share_directory("agv_mapping_with_knowns_poses"),
        "config",
        "slam_toolbox.yaml"
    ))

    # Start map_saver_server immediately (optional)
    map_saver = Node(
        package="nav2_map_server",
        executable="map_saver_server",
        name="map_saver_server",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}]
    )

    # Delay SLAM so /scan and TF are stable
    slam_toolbox_delay = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="slam_toolbox",
                executable="sync_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[slam_config, {"use_sim_time": use_sim_time}],
                remappings=[("/scan", "/scan")]
            )
        ]
    )

    lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        output="screen",
        parameters=[{"node_names": ["map_saver_server"]}, {"use_sim_time": use_sim_time}, {"autostart": True}]
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "slam_config",
            default_value=os.path.join(
                get_package_share_directory("agv_mapping_with_knowns_poses"),
                "config",
                "slam_toolbox.yaml"
            )
        ),
        map_saver,
        slam_toolbox_delay,
        lifecycle
    ])
