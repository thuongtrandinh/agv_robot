import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction, GroupAction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    slam_config = LaunchConfiguration("slam_config")

    # Config Files
    pkg_mapping = get_package_share_directory("agv_mapping_with_knowns_poses")
    pkg_localization = get_package_share_directory("agv_localization")
    
    slam_config_file = os.path.join(pkg_mapping, "config", "slam_toolbox.yaml")
    ekf_config_file = os.path.join(pkg_localization, "config", "ekf_local.yaml") # Dùng chung file ekf.yaml (Local)


    # 1. EKF Node (Cung cấp Odom mượt cho SLAM)
    # Không cần Republisher nữa vì STM32 đã có Covariance
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_local_node",
        output="screen",
        parameters=[
            ekf_config_file,
            {"use_sim_time": use_sim_time}
        ],
        remappings=[("/odometry/filtered", "/odometry/local")] # Remap cho thống nhất
    )

    # 2. SLAM Toolbox
    slam_node = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_config,
            {"use_sim_time": use_sim_time}
        ],
        remappings=[
            # SLAM Toolbox lắng nghe topic này để biết robot di chuyển
            ("/odom", "/odometry/local"), 
            ("/scan", "/scan")
        ]
    )

    # 3. ArUco Mapper & Lifecycle (Giữ nguyên)
    aruco_mapper = Node(
        package="agv_zed2", executable="aruco_mapper", name="aruco_mapper",
        parameters=[{"use_sim_time": use_sim_time}, {"map_frame": "map"}]
    )
    
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_slam", output="screen",
        parameters=[{"node_names": ["map_saver_server"]}, {"autostart": True}]
    )
    
    map_saver = Node(
        package="nav2_map_server", executable="map_saver_server",
        name="map_saver_server", output="screen",
        parameters=[{"save_map_timeout": 5.0}]
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("slam_config", default_value=slam_config_file),
        
        ekf_node,
        
        # Chờ EKF ổn định rồi mới chạy SLAM
        TimerAction(period=3.0, actions=[slam_node]),
        
        aruco_mapper,
        map_saver,
        lifecycle_manager
    ])