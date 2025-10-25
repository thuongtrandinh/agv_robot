import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node, LifecycleNode
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess

def generate_launch_description():
    # --- Declare Launch Arguments ---
    # Thay đổi đường dẫn này tới file map.yaml của bạn
    # Các tham số liên quan đến bản đồ và RViz đã được loại bỏ vì chúng được cung cấp bởi launch_sim.launch.py
    # hoặc không cần thiết.

    # --- Nodes ---
    # map_server là một LifecycleNode, cần được quản lý
    global_planner_node = Node(
        package='agv_planner',
        executable='global_planner',
        name='agv_global_planner',
        output='screen',
        parameters=[
            {'max_iter': 5000},
            {'step_len': 0.5},
            {'goal_sample_rate': 0.1},
            {'search_radius': 2.0}
        ]
    )

    return LaunchDescription([
        global_planner_node,
    ])