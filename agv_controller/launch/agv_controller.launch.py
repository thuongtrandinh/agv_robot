from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='agv_controller',
            executable='agv_controller',
            name='keyboard_teleop',
            output='screen',
        )
    ])
