from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare arguments
    wheel_base_arg = DeclareLaunchArgument(
        'wheel_base',
        default_value='0.35',
        description='Distance between wheels in meters'
    )
    
    max_linear_vel_arg = DeclareLaunchArgument(
        'max_linear_vel',
        default_value='0.5',
        description='Maximum linear velocity in m/s'
    )
    
    max_angular_vel_arg = DeclareLaunchArgument(
        'max_angular_vel',
        default_value='1.0',
        description='Maximum angular velocity in rad/s'
    )
    
    control_frequency_arg = DeclareLaunchArgument(
        'control_frequency',
        default_value='20.0',
        description='Control loop frequency in Hz'
    )
    
    goal_tolerance_arg = DeclareLaunchArgument(
        'goal_tolerance',
        default_value='0.1',
        description='Goal tolerance in meters'
    )

    # Create fuzzy trajectory controller node (Python)
    fuzzy_controller_node = Node(
        package='agv_fuzzy_trajectory',
        executable='fuzzy_trajectory_controller.py',
        name='fuzzy_trajectory_controller',
        output='screen',
        parameters=[{
            'wheel_base': LaunchConfiguration('wheel_base'),
            'max_linear_vel': LaunchConfiguration('max_linear_vel'),
            'max_angular_vel': LaunchConfiguration('max_angular_vel'),
            'control_frequency': LaunchConfiguration('control_frequency'),
            'goal_tolerance': LaunchConfiguration('goal_tolerance'),
        }]
    )

    return LaunchDescription([
        wheel_base_arg,
        max_linear_vel_arg,
        max_angular_vel_arg,
        control_frequency_arg,
        goal_tolerance_arg,
        fuzzy_controller_node,
    ])
