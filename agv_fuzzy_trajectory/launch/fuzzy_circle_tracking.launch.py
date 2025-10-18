from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Arguments for fuzzy controller
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
    
    # Arguments for circle trajectory
    radius_arg = DeclareLaunchArgument(
        'radius',
        default_value='2.0',
        description='Radius of the circle in meters'
    )
    
    center_x_arg = DeclareLaunchArgument(
        'center_x',
        default_value='2.0',
        description='X coordinate of circle center in meters'
    )
    
    center_y_arg = DeclareLaunchArgument(
        'center_y',
        default_value='2.0',
        description='Y coordinate of circle center in meters'
    )
    
    num_points_arg = DeclareLaunchArgument(
        'num_points',
        default_value='50',
        description='Number of waypoints in the circle'
    )

    # Fuzzy trajectory controller node
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

    # Circle trajectory publisher node
    circle_trajectory_node = Node(
        package='agv_fuzzy_trajectory',
        executable='circle_trajectory_publisher.py',
        name='circle_trajectory_publisher',
        output='screen',
        parameters=[{
            'radius': LaunchConfiguration('radius'),
            'center_x': LaunchConfiguration('center_x'),
            'center_y': LaunchConfiguration('center_y'),
            'num_points': LaunchConfiguration('num_points'),
            'publish_rate': 1.0,
        }]
    )

    return LaunchDescription([
        wheel_base_arg,
        max_linear_vel_arg,
        max_angular_vel_arg,
        control_frequency_arg,
        goal_tolerance_arg,
        radius_arg,
        center_x_arg,
        center_y_arg,
        num_points_arg,
        fuzzy_controller_node,
        circle_trajectory_node,
    ])
