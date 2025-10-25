from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare arguments for circle parameters
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
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='1.0',
        description='Rate to publish trajectory in Hz'
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
            'publish_rate': LaunchConfiguration('publish_rate'),
        }]
    )

    return LaunchDescription([
        radius_arg,
        center_x_arg,
        center_y_arg,
        num_points_arg,
        publish_rate_arg,
        circle_trajectory_node,
    ])
