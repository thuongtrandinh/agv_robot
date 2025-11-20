#!/usr/bin/env python3
import os
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    # read Launch args early
    spawn_diff = context.launch_configurations.get('spawn_diff_controller', 'false').lower() in ['1','true','yes']
    use_robot_state_pub = context.launch_configurations.get('use_robot_state_publisher', 'true').lower() in ['1','true','yes']
    camera_dev = context.launch_configurations.get('camera_device', '/dev/video0')
    lidar_port = context.launch_configurations.get('lidar_serial_port', '/dev/ttyUSB1')
    lidar_baud = int(context.launch_configurations.get('lidar_serial_baudrate', '256000'))
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    pkg_robot = get_package_share_directory('mobile_robot')
    urdf_file = os.path.join(pkg_robot, 'urdf/robot_real_time.urdf.xacro')

    # load URDF
    robot_description = ""
    try:
        robot_description = xacro.process_file(urdf_file, mappings={'sim_mode': 'false'}).toxml()
    except Exception:
        robot_description = ""

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Single RPLIDAR node - configure lower published scan freq via scan_mode/frequency if supported
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': lidar_port,
            'serial_baudrate': lidar_baud,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
            # prefer low frequency to avoid overload; try 5-10 Hz
            'scan_mode': 'Standard',
            'frequency': 8.0
        }]
    )

    # Delay mapping and localization to let LIDAR and TF warm up
    mapping_node_delayed = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='agv_mapping_with_knowns_poses',
                executable='agv_mapping_with_knowns_poses',
                name='mapping_with_known_poses',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )

    localization_node_delayed = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='agv_localization',
                executable='odometry_motion_model',
                name='odometry_motion_model',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )

    return [
        robot_state_pub,
        rplidar_node,
        mapping_node_delayed,
        localization_node_delayed
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('lidar_serial_port', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('lidar_serial_baudrate', default_value='256000'),
        OpaqueFunction(function=launch_setup)
    ])
