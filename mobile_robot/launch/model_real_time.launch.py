#!/usr/bin/env python3
import os
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    # Launch arguments
    spawn_diff = context.launch_configurations.get('spawn_diff_controller', 'false').lower() in ['1','true','yes']
    publish_static = context.launch_configurations.get('publish_static_odom', 'false').lower() in ['1','true','yes']
    use_robot_state_pub = context.launch_configurations.get('use_robot_state_publisher', 'true').lower() in ['1','true','yes']
    camera_dev = context.launch_configurations.get('camera_device', '/dev/video0')
    lidar_port = context.launch_configurations.get('lidar_serial_port', '/dev/ttyUSB1')
    lidar_baud = int(context.launch_configurations.get('lidar_serial_baudrate', '256000'))
    run_rviz = context.launch_configurations.get('run_rviz', 'false').lower() in ['1','true','yes']
    
    use_sim_time = False

    # Paths
    pkg_robot = get_package_share_directory('mobile_robot')
    urdf_file = os.path.join(pkg_robot, 'urdf/robot_real_time.urdf.xacro')
    rviz_file = os.path.join(pkg_robot, 'rviz/rviz2.rviz')
    controller_yaml = os.path.join(pkg_robot, 'config/robot_controller.yaml')

    # Load URDF
    robot_description = xacro.process_file(urdf_file, mappings={'sim_mode': 'false'}).toxml()

    print(f"\n🚀 REAL ROBOT LAUNCH")
    print(f"URDF      : {urdf_file}")
    print(f"CTRL YAML : {controller_yaml}\n")

    # ============================
    # Core Nodes (start immediately)
    # ============================
    
    robot_state_pub = None
    if use_robot_state_pub:
        robot_state_pub = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
            output='screen'
        )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_yaml, {'robot_description': robot_description}],
        output='screen'
    )

    spawner_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad'],
        output='screen'
    )

    spawner_diff = None
    if spawn_diff:
        spawner_diff = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_cont'],
            output='screen'
        )

    static_odom_tf = None
    if publish_static:
        static_odom_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_odom_to_base',
            arguments=['0','0','0','0','0','0','odom','base_footprint'],
            output='screen'
        )

    # ============================
    # LIDAR (start first, publishes /scan_raw)
    # ============================
    lidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar',
        parameters=[
            {'serial_port': lidar_port},
            {'serial_baudrate': lidar_baud},
            {'frame_id': 'laser'},
            {'angle_compensate': True},
            {'topic_name': '/scan_raw'},  # Raw scan from LIDAR
            {'scan_frequency': 8.0}  # Reduce LIDAR frequency to 8Hz
        ],
        output='screen'
    )

    # ============================
    # Scan Throttle (subscribes /scan_raw, publishes /scan at 5Hz)
    # ============================
    scan_throttle = Node(
        package='mobile_robot',
        executable='scan_throttle.py',
        name='scan_throttle',
        output='screen'
    )

    # ============================
    # Motor Odometry (publishes /diff_cont/odom at reduced rate)
    # ============================
    motor_odom = Node(
        package='mobile_robot',
        executable='motor_odom',
        name='motor_odom',
        output='screen',
        parameters=[
            {'wheel_radius': 0.05},
            {'wheel_separation': 0.46},
            {'left_index': 1},
            {'right_index': 0},
            {'feedback_is_linear_velocity': True},
            {'imu_topic': '/imu'},
            {'motor_topic': '/motor_feedback'},
            {'odom_topic': '/diff_cont/odom'},
            {'odom_frame': 'odom'},
            {'base_frame': 'base_footprint'},
            {'odom_publish_rate': 15.0},  # Reduced from 20Hz to 15Hz
        ]
    )

    # ============================
    # ArUco Detector (delay to let TF stabilize)
    # ============================
    aruco_detector_delayed = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='agv_zed2',
                executable='aruco_detector',
                name='aruco_detector',
                output='screen',
                parameters=[
                    {'device': camera_dev},
                    {'camera_frame': 'zed2_left_camera_frame'},
                    {'calib_pkg': 'agv_zed2'},
                    {'calib_file': 'zed2_calibration_vga.yaml'},
                    {'marker_size': 0.173},
                ]
            )
        ]
    )

    # ============================
    # RViz (optional)
    # ============================
    rviz2 = None
    if run_rviz:
        rviz2 = Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_file],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        )

    # Build node list
    nodes = [controller_manager, spawner_jsb]
    
    if robot_state_pub:
        nodes.insert(0, robot_state_pub)
    if spawner_diff:
        nodes.append(spawner_diff)
    if static_odom_tf:
        nodes.insert(1, static_odom_tf)
    
    # Add sensor nodes (LIDAR first, then throttle, then motor_odom)
    nodes.extend([
        lidar_node,
        scan_throttle,
        motor_odom,
        aruco_detector_delayed
    ])
    
    if rviz2:
        nodes.append(rviz2)

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera_device', default_value='/dev/video0'),
        DeclareLaunchArgument('spawn_diff_controller', default_value='false'),
        DeclareLaunchArgument('publish_static_odom', default_value='false'),
        DeclareLaunchArgument('use_robot_state_publisher', default_value='true'),
        DeclareLaunchArgument('run_rviz', default_value='false'),
        DeclareLaunchArgument('lidar_serial_port', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('lidar_serial_baudrate', default_value='256000'),
        OpaqueFunction(function=launch_setup)
    ])
