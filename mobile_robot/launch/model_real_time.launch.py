#!/usr/bin/env python3
import os
import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):

    # read Launch args early (used to decide whether to spawn controller / static TF)
    spawn_diff = context.launch_configurations.get('spawn_diff_controller', 'true').lower() in ['1','true','yes']
    publish_static = context.launch_configurations.get('publish_static_odom', 'true').lower() in ['1','true','yes']

    # ============================
    #  PATHS (relative in package)
    # ============================
    pkg_robot = get_package_share_directory('mobile_robot')
    pkg_zed2  = get_package_share_directory('agv_zed2')

    urdf_file = os.path.join(pkg_robot, 'urdf/robot_real_time.urdf.xacro')
    rviz_file = os.path.join(pkg_robot, 'rviz/rviz2.rviz')
    controller_yaml = os.path.join(pkg_robot, 'config/robot_controller.yaml')

    # ============================
    #  LOAD URDF
    # ============================
    robot_description = xacro.process_file(
        urdf_file,
        mappings={'sim_mode': 'false'}
    ).toxml()

    print(f"\n🚀 REAL ROBOT LAUNCH")
    print(f"URDF      : {urdf_file}")
    print(f"CTRL YAML : {controller_yaml}\n")

    # ============================
    #  1) robot_state_publisher
    # ============================
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': False}
        ],
        output='screen'
    )

    # ============================
    #  2) controller_manager
    # ============================
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            controller_yaml,
            {'robot_description': robot_description}
        ],
        output='screen'
    )

    # ============================
    #  3) Spawner — Joint State Broadcaster
    # ============================
    spawner_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad'],
        output='screen'
    )

    # Conditionally spawn diff controller
    spawner_diff = None
    if spawn_diff:
        spawner_diff = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_cont'],
            output='screen'
        )

    # Optional static odom->base for standalone runs
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
    #  ZED2 ArUco + LIDAR + RViz
    # ============================
    aruco_detector = Node(
        package='agv_zed2',
        executable='aruco_detector',
        name='aruco_detector',
        output='screen',
        parameters=[
            {'device': '/dev/video4'},
            {'camera_frame': 'zed2_left_camera_frame'},
            {'calib_pkg': 'agv_zed2'},
            {'calib_file': 'zed2_calibration_vga.yaml'},
            {'marker_size': 0.173},
        ]
    )

    lidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar',
        parameters=[
            {'serial_port': '/dev/ttyUSB0'},
            {'frame_id': 'laser'},
            {'angle_compensate': True},
        ],
        output='screen'
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': False}],
        output='screen'
    )

    # build return list conditionally
    nodes = [
        robot_state_pub,
        controller_manager,
        spawner_jsb,
    ]
    if spawner_diff:
        nodes.append(spawner_diff)
    if static_odom_tf:
        nodes.insert(1, static_odom_tf)

    # sensors / visualisation
    nodes += [aruco_detector, lidar_node, rviz2]

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('spawn_diff_controller', default_value='true',
                               description='Spawn diff controller (set false to avoid publishing odom)'),
        DeclareLaunchArgument('publish_static_odom', default_value='true',
                               description='Publish static odom->base_footprint for standalone runs'),
        OpaqueFunction(function=launch_setup)
    ])
