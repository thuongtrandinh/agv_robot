#!/usr/bin/env python3
import os
import xacro

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):

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

    # ============================
    #  4) Spawner — Diff Drive Controller
    # ============================
    spawner_diff = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont'],
        output='screen'
    )

    # ============================
    #  5) ZED2 → ArUco Detector
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

    # ============================
    #  6) RPLIDAR
    # ============================
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

    # ============================
    #  7) RViz2
    # ============================
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': False}],
        output='screen'
    )

    return [
        robot_state_pub,
        controller_manager,
        spawner_jsb,
        spawner_diff,
        lidar_node,
        aruco_detector,
        rviz2,
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
