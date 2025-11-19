#!/usr/bin/env python3
import os
import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):

    # read Launch args early (used to decide whether to spawn controller / static TF)
    # CHANGED: spawn_diff_controller default=false (motor_odom will provide /diff_cont/odom)
    spawn_diff = context.launch_configurations.get('spawn_diff_controller', 'false').lower() in ['1','true','yes']
    publish_static = context.launch_configurations.get('publish_static_odom', 'true').lower() in ['1','true','yes']
    use_robot_state_pub = context.launch_configurations.get('use_robot_state_publisher', 'true').lower() in ['1','true','yes']
    camera_dev = context.launch_configurations.get('camera_device', '/dev/video0')
    # lidar params (allow overriding the serial port and baudrate from the launch command)
    lidar_port = context.launch_configurations.get('lidar_serial_port', '/dev/ttyUSB1')
    lidar_baud = int(context.launch_configurations.get('lidar_serial_baudrate', '256000'))

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
    #  1) robot_state_publisher (optional)
    # ============================
    robot_state_pub = None
    if use_robot_state_pub:
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
    #  3b) Optionally spawn diff controller (default: false)
    #     motor_odom will provide /diff_cont/odom by default
    # ============================
    spawner_diff = None
    if spawn_diff:
        spawner_diff = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_cont'],
            output='screen'
        )

    # ============================
    #  1c) Optional static odom->base for standalone runs (default: false)
    # ============================
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
    #  ArUco Detector (only start if a usable /dev/video device is found)
    # ============================
    def probe_video_device(preferred_dev: str):
        # Try preferred device first, then test /dev/video0..3
        import os

        candidates = []
        if preferred_dev:
            candidates.append(preferred_dev)
        # add a few common device names
        for i in range(0, 4):
            dev = f"/dev/video{i}"
            if dev not in candidates:
                candidates.append(dev)

        for dev in candidates:
            try:
                if not os.path.exists(dev):
                    continue
                # try to open device file to ensure it's present and accessible
                fd = os.open(dev, os.O_RDWR | os.O_NONBLOCK)
                os.close(fd)
                return dev
            except Exception:
                continue
        return None

    selected_camera = probe_video_device(camera_dev)
    if selected_camera is None:
        aruco_detector = None
        print(f"[WARNING] No usable camera device found (tried {camera_dev} and /dev/video0-3). Skipping aruco_detector.")
    else:
        print(f"[INFO] Using camera device: {selected_camera} for aruco_detector")
        aruco_detector = Node(
            package='agv_zed2',
            executable='aruco_detector',
            name='aruco_detector',
            output='screen',
            parameters=[
                {'device': selected_camera},
                {'camera_frame': 'zed2_left_camera_frame'},
                {'calib_pkg': 'agv_zed2'},
                {'calib_file': 'zed2_calibration_vga.yaml'},
                {'marker_size': 0.18},
            ]
        )

    # ============================
    #  motor_odom: publishes /diff_cont/odom from /motor_feedback + /imu
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
            {'odom_topic': '/diff_cont/odom'},   # motor_odom is canonical odom source
            {'odom_frame': 'odom'},
            {'base_frame': 'base_footprint'},
        ]
    )

    # ============================
    #  LIDAR
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
        ],
        output='screen'
    )

    # ============================
    #  RViz (optional)
    # ============================
    rviz2 = None
    run_rviz = context.launch_configurations.get('run_rviz', 'false').lower() in ['1','true','yes']
    if run_rviz:
        rviz2 = Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_file],
            parameters=[{'use_sim_time': False}],
            output='screen'
        )

    # build return list conditionally
    nodes = [
        controller_manager,
        spawner_jsb,
    ]
    if robot_state_pub:
        # put robot_state_publisher first so TFs are available early
        nodes.insert(0, robot_state_pub)
    if spawner_diff:
        nodes.append(spawner_diff)
    if static_odom_tf:
        nodes.insert(1, static_odom_tf)

    # sensors / visualisation
    # append rviz2 only if created
    # Ensure odometry (TF) is available before LIDAR publishes to avoid TF-extrapolation
    sensors_nodes = [motor_odom, aruco_detector, lidar_node]
    if rviz2:
        sensors_nodes.append(rviz2)
    nodes += sensors_nodes

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera_device', default_value='/dev/video0',
                               description='Video device for ArUco camera (e.g. /dev/video0 or /dev/video1)'),
        DeclareLaunchArgument('spawn_diff_controller', default_value='false',
                               description='Spawn diff controller (default false; motor_odom provides odom)'),
        DeclareLaunchArgument('publish_static_odom', default_value='true',
                               description='Publish static odom->base_footprint (default true)'),
        DeclareLaunchArgument('use_robot_state_publisher', default_value='true',
                               description='Start robot_state_publisher from this launch (default false; set true if no external rsp present)'),
        DeclareLaunchArgument('run_rviz', default_value='false',
                               description='Launch RViz2 (default false). Set true to run RViz from this launch.'),
        DeclareLaunchArgument('lidar_serial_port', default_value='/dev/ttyUSB1',
                               description='Serial port for RPLIDAR (default /dev/ttyUSB1)'),
        DeclareLaunchArgument('lidar_serial_baudrate', default_value='256000',
                               description='Serial baudrate for RPLIDAR (default 256000)'),
        OpaqueFunction(function=launch_setup)
    ])
