import os
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def launch_setup(context, *args, **kwargs):
	# Launch args
	use_sim_time = LaunchConfiguration("use_sim_time", default="false")
	map_name_str = context.launch_configurations.get('map_name', 'small_house')
	init_x = float(context.launch_configurations.get('x_pos', '0'))
	init_y = float(context.launch_configurations.get('y_pos', '0'))
	init_yaw = float(context.launch_configurations.get('yaw', '0.0'))

	# PATHS
	pkg_robot = get_package_share_directory('mobile_robot')
	pkg_zed2  = get_package_share_directory('agv_zed2')
	pkg_mapping = get_package_share_directory('agv_mapping_with_knowns_poses')
	pkg_localization = get_package_share_directory('agv_localization')

	urdf_file = os.path.join(pkg_robot, 'urdf/robot_real_time.urdf.xacro')
	controller_yaml = os.path.join(pkg_robot, 'config/robot_controller.yaml')
	ekf_config_file = os.path.join(pkg_localization, 'config', 'ekf.yaml')

	# LOAD URDF
	robot_description = xacro.process_file(
		urdf_file,
		mappings={'sim_mode': 'false'}
	).toxml()

	# robot_state_publisher
	robot_state_pub = Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		parameters=[
			{'robot_description': robot_description},
			{'use_sim_time': use_sim_time}
		],
		output='screen'
	)

	# controller_manager
	controller_manager = Node(
		package='controller_manager',
		executable='ros2_control_node',
		parameters=[
			controller_yaml,
			{'robot_description': robot_description}
		],
		output='screen'
	)

	# Joint State Broadcaster
	spawner_jsb = Node(
		package='controller_manager',
		executable='spawner',
		arguments=['joint_broad'],
		output='screen'
	)

	# motor_odom
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
		]
	)

	# LIDAR
	rplidar_node = Node(
		package='rplidar_ros',
		executable='rplidar_composition',
		name='rplidar',
		parameters=[
			{'serial_port': '/dev/ttyUSB1'},
			{'serial_baudrate': 256000},
			{'frame_id': 'laser'},
			{'angle_compensate': True},
		],
		output='screen'
	)

	# ArUco Detector
	aruco_detector = Node(
		package='agv_zed2',
		executable='aruco_detector',
		name='aruco_detector',
		output='screen',
		parameters=[
			{'device': '/dev/video0'},
			{'camera_frame': 'zed2_left_camera_frame'},
			{'calib_pkg': 'agv_zed2'},
			{'calib_file': 'zed2_calibration_vga.yaml'},
			{'marker_size': 0.173},
		]
	)

	# Static transform map->odom
	static_transform = Node(
		package="tf2_ros",
		executable="static_transform_publisher",
		name="map_to_odom_publisher",
		arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
		parameters=[{"use_sim_time": use_sim_time}],
	)

	# Map server
	map_path = PathJoinSubstitution([
		pkg_mapping,
		"maps",
		map_name_str,
		"map.yaml"
	])
	nav2_map_server = Node(
		package="nav2_map_server",
		executable="map_server",
		name="map_server",
		output="screen",
		parameters=[
			{"yaml_filename": map_path},
			{"use_sim_time": use_sim_time}
		],
	)

	# AMCL config
	if map_name_str == "small_warehouse":
		amcl_config_file = os.path.join(pkg_localization, "config", "amcl_warehouse.yaml")
	else:
		amcl_config_file = os.path.join(pkg_localization, "config", "amcl.yaml")

	nav2_amcl = Node(
		package="nav2_amcl",
		executable="amcl",
		name="amcl",
		output="screen",
		emulate_tty=True,
		parameters=[
			amcl_config_file,
			{"use_sim_time": use_sim_time},
			{"set_initial_pose": True},
			{"initial_pose.x": init_x},
			{"initial_pose.y": init_y},
			{"initial_pose.z": 0.0},
			{"initial_pose.yaw": init_yaw},
		]
	)

	nav2_lifecycle_manager = Node(
		package="nav2_lifecycle_manager",
		executable="lifecycle_manager",
		name="lifecycle_manager_localization",
		output="screen",
		parameters=[
			{"node_names": ["map_server", "amcl"]},
			{"use_sim_time": use_sim_time},
			{"autostart": True}
		],
	)

	# IMU Republisher
	imu_republisher = Node(
		package="agv_localization",
		executable="imu_republisher",
		name="imu_republisher",
		output="screen",
		parameters=[
			{"use_sim_time": use_sim_time},
			{"input_topic": "/imu"},
			{"output_topic": "/imu_with_covariance"},
			{"orientation_covariance": 0.01},
			{"angular_velocity_covariance": 0.01},
			{"linear_acceleration_covariance": 0.02}
		]
	)

	# Odometry Republisher
	odom_republisher = Node(
		package='agv_localization',
		executable='odom_republisher',
		name='odom_republisher',
		output='screen',
		parameters=[
			{'input_odom_topic': '/diff_cont/odom'},
			{'output_odom_topic': '/diff_cont/odom_with_covariance'},
		]
	)

	# EKF Node
	ekf_filter = Node(
		package="robot_localization",
		executable="ekf_node",
		name="ekf_filter_node",
		output="screen",
		parameters=[
			ekf_config_file,
			{"use_sim_time": use_sim_time}
		]
	)

	# ArUco Localizer
	aruco_localizer = Node(
		package="agv_zed2",
		executable="aruco_localizer",
		name="aruco_localizer",
		output="screen",
		parameters=[
			{"use_sim_time": use_sim_time},
			{"map_frame": "map"},
			{"camera_frame_id": "zed2_left_camera_frame"},
			{"marker_topic": "aruco/markers"},
			{"marker_map_file": "aruco_map_positions.yaml"},
			{"position_variance": 0.0001},
			{"orientation_variance": 0.001},
			{"distance_scaling": 0.00005},
		],
	)

	return [
		robot_state_pub,
		controller_manager,
		spawner_jsb,
		motor_odom,
		rplidar_node,
		aruco_detector,
		static_transform,
		nav2_map_server,
		nav2_amcl,
		nav2_lifecycle_manager,
		imu_republisher,
		odom_republisher,
		ekf_filter,
		aruco_localizer,
	]

def generate_launch_description():
	map_name_arg = DeclareLaunchArgument(
		"map_name",
		default_value="small_house",
		description="Map name to load. Options: small_house, small_warehouse, room_20x20",
		choices=["small_house", "small_warehouse", "room_20x20"]
	)

	use_sim_time_arg = DeclareLaunchArgument(
		"use_sim_time",
		default_value="false"
	)

	x_pos_arg = DeclareLaunchArgument(
		"x_pos",
		default_value="0",
		description="Initial X position of robot (must match spawn position!)"
	)

	y_pos_arg = DeclareLaunchArgument(
		"y_pos",
		default_value="0",
		description="Initial Y position of robot (must match spawn position!)"
	)

	yaw_arg = DeclareLaunchArgument(
		"yaw",
		default_value="0.0",
		description="Initial yaw orientation in radians"
	)

	return LaunchDescription([
		map_name_arg,
		use_sim_time_arg,
		x_pos_arg,
		y_pos_arg,
		yaw_arg,
		OpaqueFunction(function=launch_setup),
	])
