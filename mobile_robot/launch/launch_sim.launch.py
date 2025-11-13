import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os
import xacro
import os
import xacro


def launch_setup(context, *args, **kwargs):
    package_name = 'mobile_robot'
    use_sim_time_str = context.launch_configurations['use_sim_time']
    use_sim_time = use_sim_time_str.lower() == 'true'  # Convert string to boolean
    init_height = context.launch_configurations['height']
    init_x = context.launch_configurations['x_pos']
    init_y = context.launch_configurations['y_pos']
    world_name = context.launch_configurations['world']
    launch_localization_str = context.launch_configurations['launch_localization']
    launch_localization = launch_localization_str.lower() == 'true'
    
    pkg_path = os.path.join(get_package_share_directory(package_name))
    xacro_file = os.path.join(pkg_path, 'urdf', 'mobile_robot.urdf.xacro')
    
    # ✅ Set Gazebo model path to find AWS RoboMaker models
    models_path = os.path.join(pkg_path, 'models')
    os.environ['GZ_SIM_RESOURCE_PATH'] = models_path + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    os.environ['IGN_GAZEBO_RESOURCE_PATH'] = models_path + ':' + os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    
    # ✅ SỬA: Sử dụng worlds trong package
    world_file = os.path.join(pkg_path, 'worlds', world_name)
    
    # 💡 Debug: Print paths for verification
    print(f"✅ Package path: {pkg_path}")
    print(f"✅ World file: {world_file}")
    print(f"✅ World file exists: {os.path.exists(world_file)}")
    
    # 🔥 PROPER XACRO RENDERING: Convert XACRO to URDF XML
    robot_description = xacro.process_file(xacro_file, mappings={'sim_mode': 'true'}).toxml()
    
    # 💡 Debug: Print rendered URDF size for verification
    print(f"✅ URDF successfully rendered! Size: {len(robot_description)} characters")
    
    # Determine map name based on world file
    map_name = "small_house"
    if "warehouse" in world_name:
        map_name = "small_warehouse"
    
    print(f"✅ Spawn position: x={init_x}, y={init_y}")
    print(f"✅ Auto-launch localization: {launch_localization}")
    if launch_localization:
        print(f"✅ Map name: {map_name}")

    # RViz config file
    rviz_config_file = os.path.join(pkg_path, 'rviz', 'rviz2.rviz')

    # RViz2 node
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=["-d", rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Robot State Publisher with enhanced QoS for RViz2 compatibility
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
            'publish_frequency': 50.0,
            # 🔧 CRITICAL: QoS override for RViz2 compatibility
            'qos_overrides./robot_description.publisher.durability': 'transient_local',
            'frame_prefix': '',  # Prevent namespace prefixing
        }],
        output='screen'
    )

    # Gazebo spawn entity with x, y, z coordinates
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot', 
                   '-x', init_x, '-y', init_y, '-z', init_height],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Controllers with proper timing
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad", "--controller-manager", "/controller_manager"],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    diff_drive_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont", "--controller-manager", "/controller_manager"],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Bridge - CRITICAL: Clock bridge MUST be first for time sync
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/diff_cont/cmd_vel_unstamped@geometry_msgs/msg/Twist[gz.msgs.Twist',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            
            '/zed2/left/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/zed2/left/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/zed2/right/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/zed2/right/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Scan republisher - Fix frame_id issue
    scan_republisher = Node(
        package='mobile_robot',
        executable='scan_repub.py',
        name='scan_republisher',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Aruco detector
    aruco_detector = Node(
        package='agv_localization',
        executable='aruco_map_localizer',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Gazebo launch - IMPORTANT: Force simulation time and sync
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments=[
            ('gz_args', ['-r -v 4 ' + world_file]),
            ('use_sim_time', 'true'),
            ('sync', 'true'),  # Force time synchronization
        ]
    )


    return [
        # 🔥 CRITICAL ORDER: Gazebo first to be the clock source, then the bridge
        gazebo_launch,
        bridge,
        robot_state_publisher,
        rviz,
        gz_spawn_entity,
        scan_republisher,  # Add scan republisher
        aruco_detector,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[diff_drive_controller],
            )
        ),
    ]
    
    # ==========================================
    # OPTIONAL: Auto-launch Localization Stack
    # ==========================================
    if launch_localization:
        # Include global_localization.launch.py with matching spawn position
        localization_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('agv_localization'),
                    'launch',
                    'global_localization.launch.py'
                ])
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time_str,
                'x_pos': init_x,
                'y_pos': init_y,
                'yaw': '0.0',
                'map_name': map_name
            }.items()
        )
        
        # Add to nodes list
        nodes_to_launch = [
            gz_sim,
            bridge,
            robot_state_publisher,
            rviz,
            gz_spawn_entity,
            scan_republisher,
            localization_launch,  # Add localization
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=gz_spawn_entity,
                    on_exit=[joint_state_broadcaster],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster,
                    on_exit=[diff_drive_controller],
                )
            ),
        ]
    else:
        # Without localization
        nodes_to_launch = [
            gz_sim,
            bridge,
            robot_state_publisher,
            rviz,
            gz_spawn_entity,
            scan_republisher,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=gz_spawn_entity,
                    on_exit=[joint_state_broadcaster],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster,
                    on_exit=[diff_drive_controller],
                )
            ),
        ]
    
    return nodes_to_launch


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    x_pos_arg = DeclareLaunchArgument(
        'x_pos',
        default_value='5.0',
        description='Initial X position of robot in simulation'
    )

    y_pos_arg = DeclareLaunchArgument(
        'y_pos',
        default_value='-2.0',
        description='Initial Y position of robot in simulation'
    )

    height_arg = DeclareLaunchArgument(
        'height',
        default_value='0.2',
        description='Initial spawn height in simulation'
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start Gazebo GUI'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='small_house.world',
        description='World file to load. Options: small_house.world, small_warehouse.world, room_20x20.world',
        choices=['small_house.world', 'small_warehouse.world', 'room_20x20.world']
    )
    
    # Auto-launch localization (optional, can be disabled with launch_localization:=false)
    launch_localization_arg = DeclareLaunchArgument(
        'launch_localization',
        default_value='false',
        description='Auto-launch localization stack with matching spawn position'
    )

    return LaunchDescription([
        use_sim_time_arg,
        x_pos_arg,
        y_pos_arg,
        height_arg,
        gui_arg,
        world_arg,
        launch_localization_arg,
        OpaqueFunction(function=launch_setup),
    ])
