import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    # 1. Lấy giá trị từ CLI
    traj_type = context.launch_configurations['trajectory_type']
    radius_val = float(context.launch_configurations['radius'])
    center_x = context.launch_configurations['center_x']
    center_y = context.launch_configurations['center_y']
    
    # 2. CẤU HÌNH TỰ ĐỘNG
    # LƯU Ý: ĐÃ FIX TYPO TRONG LOGIC
    
    if traj_type == '1': # === CIRCLE (Dễ) ===
        print(f"🚀 MODE: CIRCLE (R={radius_val}m)")
        traj_speed = 0.35        
        ramp_time = 3.5          
        corner_scale = 1.0
        ctrl_max_lin = 0.40      
        ctrl_max_ang = 0.8       
        
    elif traj_type == '2': # === SQUARE (Trung bình) ===
        print(f"🚀 MODE: SQUARE (Side={radius_val}m)")
        traj_speed = 0.35        
        ramp_time = 6.0          
        corner_scale = 0.8       
        ctrl_max_lin = 0.3      
        ctrl_max_ang = 1.5       
        
    elif traj_type == '3': # === FIGURE-8 (Khó nhất) ===
        print(f"🚀 MODE: FIGURE-8 (Amplitude={radius_val/2}m)")
        traj_speed = 0.18        
        ramp_time = 5.0          
        corner_scale = 0.5       
        ctrl_max_lin = 0.28      
        ctrl_max_ang = 0.8       
        
    else: # Fallback
        traj_speed = 0.15
        ramp_time = 3.0
        corner_scale = 1.0
        ctrl_max_lin = 0.25
        ctrl_max_ang = 0.5

    # 3. Định nghĩa Nodes
    
    publisher_node = Node(
        package='agv_trajectory_tracking',
        executable='trajectory_publisher',
        name='trajectory_publisher',
        output='screen',
        parameters=[{
            'trajectory_type': int(traj_type),
            'center_x': float(center_x),
            'center_y': float(center_y),
            'radius': float(radius_val),
            'publish_rate': 20.0,
            'trajectory_speed': traj_speed,
            'ramp_time': ramp_time,
            'corner_speed_scale': corner_scale,
        }]
    )

    controller_node = Node(
        package='agv_trajectory_tracking',
        executable='fuzzy_trajectory_controller',
        name='fuzzy_trajectory_controller',
        output='screen',
        parameters=[{
            'wheel_base': 0.42,
            'control_frequency': 20.0, 
            'goal_tolerance': 0.08,
            'verbose_logging': True,
            'max_linear_vel': ctrl_max_lin,
            'max_angular_vel': ctrl_max_ang,
        }]
    )

    plotter_node = Node(
        package='agv_trajectory_tracking',
        executable='trajectory_plotter',
        name='trajectory_plotter',
        output='screen',
        parameters=[{'max_history': 20000}]
    )

    return [publisher_node, controller_node, plotter_node]

def generate_launch_description():
    return LaunchDescription([
        # ⚠️ FIX: Đặt tên chính xác là 'trajectory_type'
        DeclareLaunchArgument('trajectory_type', default_value='1'),
        DeclareLaunchArgument('radius', default_value='1.0'),
        DeclareLaunchArgument('center_x', default_value='0.95'),
        DeclareLaunchArgument('center_y', default_value='0.0'),
        
        OpaqueFunction(function=launch_setup)
    ])