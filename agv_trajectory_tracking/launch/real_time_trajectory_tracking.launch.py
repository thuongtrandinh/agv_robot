import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    # 1. Lấy giá trị từ dòng lệnh
    traj_type = context.launch_configurations['trajectory_type']
    radius_val = float(context.launch_configurations['radius'])
    center_x = context.launch_configurations['center_x']
    center_y = context.launch_configurations['center_y']
    
    # 2. CẤU HÌNH TỰ ĐỘNG (TUNED PROFILES)
    
    if traj_type == '1': # === CIRCLE (Dễ) ===
        print(f"🚀 MODE: CIRCLE (R={radius_val}m)")
        traj_speed = 0.3         
        ramp_time = 3.0          
        corner_scale = 1.0
        
        # Controller limits (Dư địa để đuổi theo)
        ctrl_max_lin = 0.40      
        ctrl_max_ang = 0.8       
        
    elif traj_type == '2': # === SQUARE (Trung bình) ===
        print(f"🚀 MODE: SQUARE (Side={radius_val*2}m)")
        traj_speed = 0.20        
        ramp_time = 3.0          
        corner_scale = 0.4       # Giảm tốc ở góc vuông
        
        # Controller limits
        ctrl_max_lin = 0.30      
        ctrl_max_ang = 0.9       # Cần quay nhanh ở góc
        
    elif traj_type == '3': # === FIGURE-8 (Khó nhất) ===
        print(f"🚀 MODE: FIGURE-8 (Amplitude={radius_val/2}m)")
        traj_speed = 0.18        
        ramp_time = 5.0          # Khởi động siêu chậm
        corner_scale = 0.5       # Giảm tốc ở đỉnh cong
        
        # Controller limits
        ctrl_max_lin = 0.28      
        ctrl_max_ang = 0.8       
        
    else: # Fallback
        traj_speed = 0.15
        ramp_time = 3.0
        corner_scale = 1.0
        ctrl_max_lin = 0.25
        ctrl_max_ang = 0.5

    # 3. Định nghĩa Nodes
    
    # Node 1: Publisher
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
            'path_points': 200,
            'preview_time': 10.0,
            'enable_publish': True,
            'trajectory_speed': traj_speed,
            'ramp_time': ramp_time,
            'corner_speed_scale': corner_scale,
        }]
    )

    # Node 2: Controller
    controller_node = Node(
        package='agv_trajectory_tracking',
        executable='fuzzy_trajectory_controller',
        name='fuzzy_trajectory_controller',
        output='screen',
        parameters=[{
            'wheel_base': 0.42,        # ✅ ĐÃ CHUẨN (0.42m)
            'control_frequency': 20.0, 
            'goal_tolerance': 0.08,
            'enable_path_publish': True,
            'verbose_logging': True,
            'max_linear_vel': ctrl_max_lin,
            'max_angular_vel': ctrl_max_ang,
        }]
    )

    # Node 3: Plotter (Auto-Save)
    # Sửa lại tham số cho đúng với code Python mới
    plotter_node = Node(
        package='agv_trajectory_tracking',
        executable='trajectory_plotter',
        name='trajectory_plotter',
        output='screen',
        parameters=[{
            'max_history': 20000, # ✅ Sửa từ update_rate thành max_history
        }]
    )

    return [publisher_node, controller_node, plotter_node]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('trajectory_type', default_value='1'),
        DeclareLaunchArgument('radius', default_value='1.0'),
        DeclareLaunchArgument('center_x', default_value='0.0'), # Đổi default về 0.0 cho an toàn
        DeclareLaunchArgument('center_y', default_value='0.0'),
        OpaqueFunction(function=launch_setup)
    ])