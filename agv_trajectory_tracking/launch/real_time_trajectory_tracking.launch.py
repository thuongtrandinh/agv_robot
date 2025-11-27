import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    """
    Cấu hình động cho Robot AGV (Wheelbase 0.42m, Wheel Radius 0.05m)
    Map nhỏ: Radius 1.0m
    """
    # 1. Lấy giá trị từ CLI
    traj_type = context.launch_configurations['trajectory_type']
    radius_val = float(context.launch_configurations['radius'])
    center_x = context.launch_configurations['center_x']
    center_y = context.launch_configurations['center_y']
    
    # 2. CẤU HÌNH TỰ ĐỘNG (TUNING PRESETS)
    # Đã tinh chỉnh để phù hợp với wheelbase 0.42m (lanh hơn 0.46m một chút)
    
    if traj_type == '1': # === CIRCLE (Dễ) ===
        print(f"🚀 MODE: CIRCLE (R={radius_val}m)")
        traj_speed = 0.3        # Tốc độ ổn định
        ramp_time = 2.0          # Khởi động 2s
        corner_scale = 1.0       # Không giảm tốc
        
        # Controller limits
        ctrl_max_lin = 0.30      
        ctrl_max_ang = 0.8       
        
    elif traj_type == '2': # === SQUARE (Trung bình) ===
        print(f"🚀 MODE: SQUARE (Side={radius_val*2}m)")
        traj_speed = 0.20        
        ramp_time = .0
        corner_scale = 0.4       # Giảm 60% tốc độ ở góc vuông
        
        # Controller limits
        ctrl_max_lin = 0.30
        ctrl_max_ang = 0.9       # Cần quay nhanh ở góc
        
    elif traj_type == '3': # === FIGURE-8 (Khó nhất) ===
        print(f"🚀 MODE: FIGURE-8 (Amplitude={radius_val/2}m)")
        # R=1m -> Cua thực tế 0.5m. Robot 0.42m bám tốt hơn 0.46m
        traj_speed = 0.18        # Tăng nhẹ lên 0.18 (so với 0.16 cũ) vì robot nhỏ hơn chút
        ramp_time = 5.0          # Khởi động siêu chậm
        corner_scale = 0.5       # Giảm 50% ở đỉnh số 8
        
        # Controller limits
        ctrl_max_lin = 0.25      
        ctrl_max_ang = 0.7       
        
    else: # Fallback
        traj_speed = 0.15
        ramp_time = 3.0
        corner_scale = 1.0
        ctrl_max_lin = 0.25
        ctrl_max_ang = 0.5

    # 3. Định nghĩa Nodes
    
    # Node 1: Trajectory Publisher (Người dẫn đường)
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
            # Parameters tự động
            'trajectory_speed': traj_speed,
            'ramp_time': ramp_time,
            'corner_speed_scale': corner_scale,
        }]
    )

    # Node 2: Fuzzy Controller (Người lái xe)
    controller_node = Node(
        package='agv_trajectory_tracking',
        executable='fuzzy_trajectory_controller',
        name='fuzzy_trajectory_controller',
        output='screen',
        parameters=[{
            'wheel_base': 0.42,        # ✅ ĐÃ CẬP NHẬT CHÍNH XÁC
            'control_frequency': 20.0, # Đồng bộ 20Hz
            'goal_tolerance': 0.08,
            'enable_path_publish': True,
            'verbose_logging': True,
            # Parameters tự động
            'max_linear_vel': ctrl_max_lin,
            'max_angular_vel': ctrl_max_ang,
        }]
    )

    # # Node 3: Plotter (Người giám sát)
    # plotter_node = Node(
    #     package='agv_trajectory_tracking',
    #     executable='trajectory_plotter',
    #     name='trajectory_plotter',
    #     output='screen',
    #     parameters=[{
    #         'max_history': 5000,
    #         'update_interval': 50,
    #         'publish_rate': 30.0,
    #     }]
    # )

    return [publisher_node, controller_node] #, plotter_node

def generate_launch_description():
    return LaunchDescription([
        # Chọn quỹ đạo: 1=Tròn, 2=Vuông, 3=Số 8
        DeclareLaunchArgument('trajectory_type', default_value='1'),
            
        # Bán kính 1m
        DeclareLaunchArgument('radius', default_value='1.0'),
            
        # Tâm lệch 1m (Robot tại 0,0 sẽ cách tâm 1m -> Bám ngay vào quỹ đạo)
        DeclareLaunchArgument('center_x', default_value='0.7'),
        DeclareLaunchArgument('center_y', default_value='0.15'),
            
        OpaqueFunction(function=launch_setup)
    ])