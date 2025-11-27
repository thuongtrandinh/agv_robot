#!/usr/bin/env python3
"""
Trajectory Publisher with Adaptive Speed Profiling
Author: Thuong Tran Dinh
Updated: November 27, 2025

Features:
- Adaptive Speed: Slows down at corners (Square) and high curvature (Figure-8)
- Reduces Overshoot for PID controller
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math
import numpy as np

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        
        # Parameters
        self.declare_parameter('trajectory_type', 2)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('path_points', 200)
        self.declare_parameter('preview_time', 10.0)
        self.declare_parameter('center_x', 0.7)
        self.declare_parameter('center_y', 0.25)
        self.declare_parameter('radius', 1.0)
        self.declare_parameter('enable_publish', True)
        
        # --- SPEED PARAMETERS ---
        # Tốc độ tối đa khi đi đường thẳng
        self.declare_parameter('trajectory_speed', 0.3) 
        # Tốc độ tối thiểu khi vào cua (Corner speed)
        self.declare_parameter('corner_speed_scale', 0.3) # Giảm còn 30% tốc độ khi vào cua
        self.declare_parameter('ramp_time', 3.0) 
        
        # Get values
        self.trajectory_type = self.get_parameter('trajectory_type').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.path_points = self.get_parameter('path_points').value
        self.preview_time = self.get_parameter('preview_time').value
        self.center_x = self.get_parameter('center_x').value
        self.center_y = self.get_parameter('center_y').value
        self.radius = self.get_parameter('radius').value
        self.enable_publish = self.get_parameter('enable_publish').value
        self.base_speed = self.get_parameter('trajectory_speed').value
        self.corner_scale = self.get_parameter('corner_speed_scale').value
        self.ramp_time = self.get_parameter('ramp_time').value
        
        self.path_pub = self.create_publisher(Path, '/trajectory', 10)
        
        # Logic State
        self.is_ready = False
        self.startup_delay = 3.0
        self.current_time = 0.0
        self.real_time_elapsed = 0.0 # Để tính ramp khởi động
        self.dt = 1.0 / self.publish_rate
        
        self.startup_timer = self.create_timer(self.startup_delay, self.on_startup)
        self.timer = self.create_timer(self.dt, self.publish_loop)
        
        self.get_logger().info(f'✅ Adaptive Trajectory Publisher Started')
        self.get_logger().info(f'   Max Speed: {self.base_speed} m/s')
        self.get_logger().info(f'   Corner Factor: {self.corner_scale*100}%')

    def on_startup(self):
        self.is_ready = True
        self.startup_timer.cancel()
        self.get_logger().info('🚀 Starting Trajectory Generation...')

    # ==================================================
    # ADAPTIVE SPEED LOGIC (CORE INTELLIGENCE)
    # ==================================================
    def calculate_speed_factor(self, t):
        """
        Trả về hệ số tốc độ (0.0 -> 1.0) tùy thuộc vào hình dạng quỹ đạo
        """
        # 1. Khởi động mềm (Global Ramp)
        ramp_factor = 1.0
        if self.real_time_elapsed < self.ramp_time:
             # Hàm Sigmoid để tăng tốc mượt
             progress = self.real_time_elapsed / self.ramp_time
             ramp_factor = progress * progress * (3.0 - 2.0 * progress)

        # 2. Giảm tốc theo hình dạng (Shape-based Slowdown)
        shape_factor = 1.0
        
        if self.trajectory_type == 2: # SQUARE
            # Logic: Giảm tốc khi gần các góc (Corner)
            side = self.radius * 2.0
            # Chu vi = 4 * side. Thời gian đi hết 1 vòng (nếu speed=1) = 4*side
            # Chúng ta chuẩn hóa time t theo chu vi
            perimeter_pos = (t * self.base_speed) % (4 * side) # Vị trí trên chu vi (m)
            dist_on_side = perimeter_pos % side # Vị trí trên cạnh hiện tại (m)
            
            # Vùng giảm tốc: 15% cuối cạnh và 15% đầu cạnh mới
            threshold = side * 0.15 
            
            # Tính khoảng cách đến góc gần nhất
            dist_to_corner = min(dist_on_side, side - dist_on_side)
            
            if dist_to_corner < threshold:
                # Càng gần góc, càng chậm
                ratio = dist_to_corner / threshold
                # Map ratio [0, 1] -> [corner_scale, 1.0]
                shape_factor = self.corner_scale + (1.0 - self.corner_scale) * ratio
            else:
                shape_factor = 1.0

        elif self.trajectory_type == 3: # FIGURE-8
            # Logic: Giảm tốc ở 2 đầu vòng cung (High Curvature), nhanh ở giữa
            # Chu kỳ T xấp xỉ 2*pi*A / speed
            A = self.radius / 2.0
            omega_nominal = self.base_speed / (2.0 * A)
            phase = (omega_nominal * t) % (2 * math.pi)
            
            # Các đỉnh của số 8 là tại phase = pi/2 (90 độ) và 3pi/2 (270 độ)
            # Tại đó curvature lớn nhất -> Cần đi chậm nhất
            dist_to_peak_1 = abs(phase - math.pi/2)
            dist_to_peak_2 = abs(phase - 3*math.pi/2)
            min_dist_peak = min(dist_to_peak_1, dist_to_peak_2)
            
            # Vùng ảnh hưởng: +/- 45 độ (pi/4) quanh đỉnh
            if min_dist_peak < (math.pi / 3.0):
                ratio = min_dist_peak / (math.pi / 3.0)
                # Dùng hàm mũ để giảm tốc mượt hơn
                shape_factor = self.corner_scale + (1.0 - self.corner_scale) * (ratio**0.5)
            else:
                shape_factor = 1.0

        elif self.trajectory_type == 1: # CIRCLE
            # Hình tròn độ cong đều, chạy vận tốc đều
            shape_factor = 1.0
            
        return ramp_factor * shape_factor

    # ==================================================
    # TRAJECTORY GENERATORS
    # ==================================================
    def trajectory_reference_square(self, t):
        side = self.radius * 2.0
        # Tính toán dựa trên quãng đường s = v*t ảo
        # Để đơn giản, giả sử v=1 trong công thức hình học, 
        # việc scale tốc độ thực tế đã được xử lý ở self.current_time
        total_len = 4 * side
        s = (t * self.base_speed) % total_len # Quãng đường đã đi
        
        if s < side: # Side 1 (Up)
            x = self.center_x + side/2
            y = self.center_y - side/2 + s
            th = math.pi/2
        elif s < 2*side: # Side 2 (Left)
            x = self.center_x + side/2 - (s - side)
            y = self.center_y + side/2
            th = math.pi
        elif s < 3*side: # Side 3 (Down)
            x = self.center_x - side/2
            y = self.center_y + side/2 - (s - 2*side)
            th = -math.pi/2
        else: # Side 4 (Right)
            x = self.center_x - side/2 + (s - 3*side)
            y = self.center_y - side/2
            th = 0.0
        return x, y, th

    def trajectory_reference_figure8(self, t):
        A = self.radius / 2.0
        # Omega danh định
        omega = self.base_speed / (2.0 * A)
        phi = omega * t
        
        x_rel = A * math.cos(phi)
        y_rel = A * math.sin(2*phi) / 2.0
        
        x = self.center_x + x_rel
        y = self.center_y + y_rel
        
        dx = -A * omega * math.sin(phi)
        dy = A * omega * math.cos(2*phi)
        th = math.atan2(dy, dx)
        return x, y, th

    def trajectory_reference_circle(self, t):
        R = self.radius
        omega = self.base_speed / R
        phi = omega * t
        
        x = self.center_x + R * math.cos(phi)
        y = self.center_y + R * math.sin(phi)
        th = phi + math.pi/2
        return x, y, th

    def get_ref_pose(self, t):
        if self.trajectory_type == 1: return self.trajectory_reference_circle(t)
        elif self.trajectory_type == 2: return self.trajectory_reference_square(t)
        elif self.trajectory_type == 3: return self.trajectory_reference_figure8(t)
        return 0,0,0

    # ==================================================
    # PUBLISH LOOP
    # ==================================================
    def publish_loop(self):
        if not self.is_ready: return
        
        if self.enable_publish:
            path_msg = Path()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = 'map'
            
            # --- PREVIEW GENERATION ---
            # Ta tạo ra các điểm tương lai (Preview)
            # QUAN TRỌNG: Các điểm tương lai cũng phải tuân theo quy luật giảm tốc
            # Nên ta dùng một biến temp_time chạy mô phỏng
            temp_time = self.current_time
            
            preview_dt = self.preview_time / self.path_points
            
            for _ in range(self.path_points):
                # Tính factor tốc độ tại điểm tương lai đó
                factor = self.calculate_speed_factor(temp_time)
                
                # Time warping: Thời gian ảo tăng chậm lại nếu factor nhỏ
                # Nếu factor = 1.0 -> temp_time tăng đúng preview_dt
                # Nếu factor = 0.3 -> temp_time chỉ tăng 30% -> Các điểm xít lại gần nhau (Robot đi chậm)
                step = preview_dt * factor # Nhưng ở đây ta đang vẽ path cố định khoảng cách hay thời gian?
                # Để controller hiểu là phải đi chậm, các điểm path nên dày đặc hơn HOẶC
                # đơn giản là điểm tham chiếu hiện tại (current_time) di chuyển chậm lại.
                
                # Với MPC thì cần path dày đặc, với Pure Pursuit/Fuzzy thì chỉ cần điểm tham chiếu hiện tại chuẩn.
                # Tuy nhiên để vẽ cho đẹp, ta cứ tăng đều t
                
                x, y, th = self.get_ref_pose(temp_time)
                
                # Tăng thời gian mô phỏng cho điểm tiếp theo
                # Lưu ý: preview path chỉ để vẽ, không ảnh hưởng logic điều khiển lắm
                temp_time += preview_dt 
                
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = x
                pose.pose.position.y = y
                
                # Euler to Quat
                cy = math.cos(th * 0.5); sy = math.sin(th * 0.5)
                pose.pose.orientation.z = sy; pose.pose.orientation.w = cy
                path_msg.poses.append(pose)
            
            self.path_pub.publish(path_msg)

        # --- UPDATE CURRENT REFERENCE STATE ---
        # Đây là bước quan trọng nhất: Cập nhật vị trí con mồi
        
        # 1. Tính hệ số tốc độ hiện tại
        speed_factor = self.calculate_speed_factor(self.current_time)
        
        # 2. Time Warping: Chỉ tăng thời gian tham chiếu một lượng nhỏ nếu đang vào cua
        # self.dt là bước thời gian thực (0.05s). 
        # effective_dt là bước thời gian trên quỹ đạo.
        effective_dt = self.dt * speed_factor
        
        self.current_time += effective_dt
        self.real_time_elapsed += self.dt # Thời gian thực vẫn trôi đều

        # Log
        if self.current_time % 1.0 < self.dt: # Log mỗi giây ảo
            self.get_logger().info(f'Speed Factor: {speed_factor:.2f} | T_ref: {self.current_time:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: 
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()