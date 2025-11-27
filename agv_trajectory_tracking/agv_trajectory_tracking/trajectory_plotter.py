#!/usr/bin/env python3
"""
Offline Trajectory Plotter
Collects data during run, plots results when stopped (Ctrl+C).

Author: Thuong Tran Dinh
Updated: November 2025
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped
import matplotlib.pyplot as plt
import numpy as np
import math

class TrajectoryPlotter(Node):
    def __init__(self):
        super().__init__('trajectory_plotter')
        
        # Parameters
        self.declare_parameter('max_history', 10000) # Tăng bộ nhớ lên để lưu toàn bộ quá trình
        self.max_history = self.get_parameter('max_history').value
        
        # Data storage
        self.ref_x = []
        self.ref_y = []
        self.actual_x = []
        self.actual_y = []
        self.time_stamps = []
        self.errors = []
        
        self.start_time = None
        self.last_ref_point = None
        self.last_actual_point = None
        
        # Subscribers
        self.ref_path_sub = self.create_subscription(
            Path, '/trajectory',
            self.reference_callback, 10)
        
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose',
            self.pose_callback, 10)
            
        self.get_logger().info('📥 Plotter started in Data Collection Mode.')
        self.get_logger().info('   (Matplotlib window will open when you stop the launch file)')

    def reference_callback(self, msg):
        """Store reference trajectory"""
        if len(msg.poses) > 0:
            # Lấy điểm đầu tiên của path làm điểm tham chiếu hiện tại
            pose = msg.poses[0].pose.position
            point = (pose.x, pose.y)
            
            # Chỉ lưu nếu điểm thay đổi (tránh trùng lặp dữ liệu)
            if self.last_ref_point is None or \
               self.distance(point, self.last_ref_point) > 0.01:
                self.ref_x.append(pose.x)
                self.ref_y.append(pose.y)
                self.last_ref_point = point

    def pose_callback(self, msg):
        """Store actual robot trajectory and compute error"""
        if self.start_time is None:
            self.start_time = self.get_clock().now().nanoseconds / 1e9

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        point = (x, y)
        
        # Lưu dữ liệu thực tế
        if self.last_actual_point is None or \
           self.distance(point, self.last_actual_point) > 0.01:
            self.actual_x.append(x)
            self.actual_y.append(y)
            self.last_actual_point = point
            
            # Tính sai số bám (Cross-track Error)
            if len(self.ref_x) > 1:
                error = self.compute_cross_track_error(point)
                current_time = self.get_clock().now().nanoseconds / 1e9 - self.start_time
                
                self.time_stamps.append(current_time)
                self.errors.append(error)

    def compute_cross_track_error(self, point):
        """Tính khoảng cách vuông góc từ robot đến quỹ đạo tham chiếu"""
        # (Giữ nguyên logic tính toán của bạn vì nó đã tốt)
        px, py = point
        min_dist = float('inf')
        
        # Tìm trong 50 điểm tham chiếu gần nhất (tối ưu hiệu năng)
        check_range = 50
        start_idx = max(0, len(self.ref_x) - check_range)
        
        for i in range(start_idx, len(self.ref_x) - 1):
            x1, y1 = self.ref_x[i], self.ref_y[i]
            x2, y2 = self.ref_x[i+1], self.ref_y[i+1]
            
            dx = x2 - x1
            dy = y2 - y1
            seg_len_sq = dx*dx + dy*dy
            
            if seg_len_sq < 1e-6:
                dist = math.sqrt((px - x1)**2 + (py - y1)**2)
            else:
                t = max(0.0, min(1.0, ((px - x1)*dx + (py - y1)*dy) / seg_len_sq))
                closest_x = x1 + t * dx
                closest_y = y1 + t * dy
                dist = math.sqrt((px - closest_x)**2 + (py - closest_y)**2)
            
            if dist < min_dist:
                min_dist = dist
        return min_dist

    def distance(self, p1, p2):
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def show_results(self):
        """Hàm này sẽ được gọi khi Node tắt để vẽ đồ thị"""
        if not self.actual_x:
            print("⚠️ No data collected to plot!")
            return

        print("\n📊 Generatring Final Report...")
        
        # Tính toán chỉ số thống kê
        mean_error = np.mean(self.errors) if self.errors else 0.0
        max_error = np.max(self.errors) if self.errors else 0.0
        rmse = np.sqrt(np.mean(np.square(self.errors))) if self.errors else 0.0
        
        # Setup đồ thị
        fig = plt.figure(figsize=(12, 8))
        fig.suptitle(f'Trajectory Tracking Result\nRMSE: {rmse:.4f}m | Max Error: {max_error:.4f}m', fontsize=16)

        # 1. Đồ thị Quỹ đạo (X-Y)
        ax1 = fig.add_subplot(2, 1, 1)
        ax1.plot(self.ref_x, self.ref_y, 'b--', linewidth=2, label='Reference Path')
        ax1.plot(self.actual_x, self.actual_y, 'r-', linewidth=2, label='Actual Path', alpha=0.8)
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_title('Path Comparison')
        ax1.legend()
        ax1.grid(True)
        ax1.axis('equal')

        # 2. Đồ thị Sai số theo thời gian
        ax2 = fig.add_subplot(2, 1, 2)
        if self.errors:
            ax2.plot(self.time_stamps, self.errors, 'g-', label='Cross-Track Error')
            ax2.axhline(y=mean_error, color='orange', linestyle='--', label=f'Mean: {mean_error:.3f}m')
            ax2.fill_between(self.time_stamps, 0, self.errors, color='green', alpha=0.1)
        
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Error (m)')
        ax2.set_title('Tracking Error over Time')
        ax2.legend()
        ax2.grid(True)

        plt.tight_layout()
        print("✅ Plot displayed. Close window to finish process.")
        plt.show() # Lệnh này sẽ block cho đến khi bạn tắt cửa sổ

def main(args=None):
    rclpy.init(args=args)
    plotter = TrajectoryPlotter()
    
    try:
        # Chạy vòng lặp cho đến khi bấm Ctrl+C
        rclpy.spin(plotter)
    except KeyboardInterrupt:
        pass
    finally:
        # Khi bấm Ctrl+C, code sẽ nhảy vào đây
        plotter.get_logger().info('🛑 Stopping collection...')
        
        # Tắt ROS trước để giải phóng tài nguyên
        plotter.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
            
        # SAU ĐÓ mới hiện đồ thị
        plotter.show_results()

if __name__ == '__main__':
    main()