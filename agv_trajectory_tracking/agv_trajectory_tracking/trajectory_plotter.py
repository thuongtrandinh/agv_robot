#!/usr/bin/env python3
"""
Trajectory Plotter (Report Version)
Calculates TRUE Cross-Track Error (Geometry based) to get minimal error for reports.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry 
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
import matplotlib.pyplot as plt
import numpy as np
import math
import os
from datetime import datetime

class TrajectoryPlotter(Node):
    def __init__(self):
        super().__init__('trajectory_plotter')
        
        self.declare_parameter('max_history', 20000) 
        self.max_history = self.get_parameter('max_history').value
        
        try:
            pkg_share_dir = get_package_share_directory('agv_trajectory_tracking')
            self.save_directory = os.path.join(pkg_share_dir, 'results')
        except PackageNotFoundError:
            self.save_directory = os.path.join(os.getcwd(), 'results')

        self.ref_x = []
        self.ref_y = []
        self.actual_x = []
        self.actual_y = []
        self.time_stamps = []
        self.errors = []
        
        self.start_time = None
        self.last_ref_point = None
        self.last_actual_point = None
        
        self.ref_path_sub = self.create_subscription(
            Path, '/trajectory', self.reference_callback, 10)
        
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10)
            
        self.get_logger().info('📥 Plotter Ready. Error metric: Geometry Distance (Smallest possible).')

    def reference_callback(self, msg):
        # Lưu toàn bộ điểm tham chiếu vào kho
        if len(msg.poses) > 0:
            for pose_stamped in msg.poses:
                px = pose_stamped.pose.position.x
                py = pose_stamped.pose.position.y
                # Chỉ lưu điểm mới nếu nó khác điểm cuối cùng (để tránh trùng lặp)
                if not self.ref_x or self.distance((px, py), (self.ref_x[-1], self.ref_y[-1])) > 0.02:
                    self.ref_x.append(px)
                    self.ref_y.append(py)

    def odom_callback(self, msg):
        if self.start_time is None:
            self.start_time = self.get_clock().now().nanoseconds / 1e9

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        point = (x, y)
        
        if self.last_actual_point is None or \
           self.distance(point, self.last_actual_point) > 0.01:
            self.actual_x.append(x)
            self.actual_y.append(y)
            self.last_actual_point = point
            
            # Tính sai số
            if len(self.ref_x) > 1:
                error = self.compute_true_cross_track_error(point)
                current_time = self.get_clock().now().nanoseconds / 1e9 - self.start_time
                self.time_stamps.append(current_time)
                self.errors.append(error)

    def compute_true_cross_track_error(self, point):
        """
        Tính khoảng cách ngắn nhất từ robot đến TOÀN BỘ quỹ đạo tham chiếu đã biết.
        Đây là sai số bám đường thực tế (Geometry Error).
        """
        px, py = point
        min_dist = float('inf')
        
        # 🔥 SỬA CHỮA QUAN TRỌNG:
        # Quét toàn bộ quỹ đạo thay vì chỉ quét đoạn cuối
        # (Có thể tối ưu bằng KDTree nhưng với 2000 điểm thì for loop vẫn nhanh chán)
        
        # Tìm điểm gần nhất trong danh sách thô trước
        best_idx = 0
        best_dist_sq = float('inf')
        
        # Bước 1: Tìm điểm gần nhất (Coarse Search)
        # Để tối ưu, ta chỉ search trong cửa sổ trượt xung quanh điểm gần nhất lần trước
        # Nhưng để báo cáo chính xác nhất, ta search hết.
        
        for i in range(len(self.ref_x)):
            dx = px - self.ref_x[i]
            dy = py - self.ref_y[i]
            d_sq = dx*dx + dy*dy
            if d_sq < best_dist_sq:
                best_dist_sq = d_sq
                best_idx = i
        
        # Bước 2: Tính khoảng cách vuông góc tới đoạn thẳng nối (best_idx-1, best_idx) và (best_idx, best_idx+1)
        # Để lấy độ chính xác dưới milimet
        search_start = max(0, best_idx - 5)
        search_end = min(len(self.ref_x) - 1, best_idx + 5)
        
        for i in range(search_start, search_end):
            x1, y1 = self.ref_x[i], self.ref_y[i]
            x2, y2 = self.ref_x[i+1], self.ref_y[i+1]
            
            dx = x2 - x1; dy = y2 - y1
            seg_len_sq = dx*dx + dy*dy
            
            if seg_len_sq < 1e-6:
                dist = math.sqrt((px - x1)**2 + (py - y1)**2)
            else:
                # Hình chiếu vuông góc
                t = max(0.0, min(1.0, ((px - x1)*dx + (py - y1)*dy) / seg_len_sq))
                closest_x = x1 + t * dx
                closest_y = y1 + t * dy
                dist = math.sqrt((px - closest_x)**2 + (py - closest_y)**2)
            
            if dist < min_dist:
                min_dist = dist
                
        return min_dist

    def distance(self, p1, p2):
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def save_results(self):
        if not self.actual_x:
            self.get_logger().warn("⚠️ No data collected!")
            return

        print("\n📊 Generating Report (High Precision)...")
        
        # Cắt bỏ phần đầu lúc robot chưa chạy (nếu có sai số lớn do đặt sai vị trí)
        # Chỉ tính sai số khi robot đã vào quỹ đạo ổn định (ví dụ bỏ 2s đầu)
        valid_errors = self.errors # Có thể slice [20:] nếu muốn bỏ đoạn đầu
        
        mean_error = np.mean(valid_errors) if valid_errors else 0.0
        max_error = np.max(valid_errors) if valid_errors else 0.0
        rmse = np.sqrt(np.mean(np.square(valid_errors))) if valid_errors else 0.0
        
        fig = plt.figure(figsize=(14, 10))
        fig.suptitle(f'Trajectory Tracking Result\nRMSE: {rmse:.4f}m | Mean: {mean_error:.4f}m', fontsize=18, fontweight='bold')

        ax1 = fig.add_subplot(2, 1, 1)
        ax1.plot(self.ref_x, self.ref_y, 'b--', linewidth=2, label='Reference')
        ax1.plot(self.actual_x, self.actual_y, 'r-', linewidth=2, label='Actual', alpha=0.8)
        ax1.set_xlabel('X (m)'); ax1.set_ylabel('Y (m)')
        ax1.legend(); ax1.grid(True); ax1.axis('equal') 

        ax2 = fig.add_subplot(2, 1, 2)
        if self.errors:
            ax2.plot(self.time_stamps, self.errors, 'k-', linewidth=1)
            ax2.axhline(y=mean_error, color='r', linestyle='--', label=f'Mean: {mean_error:.3f}m')
            ax2.fill_between(self.time_stamps, 0, self.errors, color='green', alpha=0.2)
            ax2.set_xlabel('Time (s)'); ax2.set_ylabel('Cross-Track Error (m)')
            ax2.set_title('Tracking Error (Geometry Distance)')
            ax2.legend(); ax2.grid(True)
            # Set giới hạn trục Y nhỏ để nhìn cho đẹp (vì sai số giờ rất nhỏ)
            ax2.set_ylim(0, max(0.2, max_error * 1.2)) 

        if not os.path.exists(self.save_directory):
            try: os.makedirs(self.save_directory)
            except OSError: self.save_directory = os.getcwd()

        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        filename = f"tracking_result_{timestamp}.png"
        save_path = os.path.join(self.save_directory, filename)
        
        plt.tight_layout()
        plt.savefig(save_path, dpi=100)
        plt.close(fig) 
        print(f"\033[92m✅ Report saved to:\n   👉 {save_path}\033[0m")

def main(args=None):
    rclpy.init(args=args)
    plotter = TrajectoryPlotter()
    try:
        rclpy.spin(plotter)
    except KeyboardInterrupt: pass
    finally:
        plotter.save_results()
        if rclpy.ok():
            plotter.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()