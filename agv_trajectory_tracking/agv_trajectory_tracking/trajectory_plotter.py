#!/usr/bin/env python3
"""
Trajectory Plotter (Home Directory Save)
Calculates Geometry-based Cross-Track Error.
Auto-saves results to: ~/results/ (User's home directory)

Author: Thuong Tran Dinh
Updated: November 27, 2025
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf2_ros
import matplotlib.pyplot as plt
import numpy as np
import math
import os
from datetime import datetime

class TrajectoryPlotter(Node):
    def __init__(self):
        super().__init__('trajectory_plotter')
        
        # Parameters
        self.declare_parameter('max_history', 20000) 
        self.max_history = self.get_parameter('max_history').value
        
        # --- CẤU HÌNH SAVE PATH: ~/results ---
        # os.path.expanduser('~') sẽ trả về /home/hdt (hoặc user hiện tại)
        user_home = os.path.expanduser('~')
        self.save_directory = os.path.join(user_home, 'results')

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
            Path, '/trajectory', self.reference_callback, 10)
        
        # TF frames
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Timer to sample TF and compute errors
        self.declare_parameter('tf_sample_hz', 20.0)
        self.tf_sample_hz = float(self.get_parameter('tf_sample_hz').value)
        self.tf_timer = self.create_timer(1.0 / self.tf_sample_hz, self.tf_sample_callback)
            
        self.get_logger().info('📥 Plotter Ready (Geometry Error Metric).')
        self.get_logger().info(f'   📁 Target Folder: {self.save_directory}')

    def reference_callback(self, msg):
        """Store reference trajectory points (Single line trace)"""
        if len(msg.poses) > 0:
            pose = msg.poses[0].pose.position
            point = (pose.x, pose.y)
            # Save only if point changed (avoid duplicates)
            if not self.ref_x or self.distance((point[0], point[1]), (self.ref_x[-1], self.ref_y[-1])) > 0.005:
                self.ref_x.append(point[0])
                self.ref_y.append(point[1])

    def tf_sample_callback(self):
        """Sample TF map->base_footprint, store pose, and compute error"""
        if self.start_time is None:
            self.start_time = self.get_clock().now().nanoseconds / 1e9

        try:
            trans = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time())
        except Exception as e:
            # Skip until TF is available
            return

        x = trans.transform.translation.x
        y = trans.transform.translation.y
        point = (x, y)

        if self.last_actual_point is None or \
           self.distance(point, self.last_actual_point) > 0.01:
            self.actual_x.append(x)
            self.actual_y.append(y)
            self.last_actual_point = point

            if len(self.ref_x) > 5:
                error = self.compute_true_cross_track_error(point)
                current_time = self.get_clock().now().nanoseconds / 1e9 - self.start_time
                self.time_stamps.append(current_time)
                self.errors.append(error)

    def compute_true_cross_track_error(self, point):
        """Calculate shortest distance to the ENTIRE reference path"""
        px, py = point
        min_dist = float('inf')
        
        # Step 1: Coarse Search
        best_idx = 0
        best_dist_sq = float('inf')
        
        for i in range(len(self.ref_x)):
            dx = px - self.ref_x[i]
            dy = py - self.ref_y[i]
            d_sq = dx*dx + dy*dy
            if d_sq < best_dist_sq:
                best_dist_sq = d_sq
                best_idx = i
        
        # Step 2: Fine Search around best index
        search_radius = 5 
        start_i = max(0, best_idx - search_radius)
        end_i = min(len(self.ref_x) - 1, best_idx + search_radius)
        
        for i in range(start_i, end_i):
            x1, y1 = self.ref_x[i], self.ref_y[i]
            x2, y2 = self.ref_x[i+1], self.ref_y[i+1]
            
            dx = x2 - x1; dy = y2 - y1
            seg_len_sq = dx*dx + dy*dy
            
            if seg_len_sq < 1e-6:
                dist = math.sqrt((px - x1)**2 + (py - y1)**2)
            else:
                t = max(0.0, min(1.0, ((px - x1)*dx + (py - y1)*dy) / seg_len_sq))
                closest_x = x1 + t * dx
                closest_y = y1 + t * dy
                dist = math.sqrt((px - closest_x)**2 + (py - closest_y)**2)
            
            if dist < min_dist: min_dist = dist
        return min_dist

    def distance(self, p1, p2):
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def save_results(self):
        """Generate plots and save to ~/results"""
        if not self.actual_x:
            self.get_logger().warn("⚠️ No data collected to plot!")
            return

        print("\n📊 Generating Trajectory Report...")
        
        # Trim initial data (first 20 points) to remove startup jitter
        trim = min(len(self.errors), 20)
        valid_errors = self.errors[trim:] if len(self.errors) > trim else self.errors
        valid_time = self.time_stamps[trim:] if len(self.time_stamps) > trim else self.time_stamps
        
        mean_error = np.mean(valid_errors) if valid_errors else 0.0
        max_error = np.max(valid_errors) if valid_errors else 0.0
        rmse = np.sqrt(np.mean(np.square(valid_errors))) if valid_errors else 0.0
        
        fig = plt.figure(figsize=(14, 10))
        fig.suptitle(f'Trajectory Tracking Result\nRMSE: {rmse:.4f}m | Mean: {mean_error:.4f}m', fontsize=18, fontweight='bold')

        # Plot 1: Path
        ax1 = fig.add_subplot(2, 1, 1)
        ax1.plot(self.ref_x, self.ref_y, 'b--', linewidth=2, label='Reference')
        ax1.plot(self.actual_x, self.actual_y, 'r-', linewidth=2, label='Actual (EKF)', alpha=0.8)
        if self.actual_x:
            ax1.plot(self.actual_x[0], self.actual_y[0], 'go', label='Start')
            ax1.plot(self.actual_x[-1], self.actual_y[-1], 'rx', label='End')
        ax1.set_xlabel('X (m)'); ax1.set_ylabel('Y (m)')
        ax1.legend(); ax1.grid(True); ax1.axis('equal') 

        # Plot 2: Error
        ax2 = fig.add_subplot(2, 1, 2)
        if valid_errors:
            ax2.plot(valid_time, valid_errors, 'k-', linewidth=1)
            ax2.axhline(y=mean_error, color='r', linestyle='--', label=f'Mean: {mean_error:.3f}m')
            ax2.fill_between(valid_time, 0, valid_errors, color='green', alpha=0.2)
            ax2.set_xlabel('Time (s)'); ax2.set_ylabel('Cross-Track Error (m)')
            ax2.set_title('Tracking Error (Geometry Distance)')
            ax2.legend(); ax2.grid(True)
            
            y_max = max(0.1, max_error * 1.2)
            ax2.set_ylim(0, y_max)

        # --- TẠO THƯ MỤC VÀ LƯU ---
        if not os.path.exists(self.save_directory):
            try:
                os.makedirs(self.save_directory)
                print(f"📁 Created directory: {self.save_directory}")
            except OSError as e:
                print(f"⚠️ Error creating directory: {e}")
                self.save_directory = os.getcwd() # Fallback

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