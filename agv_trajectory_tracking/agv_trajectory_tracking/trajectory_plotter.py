#!/usr/bin/env python3
"""
Trajectory Plotter (Final + XY Errors)
- Plot 1: Trajectory Path
- Plot 2: Cross-track Error (Distance)
- Plot 3: Wheel Velocities
- Plot 4: Position Error (X and Y separately) -> NEW!

Author: Thuong Tran Dinh
Updated: November 27, 2025
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry 
from std_msgs.msg import Float32MultiArray
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
        
        # --- CONFIG SAVE PATH ---
        user_home = os.path.expanduser('~')
        self.base_results_dir = os.path.join(user_home, 'results')
        
        self.session_name = datetime.now().strftime("run_%Y-%m-%d_%H-%M-%S")
        self.save_directory = os.path.join(self.base_results_dir, self.session_name)

        # Data storage
        self.ref_x = []
        self.ref_y = []
        self.actual_x = []
        self.actual_y = []
        self.time_stamps = []
        self.errors = []        # Cross-track (Euclidean)
        self.errors_x = []      # Error in X
        self.errors_y = []      # Error in Y
        
        # Velocity Data
        self.vel_time = []
        self.vel_left = []
        self.vel_right = []
        
        self.start_time = None
        self.last_ref_point = None
        self.last_actual_point = None
        
        # Subscribers
        self.ref_path_sub = self.create_subscription(
            Path, '/trajectory', self.reference_callback, 10)
        
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10)
            
        self.sensor_sub = self.create_subscription(
            Float32MultiArray, '/sensor_data', self.sensor_callback, 10)
            
        self.get_logger().info('📥 Plotter Ready (XY Error Added).')
        self.get_logger().info(f'   📁 Session Folder: {self.save_directory}')

    def reference_callback(self, msg):
        if len(msg.poses) > 0:
            pose = msg.poses[0].pose.position
            point = (pose.x, pose.y)
            if not self.ref_x or self.distance((point[0], point[1]), (self.ref_x[-1], self.ref_y[-1])) > 0.005:
                self.ref_x.append(point[0])
                self.ref_y.append(point[1])

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
            
            if len(self.ref_x) > 5:
                # Lấy cả khoảng cách VÀ tọa độ điểm tham chiếu gần nhất
                dist, ref_px, ref_py = self.compute_true_cross_track_error(point)
                
                current_time = self.get_clock().now().nanoseconds / 1e9 - self.start_time
                self.time_stamps.append(current_time)
                self.errors.append(dist)
                
                # Tính sai số theo trục (Actual - Reference)
                self.errors_x.append(x - ref_px)
                self.errors_y.append(y - ref_py)

    def sensor_callback(self, msg):
        if self.start_time is None: return
        if len(msg.data) >= 8:
            current_time = self.get_clock().now().nanoseconds / 1e9 - self.start_time
            v_left = msg.data[6]
            v_right = msg.data[7]
            self.vel_time.append(current_time)
            self.vel_left.append(v_left)
            self.vel_right.append(v_right)

    def compute_true_cross_track_error(self, point):
        """
        Returns: (min_dist, closest_ref_x, closest_ref_y)
        """
        px, py = point
        min_dist = float('inf')
        closest_ref_point = (0.0, 0.0) # (x, y)
        
        best_idx = 0; best_dist_sq = float('inf')
        # Coarse search
        for i in range(len(self.ref_x)):
            dx = px - self.ref_x[i]; dy = py - self.ref_y[i]; d_sq = dx*dx + dy*dy
            if d_sq < best_dist_sq: best_dist_sq = d_sq; best_idx = i
            
        # Fine search
        search_radius = 5
        start_i = max(0, best_idx - search_radius)
        end_i = min(len(self.ref_x) - 1, best_idx + search_radius)
        
        for i in range(start_i, end_i):
            x1, y1 = self.ref_x[i], self.ref_y[i]
            x2, y2 = self.ref_x[i+1], self.ref_y[i+1]
            dx = x2 - x1; dy = y2 - y1; seg_len_sq = dx*dx + dy*dy
            
            curr_closest_x = 0.0
            curr_closest_y = 0.0
            
            if seg_len_sq < 1e-6:
                dist = math.sqrt((px - x1)**2 + (py - y1)**2)
                curr_closest_x = x1
                curr_closest_y = y1
            else:
                t = max(0.0, min(1.0, ((px - x1)*dx + (py - y1)*dy) / seg_len_sq))
                curr_closest_x = x1 + t * dx
                curr_closest_y = y1 + t * dy
                dist = math.sqrt((px - curr_closest_x)**2 + (py - curr_closest_y)**2)
            
            if dist < min_dist:
                min_dist = dist
                closest_ref_point = (curr_closest_x, curr_closest_y)
                
        return min_dist, closest_ref_point[0], closest_ref_point[1]

    def distance(self, p1, p2):
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def save_results(self):
        if not self.actual_x:
            self.get_logger().warn("⚠️ No data collected!")
            return

        print(f"\n📊 Saving reports to: {self.save_directory}")
        if not os.path.exists(self.save_directory):
            try: os.makedirs(self.save_directory)
            except OSError: self.save_directory = os.getcwd()

        # --- PLOT 1: PATH ---
        fig1 = plt.figure(figsize=(10, 10))
        plt.plot(self.ref_x, self.ref_y, 'b--', linewidth=2, label='Reference')
        plt.plot(self.actual_x, self.actual_y, 'r-', linewidth=2, label='Actual (EKF)', alpha=0.8)
        if self.actual_x:
            plt.plot(self.actual_x[0], self.actual_y[0], 'go', label='Start')
            plt.plot(self.actual_x[-1], self.actual_y[-1], 'rx', label='End')
        plt.xlabel('X (m)'); plt.ylabel('Y (m)'); plt.title('Trajectory Path')
        plt.legend(); plt.grid(True); plt.axis('equal')
        plt.savefig(os.path.join(self.save_directory, "1_path.png"), dpi=100)
        plt.close(fig1)

        # Data Trimming (Skip first 20 points)
        trim = min(len(self.errors), 20)
        valid_time = self.time_stamps[trim:] if len(self.time_stamps) > trim else self.time_stamps
        valid_errors = self.errors[trim:] if len(self.errors) > trim else self.errors
        
        # --- PLOT 2: CROSS-TRACK ERROR ---
        mean_error = np.mean(valid_errors) if valid_errors else 0.0
        max_error = np.max(valid_errors) if valid_errors else 0.0
        rmse = np.sqrt(np.mean(np.square(valid_errors))) if valid_errors else 0.0

        fig2 = plt.figure(figsize=(12, 6))
        plt.plot(valid_time, valid_errors, 'k-', linewidth=1)
        plt.axhline(y=mean_error, color='r', linestyle='--', label=f'Mean: {mean_error:.4f}m')
        plt.fill_between(valid_time, 0, valid_errors, color='green', alpha=0.2)
        plt.xlabel('Time (s)'); plt.ylabel('Error (m)')
        plt.title(f'Tracking Error (RMSE: {rmse:.4f}m)')
        plt.legend(); plt.grid(True)
        plt.ylim(0, max(0.1, max_error * 1.2))
        plt.savefig(os.path.join(self.save_directory, "2_error_dist.png"), dpi=100)
        plt.close(fig2)
        
        # --- PLOT 3: VELOCITY ---
        if self.vel_time:
            fig3 = plt.figure(figsize=(12, 6))
            plt.plot(self.vel_time, self.vel_left, 'b-', label='Left Wheel', alpha=0.7)
            plt.plot(self.vel_time, self.vel_right, 'r-', label='Right Wheel', alpha=0.7)
            plt.xlabel('Time (s)'); plt.ylabel('Velocity (m/s)')
            plt.title('Wheel Velocities')
            plt.legend(); plt.grid(True)
            plt.savefig(os.path.join(self.save_directory, "3_velocity.png"), dpi=100)
            plt.close(fig3)

        # --- PLOT 4: X/Y ERRORS (NEW!) ---
        valid_err_x = self.errors_x[trim:] if len(self.errors_x) > trim else self.errors_x
        valid_err_y = self.errors_y[trim:] if len(self.errors_y) > trim else self.errors_y
        
        if valid_err_x and valid_err_y:
            fig4 = plt.figure(figsize=(12, 6))
            plt.plot(valid_time, valid_err_x, 'r-', label='Error X', linewidth=1.5, alpha=0.8)
            plt.plot(valid_time, valid_err_y, 'b-', label='Error Y', linewidth=1.5, alpha=0.8)
            plt.axhline(y=0, color='k', linestyle='--', linewidth=0.8)
            
            plt.xlabel('Time (s)'); plt.ylabel('Deviation (m)')
            plt.title('Position Error Breakdown (X vs Y)')
            plt.legend(); plt.grid(True)
            
            # Auto scale Y to show data clearly
            max_dev = max(np.max(np.abs(valid_err_x)), np.max(np.abs(valid_err_y)))
            plt.ylim(-max_dev*1.2, max_dev*1.2)
            
            plt.savefig(os.path.join(self.save_directory, "4_xy_error.png"), dpi=100)
            plt.close(fig4)

        print(f"\033[92m✅ All 4 reports saved successfully!\033[0m")

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