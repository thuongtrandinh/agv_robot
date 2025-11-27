#!/usr/bin/env python3
"""
Trajectory Plotter (Final Report Version)
- Fixes the "Solid Circle" visual bug by plotting only the active reference point trace.
- Calculates Geometry-based Cross-Track Error for accurate reporting.
- Auto-saves results to <package_share>/results/ on exit.

Author: Thuong Tran Dinh
Updated: November 27, 2025
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
        
        # Time window to ignore in reports/plots (seconds)
        # Default per request: remove 15s–25s segment
        self.declare_parameter('ignore_time_from', 15.0)
        self.declare_parameter('ignore_time_to', 25.0)
        self.ignore_time_from = float(self.get_parameter('ignore_time_from').value)
        self.ignore_time_to = float(self.get_parameter('ignore_time_to').value)
        
        # --- CONFIG SAVE PATH ---
        try:
            pkg_share_dir = get_package_share_directory('agv_trajectory_tracking')
            self.save_directory = os.path.join(pkg_share_dir, 'results')
        except PackageNotFoundError:
            self.save_directory = os.path.join(os.getcwd(), 'results')

        # Data storage
        self.ref_x = []
        self.ref_y = []
        self.actual_x = []
        self.actual_y = []
        self.time_stamps = []
        self.errors = []
        
        self.start_time = None
        self.last_actual_point = None
        
        # Subscribers
        self.ref_path_sub = self.create_subscription(
            Path, '/trajectory', self.reference_callback, 10)
        
        # Listen to EKF output for smooth plotting
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10)
            
        self.get_logger().info('📥 Plotter Ready. Error metric: Geometry Distance.')
        self.get_logger().info(f'   Target Folder: {self.save_directory}')

    def reference_callback(self, msg):
        """
        Store ONLY the current target point to draw a clean single line.
        Previous version stored all 200 preview points, causing the 'solid disk' look.
        """
        if len(msg.poses) > 0:
            # Only take the first point (current setpoint)
            pose = msg.poses[0].pose.position
            point = (pose.x, pose.y)
            
            # Save only if point changed (avoid duplicates)
            if not self.ref_x or self.distance((point[0], point[1]), (self.ref_x[-1], self.ref_y[-1])) > 0.005:
                self.ref_x.append(point[0])
                self.ref_y.append(point[1])

    def odom_callback(self, msg):
        """Store actual robot pose and compute TRUE geometry error"""
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
            
            # Calculate error only if we have enough reference points
            if len(self.ref_x) > 5:
                error = self.compute_true_cross_track_error(point)
                current_time = self.get_clock().now().nanoseconds / 1e9 - self.start_time
                
                self.time_stamps.append(current_time)
                self.errors.append(error)

    def compute_true_cross_track_error(self, point):
        """
        Calculate the shortest distance from the robot to the ENTIRE reference path.
        """
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
        
        # Step 2: Fine Search
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
        
        # Trim initial errors (skip first 20 points to ignore start-up jitter)
        trim_idx = min(len(self.errors), 20)
        valid_errors = self.errors[trim_idx:] if len(self.errors) > trim_idx else list(self.errors)
        valid_time = self.time_stamps[trim_idx:] if len(self.time_stamps) > trim_idx else list(self.time_stamps)

        # Remove a specific time window [ignore_time_from, ignore_time_to]
        if self.ignore_time_to > self.ignore_time_from and len(valid_time) == len(valid_errors):
            filtered = [
                (t, e) for (t, e) in zip(valid_time, valid_errors)
                if not (self.ignore_time_from <= t <= self.ignore_time_to)
            ]
            if filtered:
                valid_time, valid_errors = zip(*filtered)
                valid_time = list(valid_time)
                valid_errors = list(valid_errors)
        
        mean_error = np.mean(valid_errors) if valid_errors else 0.0
        max_error = np.max(valid_errors) if valid_errors else 0.0
        rmse = np.sqrt(np.mean(np.square(valid_errors))) if valid_errors else 0.0
        
        fig = plt.figure(figsize=(14, 10))
        fig.suptitle(f'Trajectory Tracking Result\nRMSE: {rmse:.4f}m | Mean: {mean_error:.4f}m', fontsize=18, fontweight='bold')

        # Plot 1: X-Y Path
        ax1 = fig.add_subplot(2, 1, 1)
        ax1.plot(self.ref_x, self.ref_y, 'b--', linewidth=2, label='Reference')
        ax1.plot(self.actual_x, self.actual_y, 'r-', linewidth=2, label='Actual (EKF)', alpha=0.8)
        
        # Mark Start/End
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
            # Mention ignored window if applicable
            if self.ignore_time_to > self.ignore_time_from:
                ax2.set_title(f'Tracking Error (Geometry Distance) — ignored {self.ignore_time_from:.0f}-{self.ignore_time_to:.0f}s')
            else:
                ax2.set_title('Tracking Error (Geometry Distance)')
            ax2.legend(); ax2.grid(True)
            
            y_max = max(0.1, max_error * 1.2)
            ax2.set_ylim(0, y_max)

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