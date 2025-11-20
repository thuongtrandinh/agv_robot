#!/usr/bin/env python3
"""
Real-time Trajectory Plotter with Matplotlib
Plots reference trajectory, actual trajectory, and tracking error

Author: Thuong Tran Dinh
Date: November 2, 2025
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle
import numpy as np
import math
import threading


class TrajectoryPlotter(Node):
    """Real-time trajectory visualization with matplotlib"""
    
    def __init__(self):
        super().__init__('trajectory_plotter')
        
        # Parameters
        self.declare_parameter('max_history', 2000)
        self.declare_parameter('update_interval', 20)  # ms
        
        self.max_history = self.get_parameter('max_history').value
        self.update_interval = self.get_parameter('update_interval').value
        
        # Data storage
        self.ref_x = []
        self.ref_y = []
        self.actual_x = []
        self.actual_y = []
        self.time_stamps = []
        self.errors = []
        
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.last_ref_point = None
        self.last_actual_point = None
        
        # Subscribers
        self.ref_path_sub = self.create_subscription(
            Path, '/trajectory',
            self.reference_callback, 10)
        
        self.pose_sub = self.create_subscription(
            Odometry, '/diff_cont/odom',
            self.pose_callback, 10)
        
        # Setup matplotlib
        self.setup_plots()
        
        self.get_logger().info('📊 Trajectory Plotter started!')
        self.get_logger().info(f'  Max history: {self.max_history} points')
        self.get_logger().info(f'  Update interval: {self.update_interval} ms')
    
    def setup_plots(self):
        """Setup matplotlib figure with 2 subplots"""
        plt.ion()  # Interactive mode
        self.fig = plt.figure(figsize=(14, 6))
        
        # Subplot 1: X-Y trajectory plot
        self.ax1 = self.fig.add_subplot(1, 2, 1)
        self.ax1.set_xlabel('X (m)', fontsize=12)
        self.ax1.set_ylabel('Y (m)', fontsize=12)
        self.ax1.set_title('Trajectory Tracking', fontsize=14, fontweight='bold')
        self.ax1.grid(True, alpha=0.3)
        self.ax1.set_aspect('equal')
        
        # Initialize lines
        self.ref_line, = self.ax1.plot([], [], 'b-', linewidth=2, label='Reference', alpha=0.7)
        self.actual_line, = self.ax1.plot([], [], 'r-', linewidth=2, label='Actual', alpha=0.8)
        self.robot_marker, = self.ax1.plot([], [], 'ro', markersize=10, label='Robot')
        
        self.ax1.legend(loc='upper right', fontsize=10)
        
        # Subplot 2: Tracking error over time
        self.ax2 = self.fig.add_subplot(1, 2, 2)
        self.ax2.set_xlabel('Time (s)', fontsize=12)
        self.ax2.set_ylabel('Tracking Error (m)', fontsize=12)
        self.ax2.set_title('Tracking Error', fontsize=14, fontweight='bold')
        self.ax2.grid(True, alpha=0.3)
        
        self.error_line, = self.ax2.plot([], [], 'g-', linewidth=2)
        self.error_mean_line = self.ax2.axhline(y=0, color='orange', linestyle='--', 
                                                 linewidth=2, label='Mean Error')
        
        self.ax2.legend(loc='upper right', fontsize=10)
        
        # Add text for statistics
        self.stats_text = self.fig.text(0.5, 0.02, '', ha='center', fontsize=11,
                                        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        plt.tight_layout()
        plt.subplots_adjust(bottom=0.1)
    
    def reference_callback(self, msg):
        """Store reference trajectory"""
        if len(msg.poses) > 0:
            # Get first point (closest to robot)
            pose = msg.poses[0].pose.position
            point = (pose.x, pose.y)
            
            # Add if different from last point (avoid duplicates)
            if self.last_ref_point is None or \
               self.distance(point, self.last_ref_point) > 0.05:
                self.ref_x.append(pose.x)
                self.ref_y.append(pose.y)
                self.last_ref_point = point
                
                # Limit history
                if len(self.ref_x) > self.max_history:
                    self.ref_x.pop(0)
                    self.ref_y.pop(0)
    
    def pose_callback(self, msg):
        """Store actual robot trajectory and compute error from /odom"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        point = (x, y)

        # Add if different from last point
        if self.last_actual_point is None or \
           self.distance(point, self.last_actual_point) > 0.05:
            self.actual_x.append(x)
            self.actual_y.append(y)
            self.last_actual_point = point
            # Limit history
            if len(self.actual_x) > self.max_history:
                self.actual_x.pop(0)
                self.actual_y.pop(0)
            # Compute cross-track error
            if len(self.ref_x) > 1:
                cross_track_error = self.compute_cross_track_error(point)
                current_time = self.get_clock().now().nanoseconds / 1e9 - self.start_time
                self.time_stamps.append(current_time)
                self.errors.append(cross_track_error)
                if len(self.errors) > self.max_history:
                    self.time_stamps.pop(0)
                    self.errors.pop(0)
    
    def compute_cross_track_error(self, point):
        """Compute perpendicular distance from point to closest path segment
        
        🚀 This is the CORRECT metric for trajectory tracking!
        Returns the shortest perpendicular distance to the path.
        """
        px, py = point
        min_cross_track = float('inf')
        
        # Check distance to each path segment
        for i in range(len(self.ref_x) - 1):
            x1, y1 = self.ref_x[i], self.ref_y[i]
            x2, y2 = self.ref_x[i + 1], self.ref_y[i + 1]
            
            # Vector from segment start to end
            dx = x2 - x1
            dy = y2 - y1
            
            # Length of segment
            seg_length_sq = dx*dx + dy*dy
            
            if seg_length_sq < 1e-6:  # Degenerate segment
                # Just compute distance to point
                dist = math.sqrt((px - x1)**2 + (py - y1)**2)
            else:
                # Parameter t represents projection of point onto line segment
                # t=0 means projection at (x1,y1), t=1 means at (x2,y2)
                t = max(0.0, min(1.0, ((px - x1)*dx + (py - y1)*dy) / seg_length_sq))
                
                # Find closest point on segment
                closest_x = x1 + t * dx
                closest_y = y1 + t * dy
                
                # Distance to closest point (this is cross-track error)
                dist = math.sqrt((px - closest_x)**2 + (py - closest_y)**2)
            
            if dist < min_cross_track:
                min_cross_track = dist
        
        return min_cross_track
    
    def distance(self, p1, p2):
        """Euclidean distance"""
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
    
    def update_plot(self, frame):
        """Update plot data"""
        # Update trajectory plot
        if len(self.ref_x) > 0:
            self.ref_line.set_data(self.ref_x, self.ref_y)
        
        if len(self.actual_x) > 0:
            self.actual_line.set_data(self.actual_x, self.actual_y)
            # Update robot position marker
            self.robot_marker.set_data([self.actual_x[-1]], [self.actual_y[-1]])
        
        # Auto-scale axes
        all_x = self.ref_x + self.actual_x
        all_y = self.ref_y + self.actual_y
        
        if len(all_x) > 0:
            margin = 1.0
            x_min, x_max = min(all_x) - margin, max(all_x) + margin
            y_min, y_max = min(all_y) - margin, max(all_y) + margin
            self.ax1.set_xlim(x_min, x_max)
            self.ax1.set_ylim(y_min, y_max)
        
        # Update error plot
        if len(self.errors) > 0:
            self.error_line.set_data(self.time_stamps, self.errors)
            
            # Update mean error line
            mean_error = np.mean(self.errors)
            self.error_mean_line.set_ydata([mean_error, mean_error])
            
            # Auto-scale error plot
            time_margin = 1.0
            error_margin = 0.5
            self.ax2.set_xlim(0, max(self.time_stamps) + time_margin)
            self.ax2.set_ylim(0, max(self.errors) + error_margin)
            
            # Update statistics text
            max_error = max(self.errors)
            min_error = min(self.errors)
            current_error = self.errors[-1]
            
            stats_str = (
                f'📊 Statistics: '
                f'Mean Error: {mean_error:.3f}m | '
                f'Current: {current_error:.3f}m | '
                f'Max: {max_error:.3f}m | '
                f'Min: {min_error:.3f}m | '
                f'Points: Ref={len(self.ref_x)}, Actual={len(self.actual_x)}'
            )
            self.stats_text.set_text(stats_str)
        
        return self.ref_line, self.actual_line, self.robot_marker, self.error_line
    
    def run(self):
        """Run the plotter with animation"""
        # Start animation
        anim = animation.FuncAnimation(
            self.fig, 
            self.update_plot,
            interval=self.update_interval,
            blit=False,
            cache_frame_data=False
        )
        
        try:
            plt.show(block=False)
            
            # Keep ROS spinning in this thread
            while rclpy.ok() and plt.fignum_exists(self.fig.number):
                rclpy.spin_once(self, timeout_sec=0.01)
                plt.pause(0.001)
                
        except KeyboardInterrupt:
            self.get_logger().info('🛑 Shutting down plotter...')
        finally:
            plt.close('all')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        plotter = TrajectoryPlotter()
        plotter.run()
    except KeyboardInterrupt:
        print('\n🛑 Keyboard interrupt!')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()