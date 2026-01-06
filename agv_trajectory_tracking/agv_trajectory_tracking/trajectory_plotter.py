#!/usr/bin/env python3
"""
Real-time Trajectory Plotter with Matplotlib
Plots reference trajectory, actual trajectory, and tracking error

Author: Thuong Tran Dinh
Date: November 2, 2025
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
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
        self.declare_parameter('update_interval', 50)  # ms - Plot update rate (default: 50ms = 20Hz)
        self.declare_parameter('publish_rate', 10.0)   # Hz - Trajectory publish rate for reference
        
        self.max_history = self.get_parameter('max_history').value
        self.update_interval = self.get_parameter('update_interval').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # Auto-adjust update interval based on publish rate for real-time plotting
        # Update at least 2x faster than publish rate for smooth visualization
        recommended_interval = int(1000.0 / (self.publish_rate * 2))  # ms
        if self.update_interval > recommended_interval:
            self.update_interval = recommended_interval
            self.get_logger().info(f'⚡ Auto-adjusted plot update interval to {self.update_interval}ms for smooth real-time visualization')
        
        # Data storage
        self.ref_x = []
        self.ref_y = []
        self.actual_x = []
        self.actual_y = []
        self.time_stamps = []
        self.errors = []
        self.errors_x = []  # Sai số theo trục X
        self.errors_y = []  # Sai số theo trục Y
        self.vel_linear = []  # Vận tốc tuyến tính (m/s)
        self.vel_angular = []  # Vận tốc góc (rad/s)
        self.vel_time_stamps = []  # Timestamps cho vận tốc
        
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.last_ref_point = None
        self.last_actual_point = None
        
        # Ready flag - ensure subscribers are ready before data arrives
        self.is_ready = False
        self.data_received = False
        
        # Parameter for pose topic (default: /odometry/filtered for both sim and real)
        self.declare_parameter('pose_topic', '/odometry/filtered')
        self.declare_parameter('cmd_vel_topic', '/diff_cont/cmd_vel')
        pose_topic = self.get_parameter('pose_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        
        # Subscribers
        self.ref_path_sub = self.create_subscription(
            Path, '/trajectory',
            self.reference_callback, 10)
        
        # Subscribe to odometry topic (works for both simulation and real robot)
        self.odom_sub = self.create_subscription(
            Odometry, pose_topic,
            self.odom_callback, 10)
        
        # Subscribe to cmd_vel for velocity
        self.cmd_vel_sub = self.create_subscription(
            TwistStamped, cmd_vel_topic,
            self.cmd_vel_callback, 10)
        
        # Setup matplotlib EARLY to be ready for first data
        self.setup_plots()
        
        # Mark as ready after a short delay to ensure ROS2 connections established
        self.create_timer(0.5, self.mark_ready)
        
        self.get_logger().info('📊 Trajectory Plotter started!')
        self.get_logger().info(f'  Pose topic: {pose_topic}')
        self.get_logger().info(f'  Cmd vel topic: {cmd_vel_topic}')
        self.get_logger().info(f'  Max history: {self.max_history} points')
        self.get_logger().info(f'  Update interval: {self.update_interval} ms ({1000.0/self.update_interval:.1f} Hz)')
        self.get_logger().info(f'  Trajectory publish rate: {self.publish_rate} Hz')
        self.get_logger().info(f'  Real-time plotting: ✓ ENABLED')
        self.get_logger().info('⏳ Initializing subscribers...')
    
    def mark_ready(self):
        """Mark plotter as ready after initialization"""
        self.is_ready = True
        self.get_logger().info('✅ Plotter ready! Waiting for trajectory data...')
    
    def setup_plots(self):
        """Setup matplotlib figure with 6 subplots (2x3)"""
        plt.ion()  # Interactive mode
        self.fig = plt.figure(figsize=(20, 10))
        
        # Subplot 1: X-Y response vs reference (X over time)
        self.ax1 = self.fig.add_subplot(2, 3, 1)
        self.ax1.set_xlabel('Time (s)', fontsize=12)
        self.ax1.set_ylabel('X Position (m)', fontsize=12)
        self.ax1.set_title('X Position Response', fontsize=14, fontweight='bold')
        self.ax1.grid(True, alpha=0.3)
        
        self.ref_x_line, = self.ax1.plot([], [], 'b-', linewidth=2, label='X_ref', alpha=0.7)
        self.actual_x_line, = self.ax1.plot([], [], 'r-', linewidth=2, label='X_actual', alpha=0.8)
        self.ax1.legend(loc='upper right', fontsize=10)
        
        # Subplot 1B: Y over time (will be on same figure)
        self.ax1b = self.fig.add_subplot(2, 3, 2)
        self.ax1b.set_xlabel('Time (s)', fontsize=12)
        self.ax1b.set_ylabel('Y Position (m)', fontsize=12)
        self.ax1b.set_title('Y Position Response', fontsize=14, fontweight='bold')
        self.ax1b.grid(True, alpha=0.3)
        
        self.ref_y_line, = self.ax1b.plot([], [], 'b-', linewidth=2, label='Y_ref', alpha=0.7)
        self.actual_y_line, = self.ax1b.plot([], [], 'r-', linewidth=2, label='Y_actual', alpha=0.8)
        self.ax1b.legend(loc='upper right', fontsize=10)
        
        # Subplot 1C: Velocity (linear & angular)
        self.ax1c = self.fig.add_subplot(2, 3, 3)
        self.ax1c.set_xlabel('Time (s)', fontsize=12)
        self.ax1c.set_ylabel('Velocity', fontsize=12)
        self.ax1c.set_title('Commanded Velocity', fontsize=14, fontweight='bold')
        self.ax1c.grid(True, alpha=0.3)
        
        self.vel_linear_line, = self.ax1c.plot([], [], 'g-', linewidth=2, label='v_linear (m/s)')
        self.vel_angular_line, = self.ax1c.plot([], [], 'orange', linewidth=2, label='ω (rad/s)')
        self.ax1c.legend(loc='upper right', fontsize=10)
        
        # Subplot 2: Error X & Y over time
        self.ax2 = self.fig.add_subplot(2, 3, 4)
        self.ax2.set_xlabel('Time (s)', fontsize=12)
        self.ax2.set_ylabel('Error (m)', fontsize=12)
        self.ax2.set_title('Tracking Error (X & Y)', fontsize=14, fontweight='bold')
        self.ax2.grid(True, alpha=0.3)
        
        self.error_x_line, = self.ax2.plot([], [], 'r-', linewidth=2, label='Error X')
        self.error_y_line, = self.ax2.plot([], [], 'b-', linewidth=2, label='Error Y')
        self.ax2.legend(loc='upper right', fontsize=10)
        
        # Subplot 3: Robot trajectory (2D X-Y plot)
        self.ax3 = self.fig.add_subplot(2, 3, 5)
        self.ax3.set_xlabel('X Position (m)', fontsize=12)
        self.ax3.set_ylabel('Y Position (m)', fontsize=12)
        self.ax3.set_title('Robot Trajectory', fontsize=14, fontweight='bold')
        self.ax3.grid(True, alpha=0.3)
        self.ax3.set_aspect('equal')
        
        self.ref_trajectory_line, = self.ax3.plot([], [], 'b-', linewidth=2, label='Reference', alpha=0.7)
        self.actual_trajectory_line, = self.ax3.plot([], [], 'r-', linewidth=2, label='Actual', alpha=0.8)
        self.robot_marker, = self.ax3.plot([], [], 'ro', markersize=10, label='Robot Position')
        self.ax3.legend(loc='upper right', fontsize=10)
        
        # Add text for statistics
        self.stats_text = self.fig.text(0.5, 0.02, '', ha='center', fontsize=11,
                                        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        plt.tight_layout()
        plt.subplots_adjust(bottom=0.12)
    def reference_callback(self, msg):
        """Store reference trajectory"""
        if not self.is_ready:
            return
        
        if not self.data_received:
            self.data_received = True
            self.get_logger().info('📍 First trajectory data received! Starting visualization...')
        
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
    
    def odom_callback(self, msg):
        """Store actual robot trajectory from odometry and compute error"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self._process_robot_position(x, y)
    
    def pose_callback(self, msg):
        """Store actual robot trajectory from PoseWithCovarianceStamped and compute error"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self._process_robot_position(x, y)
    
    def cmd_vel_callback(self, msg):
        """Store commanded velocity from TwistStamped"""
        self.vel_linear.append(msg.twist.linear.x)
        self.vel_angular.append(msg.twist.angular.z)
        
        # Store timestamp aligned with message header
        current_time = self.get_clock().now().nanoseconds / 1e9 - self.start_time
        self.vel_time_stamps.append(current_time)
        
        # Limit history
        if len(self.vel_linear) > self.max_history:
            self.vel_linear.pop(0)
            self.vel_angular.pop(0)
            self.vel_time_stamps.pop(0)
    
    def _process_robot_position(self, x, y):
        """Process robot position and compute error"""
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
            
            # 🚀 Tính sai số theo trục X và Y (so với điểm gần nhất trên đường tham chiếu)
            if len(self.ref_x) > 1:
                # Tìm điểm gần nhất trên đường tham chiếu
                closest_ref_x, closest_ref_y = self.find_closest_reference_point(point)
                
                # Sai số theo từng trục
                error_x = x - closest_ref_x
                error_y = y - closest_ref_y
                
                # Sai số tổng (Euclidean distance)
                total_error = math.sqrt(error_x**2 + error_y**2)
                
                current_time = self.get_clock().now().nanoseconds / 1e9 - self.start_time
                self.time_stamps.append(current_time)
                self.errors_x.append(error_x)
                self.errors_y.append(error_y)
                self.errors.append(total_error)
                
                # Limit history
                if len(self.errors) > self.max_history:
                    self.time_stamps.pop(0)
                    self.errors_x.pop(0)
                    self.errors_y.pop(0)
                    self.errors.pop(0)
    
    def find_closest_reference_point(self, point):
        """Tìm điểm gần nhất trên đường tham chiếu"""
        px, py = point
        min_dist = float('inf')
        closest_x, closest_y = self.ref_x[0], self.ref_y[0]
        
        # Tìm điểm gần nhất trên từng đoạn đường
        for i in range(len(self.ref_x) - 1):
            x1, y1 = self.ref_x[i], self.ref_y[i]
            x2, y2 = self.ref_x[i + 1], self.ref_y[i + 1]
            
            # Vector từ điểm đầu đến điểm cuối của đoạn
            dx = x2 - x1
            dy = y2 - y1
            
            seg_length_sq = dx*dx + dy*dy
            
            if seg_length_sq < 1e-6:
                # Đoạn quá ngắn, dùng điểm đầu
                proj_x, proj_y = x1, y1
            else:
                # Tính hệ số t (chiếu điểm lên đoạn thẳng)
                t = max(0.0, min(1.0, ((px - x1)*dx + (py - y1)*dy) / seg_length_sq))
                proj_x = x1 + t * dx
                proj_y = y1 + t * dy
            
            # Khoảng cách từ điểm robot đến điểm chiếu
            dist = math.sqrt((px - proj_x)**2 + (py - proj_y)**2)
            
            if dist < min_dist:
                min_dist = dist
                closest_x = proj_x
                closest_y = proj_y
        
        return closest_x, closest_y
    
    def distance(self, p1, p2):
        """Euclidean distance"""
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
    
    def update_plot(self, frame):
        """Update plot data"""
        try:
            # Update X-Y response plot (X over time)
            if len(self.ref_x) > 0 and len(self.time_stamps) > 0:
                self.ref_x_line.set_data(self.time_stamps, self.ref_x[:len(self.time_stamps)])
            
            if len(self.actual_x) > 0 and len(self.time_stamps) > 0:
                self.actual_x_line.set_data(self.time_stamps, self.actual_x[:len(self.time_stamps)])
            
            # Auto-scale X plot
            if len(self.time_stamps) > 0:
                time_margin = 1.0
                self.ax1.set_xlim(0, max(self.time_stamps) + time_margin)
                if len(self.ref_x) > 0 or len(self.actual_x) > 0:
                    x_data = [x for x in self.ref_x[:len(self.time_stamps)]] + [x for x in self.actual_x[:len(self.time_stamps)]]
                    if x_data:
                        x_margin = max(0.5, (max(x_data) - min(x_data)) * 0.1)
                        self.ax1.set_ylim(min(x_data) - x_margin, max(x_data) + x_margin)
            
            # Update Y-position response plot (Y over time)
            if len(self.ref_y) > 0 and len(self.time_stamps) > 0:
                self.ref_y_line.set_data(self.time_stamps, self.ref_y[:len(self.time_stamps)])
            
            if len(self.actual_y) > 0 and len(self.time_stamps) > 0:
                self.actual_y_line.set_data(self.time_stamps, self.actual_y[:len(self.time_stamps)])
            
            # Auto-scale Y plot
            if len(self.time_stamps) > 0:
                time_margin = 1.0
                self.ax1b.set_xlim(0, max(self.time_stamps) + time_margin)
                if len(self.ref_y) > 0 or len(self.actual_y) > 0:
                    y_data = [y for y in self.ref_y[:len(self.time_stamps)]] + [y for y in self.actual_y[:len(self.time_stamps)]]
                    if y_data:
                        y_margin = max(0.5, (max(y_data) - min(y_data)) * 0.1)
                        self.ax1b.set_ylim(min(y_data) - y_margin, max(y_data) + y_margin)
            
            # Update velocity plot
            if len(self.vel_linear) > 0 and len(self.vel_angular) > 0:
                vel_time = list(range(len(self.vel_linear)))
                self.vel_linear_line.set_data(vel_time, self.vel_linear)
                self.vel_angular_line.set_data(vel_time, self.vel_angular)
                
                # Auto-scale velocity plot
                time_margin = 5.0
                all_vel = self.vel_linear + self.vel_angular
                if all_vel:
                    vel_max = max(abs(min(all_vel)), abs(max(all_vel)))
                    vel_margin = max(0.1, vel_max * 0.2)
                    self.ax1c.set_xlim(0, max(vel_time) + time_margin if vel_time else 1)
                    self.ax1c.set_ylim(-vel_max - vel_margin, vel_max + vel_margin)
            
            # Update error X & Y plot
            if len(self.errors_x) > 0:
                self.error_x_line.set_data(self.time_stamps, self.errors_x)
                self.error_y_line.set_data(self.time_stamps, self.errors_y)
                
                # Auto-scale error X & Y plot
                time_margin = 1.0
                all_errors_xy = self.errors_x + self.errors_y
                error_max = max(abs(min(all_errors_xy)), abs(max(all_errors_xy)))
                error_margin = max(0.1, error_max * 0.2)
                self.ax2.set_xlim(0, max(self.time_stamps) + time_margin)
                self.ax2.set_ylim(-error_max - error_margin, error_max + error_margin)
            
            # Update robot trajectory (2D XY plot)
            if len(self.actual_x) > 0 and len(self.actual_y) > 0:
                self.actual_trajectory_line.set_data(self.actual_x, self.actual_y)
                self.robot_marker.set_data([self.actual_x[-1]], [self.actual_y[-1]])
            
            if len(self.ref_x) > 0 and len(self.ref_y) > 0:
                self.ref_trajectory_line.set_data(self.ref_x, self.ref_y)
            
            # Auto-scale trajectory plot
            all_x = self.ref_x + self.actual_x
            all_y = self.ref_y + self.actual_y
            
            if len(all_x) > 0:
                margin = 1.0
                x_min, x_max = min(all_x) - margin, max(all_x) + margin
                y_min, y_max = min(all_y) - margin, max(all_y) + margin
                self.ax3.set_xlim(x_min, x_max)
                self.ax3.set_ylim(y_min, y_max)
            
            # Update statistics text
            if len(self.errors) > 0:
                max_error = max(self.errors)
                mean_error = np.mean(self.errors)
                mean_error_x = np.mean(self.errors_x) if self.errors_x else 0
                mean_error_y = np.mean(self.errors_y) if self.errors_y else 0
                current_error = self.errors[-1]
                
                stats_str = (
                    f'📊 Mean Error: {mean_error:.3f}m | '
                    f'Mean X: {mean_error_x:.3f}m | Mean Y: {mean_error_y:.3f}m | '
                    f'Current: {current_error:.3f}m | Max: {max_error:.3f}m'
                )
                self.stats_text.set_text(stats_str)
            
            return self.ref_x_line, self.actual_x_line, self.ref_y_line, self.actual_y_line, \
                   self.error_x_line, self.error_y_line, self.ref_trajectory_line, self.actual_trajectory_line, \
                   self.robot_marker, self.vel_linear_line, self.vel_angular_line
        except Exception as e:
            self.get_logger().error(f'Error in update_plot: {e}')
            return self.ref_x_line, self.actual_x_line, self.ref_y_line, self.actual_y_line, \
                   self.error_x_line, self.error_y_line, self.ref_trajectory_line, self.actual_trajectory_line, \
                   self.robot_marker, self.vel_linear_line, self.vel_angular_line
    
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