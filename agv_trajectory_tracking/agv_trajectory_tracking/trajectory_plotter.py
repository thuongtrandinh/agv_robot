#!/usr/bin/env python3
"""
Real-Time Trajectory Plotter with Live Visualization
- Real-time robot position display
- X and Y error tracking
- Linear and Angular velocity plotting
- Auto-saves results to: ~/results/

Author: Thuong Tran Dinh
Updated: December 26, 2025
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import TwistStamped, Twist
import tf2_ros
import matplotlib
matplotlib.use('TkAgg')  # Use TkAgg backend for real-time display
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import math
import os
from datetime import datetime
import threading


class RealTimeTrajectoryPlotter(Node):
    def __init__(self):
        super().__init__('trajectory_plotter')
        
        # Parameters
        self.declare_parameter('max_history', 20000)
        self.declare_parameter('update_rate', 10.0)  # Hz for plot update
        self.max_history = self.get_parameter('max_history').value
        self.update_rate = self.get_parameter('update_rate').value
        
        # --- SAVE PATH: ~/results ---
        user_home = os.path.expanduser('~')
        self.save_directory = os.path.join(user_home, 'results')

        # Data storage
        self.ref_x = []
        self.ref_y = []
        self.actual_x = []
        self.actual_y = []
        self.time_stamps = []
        
        # Error tracking (X and Y separately)
        self.errors_x = []
        self.errors_y = []
        self.errors_total = []
        
        # Velocity tracking
        self.linear_vel = []
        self.angular_vel = []
        self.vel_time = []
        
        self.start_time = None
        self.last_ref_point = None
        self.last_actual_point = None
        self.current_ref_x = 0.0
        self.current_ref_y = 0.0
        
        # Lock for thread safety
        self.data_lock = threading.Lock()
        
        # Subscribers
        self.ref_path_sub = self.create_subscription(
            Path, '/trajectory', self.reference_callback, 10)
        
        # Subscribe to cmd_vel for velocity data
        self.cmd_vel_sub = self.create_subscription(
            TwistStamped, '/diff_cont/cmd_vel', self.cmd_vel_callback, 10)
        
        # Also try regular Twist message
        self.cmd_vel_sub2 = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_twist_callback, 10)
        
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
        
        # Setup matplotlib figure for real-time display
        self.setup_realtime_plot()
        
        self.get_logger().info('📥 Real-Time Plotter Ready!')
        self.get_logger().info(f'   📁 Save Folder: {self.save_directory}')
        self.get_logger().info(f'   📊 Features: Position, X/Y Errors, Velocity')

    def setup_realtime_plot(self):
        """Setup matplotlib figure with 4 subplots for real-time visualization"""
        plt.ion()  # Enable interactive mode
        
        self.fig = plt.figure(figsize=(16, 10))
        self.fig.suptitle('🤖 Real-Time Trajectory Tracking Monitor', fontsize=16, fontweight='bold')
        
        # Subplot 1: Trajectory (Top-left)
        self.ax_traj = self.fig.add_subplot(2, 2, 1)
        self.ax_traj.set_xlabel('X (m)')
        self.ax_traj.set_ylabel('Y (m)')
        self.ax_traj.set_title('📍 Robot Position')
        self.ax_traj.grid(True, alpha=0.3)
        self.ax_traj.set_aspect('equal')
        
        # Initialize trajectory lines
        self.line_ref, = self.ax_traj.plot([], [], 'b--', linewidth=2, label='Reference')
        self.line_actual, = self.ax_traj.plot([], [], 'r-', linewidth=2, label='Actual', alpha=0.8)
        self.point_current, = self.ax_traj.plot([], [], 'go', markersize=12, label='Current')
        self.ax_traj.legend(loc='upper right')
        
        # Subplot 2: X and Y Errors (Top-right)
        self.ax_error = self.fig.add_subplot(2, 2, 2)
        self.ax_error.set_xlabel('Time (s)')
        self.ax_error.set_ylabel('Error (m)')
        self.ax_error.set_title('📐 Tracking Errors (X, Y)')
        self.ax_error.grid(True, alpha=0.3)
        
        self.line_err_x, = self.ax_error.plot([], [], 'r-', linewidth=1.5, label='Error X')
        self.line_err_y, = self.ax_error.plot([], [], 'b-', linewidth=1.5, label='Error Y')
        self.line_err_total, = self.ax_error.plot([], [], 'k--', linewidth=1, label='Total Error', alpha=0.7)
        self.ax_error.legend(loc='upper right')
        
        # Subplot 3: Linear Velocity (Bottom-left)
        self.ax_lin_vel = self.fig.add_subplot(2, 2, 3)
        self.ax_lin_vel.set_xlabel('Time (s)')
        self.ax_lin_vel.set_ylabel('Linear Velocity (m/s)')
        self.ax_lin_vel.set_title('🚗 Linear Velocity')
        self.ax_lin_vel.grid(True, alpha=0.3)
        
        self.line_lin_vel, = self.ax_lin_vel.plot([], [], 'g-', linewidth=2)
        self.ax_lin_vel.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
        
        # Subplot 4: Angular Velocity (Bottom-right)
        self.ax_ang_vel = self.fig.add_subplot(2, 2, 4)
        self.ax_ang_vel.set_xlabel('Time (s)')
        self.ax_ang_vel.set_ylabel('Angular Velocity (rad/s)')
        self.ax_ang_vel.set_title('🔄 Angular Velocity')
        self.ax_ang_vel.grid(True, alpha=0.3)
        
        self.line_ang_vel, = self.ax_ang_vel.plot([], [], 'm-', linewidth=2)
        self.ax_ang_vel.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
        
        plt.tight_layout()
        
        # Create animation for real-time update
        self.ani = FuncAnimation(
            self.fig, self.update_plot,
            interval=int(1000 / self.update_rate),  # ms
            blit=False,
            cache_frame_data=False
        )
        
        plt.show(block=False)
        plt.pause(0.1)

    def reference_callback(self, msg):
        """Store reference trajectory points"""
        if len(msg.poses) > 0:
            pose = msg.poses[0].pose.position
            point = (pose.x, pose.y)
            
            with self.data_lock:
                self.current_ref_x = pose.x
                self.current_ref_y = pose.y
                
                # Save only if point changed
                if not self.ref_x or self.distance((point[0], point[1]), (self.ref_x[-1], self.ref_y[-1])) > 0.005:
                    self.ref_x.append(point[0])
                    self.ref_y.append(point[1])

    def cmd_vel_callback(self, msg):
        """Store velocity commands (TwistStamped)"""
        if self.start_time is None:
            return
            
        current_time = self.get_clock().now().nanoseconds / 1e9 - self.start_time
        
        with self.data_lock:
            self.linear_vel.append(msg.twist.linear.x)
            self.angular_vel.append(msg.twist.angular.z)
            self.vel_time.append(current_time)
            
            # Limit history
            if len(self.linear_vel) > self.max_history:
                self.linear_vel = self.linear_vel[-self.max_history:]
                self.angular_vel = self.angular_vel[-self.max_history:]
                self.vel_time = self.vel_time[-self.max_history:]

    def cmd_vel_twist_callback(self, msg):
        """Store velocity commands (regular Twist)"""
        if self.start_time is None:
            return
            
        current_time = self.get_clock().now().nanoseconds / 1e9 - self.start_time
        
        with self.data_lock:
            self.linear_vel.append(msg.linear.x)
            self.angular_vel.append(msg.angular.z)
            self.vel_time.append(current_time)

    def tf_sample_callback(self):
        """Sample TF and compute X/Y errors"""
        if self.start_time is None:
            self.start_time = self.get_clock().now().nanoseconds / 1e9

        try:
            trans = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time())
        except Exception:
            return

        x = trans.transform.translation.x
        y = trans.transform.translation.y
        point = (x, y)
        current_time = self.get_clock().now().nanoseconds / 1e9 - self.start_time

        with self.data_lock:
            if self.last_actual_point is None or \
               self.distance(point, self.last_actual_point) > 0.01:
                self.actual_x.append(x)
                self.actual_y.append(y)
                self.last_actual_point = point

                if len(self.ref_x) > 5:
                    # Find closest reference point
                    closest_ref = self.find_closest_reference_point(point)
                    
                    # Calculate X and Y errors separately
                    error_x = x - closest_ref[0]
                    error_y = y - closest_ref[1]
                    error_total = math.sqrt(error_x**2 + error_y**2)
                    
                    self.errors_x.append(error_x)
                    self.errors_y.append(error_y)
                    self.errors_total.append(error_total)
                    self.time_stamps.append(current_time)

    def find_closest_reference_point(self, point):
        """Find the closest point on reference trajectory"""
        px, py = point
        min_dist = float('inf')
        closest = (self.ref_x[0], self.ref_y[0])
        
        for i in range(len(self.ref_x) - 1):
            x1, y1 = self.ref_x[i], self.ref_y[i]
            x2, y2 = self.ref_x[i+1], self.ref_y[i+1]
            
            dx = x2 - x1
            dy = y2 - y1
            seg_len_sq = dx*dx + dy*dy
            
            if seg_len_sq < 1e-6:
                closest_x, closest_y = x1, y1
            else:
                t = max(0.0, min(1.0, ((px - x1)*dx + (py - y1)*dy) / seg_len_sq))
                closest_x = x1 + t * dx
                closest_y = y1 + t * dy
            
            dist = math.sqrt((px - closest_x)**2 + (py - closest_y)**2)
            if dist < min_dist:
                min_dist = dist
                closest = (closest_x, closest_y)
        
        return closest

    def distance(self, p1, p2):
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def update_plot(self, frame):
        """Update real-time plot"""
        with self.data_lock:
            # Update trajectory plot
            if self.ref_x:
                self.line_ref.set_data(self.ref_x, self.ref_y)
            if self.actual_x:
                self.line_actual.set_data(self.actual_x, self.actual_y)
                self.point_current.set_data([self.actual_x[-1]], [self.actual_y[-1]])
            
            # Auto-scale trajectory axes
            if self.ref_x and self.actual_x:
                all_x = self.ref_x + self.actual_x
                all_y = self.ref_y + self.actual_y
                margin = 0.5
                self.ax_traj.set_xlim(min(all_x) - margin, max(all_x) + margin)
                self.ax_traj.set_ylim(min(all_y) - margin, max(all_y) + margin)
            
            # Update error plot
            if self.time_stamps and self.errors_x:
                self.line_err_x.set_data(self.time_stamps, self.errors_x)
                self.line_err_y.set_data(self.time_stamps, self.errors_y)
                self.line_err_total.set_data(self.time_stamps, self.errors_total)
                
                self.ax_error.set_xlim(0, max(self.time_stamps) + 1)
                max_err = max(max(abs(e) for e in self.errors_x + self.errors_y), 0.05)
                self.ax_error.set_ylim(-max_err * 1.2, max_err * 1.2)
            
            # Update velocity plots
            if self.vel_time and self.linear_vel:
                self.line_lin_vel.set_data(self.vel_time, self.linear_vel)
                self.ax_lin_vel.set_xlim(0, max(self.vel_time) + 1)
                max_lin = max(abs(v) for v in self.linear_vel) if self.linear_vel else 0.5
                self.ax_lin_vel.set_ylim(-0.1, max_lin * 1.2 + 0.1)
            
            if self.vel_time and self.angular_vel:
                self.line_ang_vel.set_data(self.vel_time, self.angular_vel)
                self.ax_ang_vel.set_xlim(0, max(self.vel_time) + 1)
                max_ang = max(abs(v) for v in self.angular_vel) if self.angular_vel else 1.0
                self.ax_ang_vel.set_ylim(-max_ang * 1.2 - 0.1, max_ang * 1.2 + 0.1)
        
        # Update figure title with current stats
        if self.errors_total:
            rmse = np.sqrt(np.mean(np.square(self.errors_total)))
            self.fig.suptitle(
                f'🤖 Real-Time Trajectory Tracking | RMSE: {rmse:.4f}m | Points: {len(self.actual_x)}',
                fontsize=14, fontweight='bold'
            )
        
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
        
        return [self.line_ref, self.line_actual, self.point_current,
                self.line_err_x, self.line_err_y, self.line_err_total,
                self.line_lin_vel, self.line_ang_vel]

    def save_results(self):
        """Generate final report and save to ~/results"""
        if not self.actual_x:
            self.get_logger().warn("⚠️ No data collected to save!")
            return

        print("\n📊 Generating Final Trajectory Report...")
        
        # Create save directory
        if not os.path.exists(self.save_directory):
            try:
                os.makedirs(self.save_directory)
                print(f"📁 Created directory: {self.save_directory}")
            except OSError as e:
                print(f"⚠️ Error creating directory: {e}")
                self.save_directory = os.getcwd()
        
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        
        # Calculate statistics
        trim = min(len(self.errors_total), 20)
        valid_errors = self.errors_total[trim:] if len(self.errors_total) > trim else self.errors_total
        valid_errors_x = self.errors_x[trim:] if len(self.errors_x) > trim else self.errors_x
        valid_errors_y = self.errors_y[trim:] if len(self.errors_y) > trim else self.errors_y
        valid_time = self.time_stamps[trim:] if len(self.time_stamps) > trim else self.time_stamps
        
        mean_error = np.mean(valid_errors) if valid_errors else 0.0
        max_error = np.max(valid_errors) if valid_errors else 0.0
        rmse = np.sqrt(np.mean(np.square(valid_errors))) if valid_errors else 0.0
        rmse_x = np.sqrt(np.mean(np.square(valid_errors_x))) if valid_errors_x else 0.0
        rmse_y = np.sqrt(np.mean(np.square(valid_errors_y))) if valid_errors_y else 0.0
        
        # === FIGURE 1: Comprehensive Report ===
        fig1 = plt.figure(figsize=(16, 12))
        fig1.suptitle(
            f'Trajectory Tracking Report - {timestamp}\n'
            f'RMSE: {rmse:.4f}m | RMSE_X: {rmse_x:.4f}m | RMSE_Y: {rmse_y:.4f}m',
            fontsize=16, fontweight='bold'
        )
        
        # Plot 1: Trajectory
        ax1 = fig1.add_subplot(2, 2, 1)
        ax1.plot(self.ref_x, self.ref_y, 'b--', linewidth=2, label='Reference')
        ax1.plot(self.actual_x, self.actual_y, 'r-', linewidth=2, label='Actual', alpha=0.8)
        if self.actual_x:
            ax1.plot(self.actual_x[0], self.actual_y[0], 'go', markersize=10, label='Start')
            ax1.plot(self.actual_x[-1], self.actual_y[-1], 'r^', markersize=10, label='End')
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_title('📍 Trajectory Path')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        ax1.axis('equal')
        
        # Plot 2: X and Y Errors
        ax2 = fig1.add_subplot(2, 2, 2)
        if valid_time and valid_errors_x:
            ax2.plot(valid_time, valid_errors_x, 'r-', linewidth=1.5, label=f'Error X (RMSE={rmse_x:.4f}m)', alpha=0.8)
            ax2.plot(valid_time, valid_errors_y, 'b-', linewidth=1.5, label=f'Error Y (RMSE={rmse_y:.4f}m)', alpha=0.8)
            ax2.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
            ax2.fill_between(valid_time, valid_errors_x, 0, color='red', alpha=0.1)
            ax2.fill_between(valid_time, valid_errors_y, 0, color='blue', alpha=0.1)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Error (m)')
        ax2.set_title('📐 X and Y Tracking Errors')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # Plot 3: Total Error
        ax3 = fig1.add_subplot(2, 2, 3)
        if valid_time and valid_errors:
            ax3.plot(valid_time, valid_errors, 'k-', linewidth=1.5)
            ax3.axhline(y=mean_error, color='r', linestyle='--', label=f'Mean: {mean_error:.4f}m')
            ax3.axhline(y=max_error, color='orange', linestyle=':', label=f'Max: {max_error:.4f}m')
            ax3.fill_between(valid_time, 0, valid_errors, color='green', alpha=0.2)
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Cross-Track Error (m)')
        ax3.set_title(f'📏 Total Error (RMSE: {rmse:.4f}m)')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        ax3.set_ylim(0, max(0.1, max_error * 1.3))
        
        # Plot 4: Velocity
        ax4 = fig1.add_subplot(2, 2, 4)
        if self.vel_time and self.linear_vel:
            ax4.plot(self.vel_time, self.linear_vel, 'g-', linewidth=1.5, label='Linear (m/s)')
            ax4.plot(self.vel_time, self.angular_vel, 'm-', linewidth=1.5, label='Angular (rad/s)', alpha=0.8)
            ax4.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Velocity')
        ax4.set_title('🚗 Robot Velocity Commands')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        # Save comprehensive report
        filename1 = f"tracking_report_{timestamp}.png"
        save_path1 = os.path.join(self.save_directory, filename1)
        plt.savefig(save_path1, dpi=150)
        plt.close(fig1)
        
        print(f"\033[92m✅ Report saved:\n   👉 {save_path1}\033[0m")
        
        # === FIGURE 2: Velocity Only ===
        fig2 = plt.figure(figsize=(12, 6))
        fig2.suptitle(f'Velocity Profile - {timestamp}', fontsize=14, fontweight='bold')
        
        ax_v1 = fig2.add_subplot(1, 2, 1)
        if self.vel_time and self.linear_vel:
            ax_v1.plot(self.vel_time, self.linear_vel, 'g-', linewidth=2)
            ax_v1.axhline(y=np.mean(self.linear_vel), color='r', linestyle='--', 
                         label=f'Mean: {np.mean(self.linear_vel):.3f} m/s')
        ax_v1.set_xlabel('Time (s)')
        ax_v1.set_ylabel('Linear Velocity (m/s)')
        ax_v1.set_title('Linear Velocity')
        ax_v1.legend()
        ax_v1.grid(True, alpha=0.3)
        
        ax_v2 = fig2.add_subplot(1, 2, 2)
        if self.vel_time and self.angular_vel:
            ax_v2.plot(self.vel_time, self.angular_vel, 'm-', linewidth=2)
            ax_v2.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
        ax_v2.set_xlabel('Time (s)')
        ax_v2.set_ylabel('Angular Velocity (rad/s)')
        ax_v2.set_title('Angular Velocity')
        ax_v2.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        filename2 = f"velocity_profile_{timestamp}.png"
        save_path2 = os.path.join(self.save_directory, filename2)
        plt.savefig(save_path2, dpi=150)
        plt.close(fig2)
        
        print(f"\033[92m✅ Velocity plot saved:\n   👉 {save_path2}\033[0m")
        
        # Print summary statistics
        print("\n" + "="*50)
        print("📊 TRACKING SUMMARY")
        print("="*50)
        print(f"  Total Points:     {len(self.actual_x)}")
        print(f"  Duration:         {self.time_stamps[-1]:.1f} s" if self.time_stamps else "  Duration: N/A")
        print(f"  RMSE Total:       {rmse:.4f} m")
        print(f"  RMSE X:           {rmse_x:.4f} m")
        print(f"  RMSE Y:           {rmse_y:.4f} m")
        print(f"  Mean Error:       {mean_error:.4f} m")
        print(f"  Max Error:        {max_error:.4f} m")
        if self.linear_vel:
            print(f"  Avg Linear Vel:   {np.mean(self.linear_vel):.3f} m/s")
            print(f"  Max Linear Vel:   {np.max(self.linear_vel):.3f} m/s")
        print("="*50)


def main(args=None):
    rclpy.init(args=args)
    plotter = RealTimeTrajectoryPlotter()
    
    try:
        rclpy.spin(plotter)
    except KeyboardInterrupt:
        print("\n⏹️ Stopping plotter...")
    finally:
        plotter.save_results()
        plt.close('all')
        if rclpy.ok():
            plotter.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()