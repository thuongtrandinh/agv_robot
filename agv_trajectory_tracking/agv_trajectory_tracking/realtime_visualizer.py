#!/usr/bin/env python3
"""
Real-time Trajectory Visualizer for AGV
Shows live robot position, trajectory, and errors using matplotlib

Author: Thuong Tran Dinh
Date: December 26, 2025
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy as np
import math
from collections import deque
import threading
import time

# Import tf_transformations for quaternion to euler conversion
try:
    from tf_transformations import euler_from_quaternion
    USE_TF2 = True
except ImportError:
    USE_TF2 = False


def quaternion_to_yaw(qx, qy, qz, qw):
    """Extract yaw from quaternion using tf2 or manual calculation."""
    if USE_TF2:
        _, _, yaw = euler_from_quaternion([qx, qy, qz, qw])
        return yaw
    else:
        t3 = 2.0 * (qw * qz + qx * qy)
        t4 = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(t3, t4)


class RealtimeVisualizer(Node):
    """Real-time visualization node"""
    
    def __init__(self):
        super().__init__('realtime_visualizer')
        
        # Parameters
        self.declare_parameter('max_history', 1000)
        self.max_history = self.get_parameter('max_history').value
        
        # Data storage
        self.ref_x = []
        self.ref_y = []
        self.actual_x = deque(maxlen=self.max_history)
        self.actual_y = deque(maxlen=self.max_history)
        self.error_x = deque(maxlen=self.max_history)
        self.error_y = deque(maxlen=self.max_history)
        self.error_dist = deque(maxlen=self.max_history)
        self.time_stamps = deque(maxlen=self.max_history)
        
        # Robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        
        self.start_time = None
        self.data_lock = threading.Lock()
        
        # Subscribers
        self.path_sub = self.create_subscription(
            Path, '/trajectory', self.path_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10)
        
        self.get_logger().info('Real-time Visualizer started!')
        
    def path_callback(self, msg):
        """Receive reference trajectory"""
        with self.data_lock:
            if len(msg.poses) > 0:
                self.ref_x = [p.pose.position.x for p in msg.poses]
                self.ref_y = [p.pose.position.y for p in msg.poses]
    
    def odom_callback(self, msg):
        """Update robot position and calculate errors"""
        if self.start_time is None:
            self.start_time = self.get_clock().now().nanoseconds / 1e9
        
        with self.data_lock:
            self.robot_x = msg.pose.pose.position.x
            self.robot_y = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            self.robot_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
            
            self.actual_x.append(self.robot_x)
            self.actual_y.append(self.robot_y)
            
            current_time = self.get_clock().now().nanoseconds / 1e9 - self.start_time
            self.time_stamps.append(current_time)
            
            if len(self.ref_x) > 0:
                min_dist, closest_x, closest_y = self.find_closest_ref()
                self.error_x.append(self.robot_x - closest_x)
                self.error_y.append(self.robot_y - closest_y)
                self.error_dist.append(min_dist)
    
    def find_closest_ref(self):
        """Find closest point on reference trajectory"""
        min_dist = float('inf')
        closest_x, closest_y = 0.0, 0.0
        
        for i in range(len(self.ref_x)):
            dx = self.robot_x - self.ref_x[i]
            dy = self.robot_y - self.ref_y[i]
            dist = math.sqrt(dx*dx + dy*dy)
            if dist < min_dist:
                min_dist = dist
                closest_x = self.ref_x[i]
                closest_y = self.ref_y[i]
        
        return min_dist, closest_x, closest_y
    
    def get_plot_data(self):
        """Get current data for plotting (thread-safe)"""
        with self.data_lock:
            return {
                'ref_x': list(self.ref_x),
                'ref_y': list(self.ref_y),
                'actual_x': list(self.actual_x),
                'actual_y': list(self.actual_y),
                'error_x': list(self.error_x),
                'error_y': list(self.error_y),
                'error_dist': list(self.error_dist),
                'time': list(self.time_stamps),
                'robot_x': self.robot_x,
                'robot_y': self.robot_y,
                'robot_yaw': self.robot_yaw
            }


def run_visualization(node):
    """Run matplotlib visualization in main thread"""
    
    plt.ion()
    fig = plt.figure(figsize=(14, 8))
    fig.suptitle('AGV Real-time Trajectory Tracking', fontsize=14, fontweight='bold')
    
    ax1 = fig.add_subplot(2, 2, (1, 3))
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title('Trajectory Path')
    ax1.grid(True, alpha=0.3)
    ax1.set_aspect('equal')
    
    ax2 = fig.add_subplot(2, 2, 2)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Error (m)')
    ax2.set_title('Position Error (X, Y)')
    ax2.grid(True, alpha=0.3)
    
    ax3 = fig.add_subplot(2, 2, 4)
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Distance (m)')
    ax3.set_title('Cross-track Error')
    ax3.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show(block=False)
    
    while rclpy.ok():
        try:
            data = node.get_plot_data()
            
            # Plot 1: Trajectory
            ax1.cla()
            ax1.set_xlabel('X (m)')
            ax1.set_ylabel('Y (m)')
            ax1.set_title('Trajectory Path')
            ax1.grid(True, alpha=0.3)
            
            if len(data['ref_x']) > 0:
                ax1.plot(data['ref_x'], data['ref_y'], 'b--', linewidth=2, 
                        label=f'Reference ({len(data["ref_x"])} pts)', alpha=0.7)
            
            if len(data['actual_x']) > 1:
                ax1.plot(data['actual_x'], data['actual_y'], 'r-', linewidth=2, 
                        label=f'Actual ({len(data["actual_x"])} pts)', alpha=0.9)
                ax1.plot(data['actual_x'][0], data['actual_y'][0], 'go', 
                        markersize=10, label='Start', zorder=5)
            
            robot_x = data['robot_x']
            robot_y = data['robot_y']
            robot_yaw = data['robot_yaw']
            
            if len(data['actual_x']) > 0:
                ax1.plot(robot_x, robot_y, 'ko', markersize=12, label='Current', zorder=10)
                arrow_len = 0.15
                ax1.arrow(robot_x, robot_y, 
                         arrow_len * math.cos(robot_yaw),
                         arrow_len * math.sin(robot_yaw),
                         head_width=0.05, head_length=0.03, fc='green', ec='green', zorder=10)
            
            ax1.legend(loc='upper right')
            ax1.set_aspect('equal', adjustable='box')
            
            if data['ref_x'] or data['actual_x']:
                all_x = data['ref_x'] + data['actual_x']
                all_y = data['ref_y'] + data['actual_y']
                if all_x and all_y:
                    margin = 0.2
                    ax1.set_xlim(min(all_x) - margin, max(all_x) + margin)
                    ax1.set_ylim(min(all_y) - margin, max(all_y) + margin)
            
            # Plot 2: X/Y Error
            ax2.cla()
            ax2.set_xlabel('Time (s)')
            ax2.set_ylabel('Error (m)')
            ax2.set_title('Position Error (X, Y)')
            ax2.grid(True, alpha=0.3)
            
            if len(data['time']) > 1 and len(data['error_x']) > 1:
                ax2.plot(data['time'], data['error_x'], 'r-', label='Error X', alpha=0.8, linewidth=1.5)
                ax2.plot(data['time'], data['error_y'], 'b-', label='Error Y', alpha=0.8, linewidth=1.5)
                ax2.axhline(y=0, color='k', linestyle='--', linewidth=0.5)
                ax2.legend(loc='upper right')
                
                max_err = max(max(abs(e) for e in data['error_x']), 
                             max(abs(e) for e in data['error_y']), 0.02)
                ax2.set_ylim(-max_err*1.3, max_err*1.3)
            
            # Plot 3: Distance Error
            ax3.cla()
            ax3.set_xlabel('Time (s)')
            ax3.set_ylabel('Distance (m)')
            ax3.set_title('Cross-track Error')
            ax3.grid(True, alpha=0.3)
            
            if len(data['time']) > 1 and len(data['error_dist']) > 1:
                ax3.fill_between(data['time'], 0, data['error_dist'], 
                               color='green', alpha=0.3)
                ax3.plot(data['time'], data['error_dist'], 'g-', linewidth=1.5)
                
                current_err = data['error_dist'][-1]
                mean_err = np.mean(data['error_dist'])
                ax3.axhline(y=mean_err, color='r', linestyle='--', 
                           label=f'Mean: {mean_err:.3f}m')
                ax3.set_title(f'Cross-track Error (Current: {current_err:.3f}m)')
                ax3.legend(loc='upper right')
                ax3.set_ylim(0, max(data['error_dist']) * 1.3 + 0.01)
            
            fig.canvas.draw_idle()
            fig.canvas.flush_events()
            
            time.sleep(0.1)
            
        except Exception as e:
            print(f'Plot error: {e}')
            time.sleep(0.5)
    
    plt.close(fig)


def main(args=None):
    rclpy.init(args=args)
    node = RealtimeVisualizer()
    
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    try:
        run_visualization(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
