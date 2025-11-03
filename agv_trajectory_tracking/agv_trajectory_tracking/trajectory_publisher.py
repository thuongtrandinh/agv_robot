#!/usr/bin/env python3
"""
Trajectory Publisher for AGV
Generates reference trajectories and publishes to /trajectory topic

Based on MATLAB code: trajectory_reference.m
Supports: Circle, Square, Figure-8 (Lemniscate)

Author: Thuong Tran Dinh
Date: October 26, 2025
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math
import numpy as np


class TrajectoryPublisher(Node):
    """Publishes reference trajectories for robot to follow"""
    
    def __init__(self):
        super().__init__('trajectory_publisher')
        
        # Parameters
        self.declare_parameter('trajectory_type', 1)  # 1:Circle, 2:Square, 3:Figure-8
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('path_points', 200)    # Number of points in path
        self.declare_parameter('preview_time', 20.0)  # Seconds of trajectory to preview
        
        self.trajectory_type = self.get_parameter('trajectory_type').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.path_points = self.get_parameter('path_points').value
        self.preview_time = self.get_parameter('preview_time').value
        
        # Publisher
        self.path_pub = self.create_publisher(Path, '/trajectory', 10)
        
        # Timer for publishing
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_trajectory)
        
        # Trajectory state
        self.current_time = 0.0
        self.dt = timer_period
        
        self.get_logger().info(f'Trajectory Publisher started!')
        self.get_logger().info(f'  Type: {self.get_trajectory_name()}')
        self.get_logger().info(f'  Publishing at {self.publish_rate} Hz')
        self.get_logger().info(f'  Path points: {self.path_points}')
    
    def get_trajectory_name(self):
        """Get human-readable trajectory name"""
        names = {1: 'Circle', 2: 'Square', 3: 'Figure-8 (Lemniscate)'}
        return names.get(self.trajectory_type, 'Unknown')
    
    def trajectory_reference_circle(self, t):
        """
        Circular trajectory
        
        Args:
            t: time (s)
        Returns:
            x_ref, y_ref, theta_ref
        """
        # Parameters
        R = 5.0           # Radius [m] - Increased for better visibility in large maps
        omega = 0.2       # Angular velocity [rad/s]
        center = [0, 0]   # Center of circle
        
        # Compute trajectory
        phi = omega * t
        x_ref = center[0] + R * math.cos(phi)
        y_ref = center[1] + R * math.sin(phi)
        theta_ref = phi + math.pi/2
        
        return x_ref, y_ref, theta_ref
    
    def trajectory_reference_square(self, t):
        """
        Square trajectory
        
        Args:
            t: time (s)
        Returns:
            x_ref, y_ref, theta_ref
        """
        # Parameters
        side = 8.0          # Side length [m] - Increased for better visibility
        T_side = 20.0       # Time per side (s) -> v = 8/20 = 0.4 m/s
        T_period = 4 * T_side  # Period (time for one complete loop)
        
        # Compute current position on square
        time_in_period = t % T_period
        current_side = int(time_in_period / T_side)  # Which side (0,1,2,3)
        time_in_side = time_in_period % T_side
        
        if current_side == 0:  # Side 1: Moving up
            x_ref = side/2
            y_ref = -side/2 + (side/T_side) * time_in_side
            theta_ref = math.pi/2  # 90 degrees
            
        elif current_side == 1:  # Side 2: Moving left
            x_ref = side/2 - (side/T_side) * time_in_side
            y_ref = side/2
            theta_ref = math.pi  # 180 degrees
            
        elif current_side == 2:  # Side 3: Moving down
            x_ref = -side/2
            y_ref = side/2 - (side/T_side) * time_in_side
            theta_ref = -math.pi/2  # -90 degrees
            
        else:  # Side 4: Moving right
            x_ref = -side/2 + (side/T_side) * time_in_side
            y_ref = -side/2
            theta_ref = 0.0  # 0 degrees
        
        return x_ref, y_ref, theta_ref
    
    def trajectory_reference_figure8(self, t):
        """
        Figure-8 trajectory (Lemniscate of Gerono)
        
        Args:
            t: time (s)
        Returns:
            x_ref, y_ref, theta_ref
        """
        # Parameters
        A = 5.0           # Amplitude (half width) - Increased for better visibility
        omega = 0.2       # Angular velocity [rad/s]
        
        # Compute trajectory (Lemniscate of Gerono parametric equations)
        phi = omega * t
        x_ref = A * math.cos(phi)
        y_ref = A * math.sin(2*phi) / 2.0
        
        # Compute derivatives to find tangent angle
        dx_dt = -A * omega * math.sin(phi)
        dy_dt = A * omega * math.cos(2*phi)
        theta_ref = math.atan2(dy_dt, dx_dt)
        
        return x_ref, y_ref, theta_ref
    
    def trajectory_reference(self, t):
        """
        Get reference trajectory at time t
        
        Args:
            t: time (s)
        Returns:
            x_ref, y_ref, theta_ref
        """
        if self.trajectory_type == 1:
            return self.trajectory_reference_circle(t)
        elif self.trajectory_type == 2:
            return self.trajectory_reference_square(t)
        elif self.trajectory_type == 3:
            return self.trajectory_reference_figure8(t)
        else:
            self.get_logger().warn(f'Unknown trajectory type: {self.trajectory_type}')
            return 0.0, 0.0, 0.0
    
    def normalize_angle(self, angle):
        """Wrap angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Convert Euler angles to quaternion (x, y, z, w)
        
        Args:
            roll, pitch, yaw: Euler angles in radians
        Returns:
            (x, y, z, w): quaternion components
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        
        return x, y, z, w
    
    def generate_path_message(self):
        """Generate Path message with trajectory points"""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        
        # Generate path points looking ahead
        dt_path = self.preview_time / self.path_points
        
        for i in range(self.path_points):
            t = self.current_time + i * dt_path
            x_ref, y_ref, theta_ref = self.trajectory_reference(t)
            theta_ref = self.normalize_angle(theta_ref)
            
            # Create PoseStamped
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = path_msg.header.stamp
            pose_stamped.header.frame_id = 'map'
            
            # Position
            pose_stamped.pose.position.x = x_ref
            pose_stamped.pose.position.y = y_ref
            pose_stamped.pose.position.z = 0.0
            
            # Orientation (from yaw angle)
            qx, qy, qz, qw = self.quaternion_from_euler(0.0, 0.0, theta_ref)
            pose_stamped.pose.orientation.x = qx
            pose_stamped.pose.orientation.y = qy
            pose_stamped.pose.orientation.z = qz
            pose_stamped.pose.orientation.w = qw
            
            path_msg.poses.append(pose_stamped)
        
        return path_msg
    
    def publish_trajectory(self):
        """Timer callback to publish trajectory"""
        # Generate and publish path
        path_msg = self.generate_path_message()
        self.path_pub.publish(path_msg)
        
        # Update time
        self.current_time += self.dt
        
        # Get current reference for logging
        x_ref, y_ref, theta_ref = self.trajectory_reference(self.current_time)
        
        # Log periodically
        self.get_logger().info(
            f'Published trajectory | t={self.current_time:.2f}s | '
            f'Current ref: x={x_ref:.3f}, y={y_ref:.3f}, θ={math.degrees(theta_ref):.1f}°',
            throttle_duration_sec=2.0
        )


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = TrajectoryPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
