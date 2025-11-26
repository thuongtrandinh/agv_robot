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
        self.declare_parameter('trajectory_type', 2)  # 1:Circle, 2:Square, 3:Figure-8
        self.declare_parameter('publish_rate', 20.0)  # Hz - Fixed optimal rate for real-time tracking
        self.declare_parameter('path_points', 200)    # Number of points in path
        self.declare_parameter('preview_time', 10.0)  # Seconds of trajectory to preview
        self.declare_parameter('center_x', 5.0)       # Trajectory center X coordinate
        self.declare_parameter('center_y', -2.0)      # Trajectory center Y coordinate
        self.declare_parameter('radius', 5.0)         # Circle radius (m)
        self.declare_parameter('enable_publish', True)  # Enable/disable trajectory publishing
        self.declare_parameter('trajectory_speed', 0.3)  # Trajectory reference speed (m/s) - Reduced for smoother motion
        self.declare_parameter('ramp_time', 3.0)  # Time to ramp up/down speed (s) for smooth start/stop
        
        self.trajectory_type = self.get_parameter('trajectory_type').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.path_points = self.get_parameter('path_points').value
        self.preview_time = self.get_parameter('preview_time').value
        self.center_x = self.get_parameter('center_x').value
        self.center_y = self.get_parameter('center_y').value
        self.radius = self.get_parameter('radius').value
        self.enable_publish = self.get_parameter('enable_publish').value
        self.trajectory_speed = self.get_parameter('trajectory_speed').value
        self.ramp_time = self.get_parameter('ramp_time').value
        
        # Publisher
        self.path_pub = self.create_publisher(Path, '/trajectory', 10)
        
        # Wait for robot to be ready (localization, controller, etc.)
        self.startup_delay = 3.0  # seconds - Wait for robot systems to initialize
        self.is_ready = False
        
        # Create startup timer (one-shot)
        self.startup_timer = self.create_timer(self.startup_delay, self.on_startup_complete)
        
        # Trajectory state
        self.current_time = 0.0
        timer_period = 1.0 / self.publish_rate
        self.dt = timer_period
        
        # Timer for publishing (will start after startup delay)
        self.timer = self.create_timer(timer_period, self.publish_trajectory)
        
        self.get_logger().info(f'Trajectory Publisher started!')
        self.get_logger().info(f'  Type: {self.get_trajectory_name()}')
        self.get_logger().info(f'  Center: ({self.center_x:.1f}, {self.center_y:.1f})')
        
        # Display size based on trajectory type
        if self.trajectory_type == 1:  # Circle
            self.get_logger().info(f'  Circle Radius: {self.radius:.1f} m')
        elif self.trajectory_type == 2:  # Square
            side = self.radius * 2.0
            self.get_logger().info(f'  Square Side: {side:.1f} m (radius param: {self.radius:.1f} m)')
        elif self.trajectory_type == 3:  # Figure-8
            amplitude = self.radius / 2.0
            self.get_logger().info(f'  Figure-8 Amplitude: {amplitude:.1f} m (each circle radius: {amplitude:.1f} m, total radius param: {self.radius:.1f} m)')
        
        self.get_logger().info(f'  Publishing at {self.publish_rate} Hz (FIXED for real-time)')
        self.get_logger().info(f'  Path points: {self.path_points}')
        self.get_logger().info(f'  Trajectory speed: {self.trajectory_speed} m/s (with {self.ramp_time}s smooth ramp)')
        self.get_logger().info(f'⏳ Waiting {self.startup_delay}s for robot to be ready...')
    
    def on_startup_complete(self):
        """Called after startup delay - robot is ready"""
        self.is_ready = True
        self.startup_timer.cancel()  # Stop the one-shot timer
        self.get_logger().info('✅ Robot ready! Starting trajectory publishing...')
    
    def get_trajectory_name(self):
        """Get human-readable trajectory name"""
        names = {1: 'Circle', 2: 'Square', 3: 'Figure-8 (Lemniscate)'}
        return names.get(self.trajectory_type, 'Unknown')
    
    def smooth_speed_ramp(self, t):
        """
        Apply smooth speed ramping to avoid sudden velocity changes
        Uses sigmoid function for smooth acceleration/deceleration
        
        Args:
            t: current time (s)
        Returns:
            speed_scale: multiplier (0.0 to 1.0) for trajectory speed
        """
        if t < self.ramp_time:
            # Smooth ramp up using sigmoid-like function
            # Maps [0, ramp_time] -> [0, 1] smoothly
            progress = t / self.ramp_time
            # Use smooth S-curve (cubic easing)
            speed_scale = progress * progress * (3.0 - 2.0 * progress)
        else:
            # Full speed after ramp time
            speed_scale = 1.0
        
        return speed_scale
    
    def trajectory_reference_circle(self, t):
        """
        Circular trajectory
        
        Args:
            t: time (s)
        Returns:
            x_ref, y_ref, theta_ref
        """
        # Parameters
        R = self.radius    # Radius [m] - Use configurable radius
        # Angular velocity calculated from desired linear velocity
        # omega = v / R, where v is desired speed
        # Apply smooth speed ramping to avoid sudden velocity changes
        speed_scale = self.smooth_speed_ramp(t)
        v_desired = self.trajectory_speed * speed_scale  # Ramped speed
        omega = v_desired / max(R, 0.1)  # Prevent division by zero
        
        # Use configurable center instead of fixed [0,0]
        center = [self.center_x, self.center_y]
        
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
        side = self.radius * 2.0  # Side length [m] - Based on radius parameter
        # Apply smooth speed ramping to avoid sudden velocity changes
        speed_scale = self.smooth_speed_ramp(t)
        v_desired = self.trajectory_speed * speed_scale  # Ramped speed
        T_side = side / max(v_desired, 0.01) # Time per side (s) - prevent division by zero
        T_period = 4 * T_side     # Period (time for one complete loop)
        
        # Compute current position on square (relative to center)
        time_in_period = t % T_period
        current_side = int(time_in_period / T_side)  # Which side (0,1,2,3)
        time_in_side = time_in_period % T_side
        
        if current_side == 0:  # Side 1: Moving up
            x_rel = side/2
            y_rel = -side/2 + (side/T_side) * time_in_side
            theta_ref = math.pi/2  # 90 degrees
            
        elif current_side == 1:  # Side 2: Moving left
            x_rel = side/2 - (side/T_side) * time_in_side
            y_rel = side/2
            theta_ref = math.pi  # 180 degrees
            
        elif current_side == 2:  # Side 3: Moving down
            x_rel = -side/2
            y_rel = side/2 - (side/T_side) * time_in_side
            theta_ref = -math.pi/2  # -90 degrees
            
        else:  # Side 4: Moving right
            x_rel = -side/2 + (side/T_side) * time_in_side
            y_rel = -side/2
            theta_ref = 0.0  # 0 degrees
        
        # Translate to world coordinates with configurable center
        x_ref = self.center_x + x_rel
        y_ref = self.center_y + y_rel
        
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
        # Each circle in figure-8 has radius = r/2, so amplitude A = r/2
        A = self.radius / 2.0  # Amplitude (half width) - Based on radius parameter
        # Apply smooth speed ramping to avoid sudden velocity changes
        speed_scale = self.smooth_speed_ramp(t)
        v_desired = self.trajectory_speed * speed_scale  # Ramped speed
        # Approximate perimeter for figure-8, omega adjusted for desired speed
        omega = v_desired / max(2.0 * A, 0.1)  # Adjusted angular velocity, prevent division by zero
        
        # Compute trajectory (Lemniscate of Gerono parametric equations)
        # Relative to origin
        phi = omega * t
        x_rel = A * math.cos(phi)
        y_rel = A * math.sin(2*phi) / 2.0
        
        # Translate to world coordinates with configurable center
        x_ref = self.center_x + x_rel
        y_ref = self.center_y + y_rel
        
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
        """Generate Path message with trajectory reference points"""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        
        # Generate path points looking ahead (not the entire path, just reference points)
        dt_path = self.preview_time / self.path_points  # Time interval for each point
        
        for i in range(self.path_points):
            t = self.current_time + i * dt_path
            x_ref, y_ref, theta_ref = self.trajectory_reference(t)
            theta_ref = self.normalize_angle(theta_ref)
            
            # Create PoseStamped message for each reference point
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = path_msg.header.stamp
            pose_stamped.header.frame_id = 'map'
            
            # Set position of the reference point
            pose_stamped.pose.position.x = x_ref
            pose_stamped.pose.position.y = y_ref
            pose_stamped.pose.position.z = 0.0
            
            # Convert yaw angle (theta_ref) to quaternion orientation
            qx, qy, qz, qw = self.quaternion_from_euler(0.0, 0.0, theta_ref)
            pose_stamped.pose.orientation.x = qx
            pose_stamped.pose.orientation.y = qy
            pose_stamped.pose.orientation.z = qz
            pose_stamped.pose.orientation.w = qw
            
            # Append the reference point to the path
            path_msg.poses.append(pose_stamped)
        
        return path_msg

    
    def publish_trajectory(self):
        """Timer callback to publish reference trajectory points"""
        # Wait for startup delay before publishing
        if not self.is_ready:
            return
        
        # Check if publishing is enabled
        if self.enable_publish:
            # Generate and publish the reference points in the trajectory
            path_msg = self.generate_path_message()
            self.path_pub.publish(path_msg)
            
            # Get current reference for logging
            x_ref, y_ref, theta_ref = self.trajectory_reference(self.current_time)
            
            # Log periodically for debugging purposes
            self.get_logger().info(
                f'Published trajectory | t={self.current_time:.2f}s | '
                f'ref: x={x_ref:.3f}, y={y_ref:.3f}, θ={math.degrees(theta_ref):.1f}°',
                throttle_duration_sec=2.0
            )
        else:
            # Still log even if not publishing
            x_ref, y_ref, theta_ref = self.trajectory_reference(self.current_time)
            self.get_logger().info(
                f'[NOT PUBLISHING] t={self.current_time:.2f}s | '
                f'ref: x={x_ref:.3f}, y={y_ref:.3f}, θ={math.degrees(theta_ref):.1f}°',
                throttle_duration_sec=2.0
            )
        
        # Update time for the next set of points
        self.current_time += self.dt



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
