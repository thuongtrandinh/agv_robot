#!/usr/bin/env python3
"""
Fuzzy Trajectory Tracking Controller for AGV (Pure Python Implementation)
Author: Thuong Tran Dinh
Date: December 26, 2025

🔧 v13: SIMPLIFIED VERSION - Focus on working correctly
- Simple Pure Pursuit for target point selection
- Simple Fuzzy controller for velocity commands
- Fixed lookahead distance instead of complex curvature detection
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import TwistStamped
import math
import signal
import sys


def quaternion_to_euler(x, y, z, w):
    """Convert quaternion (x,y,z,w) to Euler angles (roll, pitch, yaw)."""
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = max(-1.0, min(1.0, t2))
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw


def normalize_angle(angle):
    """Normalize angle to [-pi, pi]"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class FuzzyTrajectoryController(Node):
    """Simplified Fuzzy Logic Trajectory Controller"""
    
    def __init__(self):
        super().__init__('fuzzy_trajectory_controller')
        
        # Parameters
        self.declare_parameter('wheel_base', 0.46)
        self.declare_parameter('max_linear_vel', 0.35)
        self.declare_parameter('max_angular_vel', 0.8)
        self.declare_parameter('control_frequency', 20.0)
        self.declare_parameter('goal_tolerance', 0.10)
        self.declare_parameter('lookahead_distance', 0.25)  # Fixed lookahead
        
        self.wheel_base = self.get_parameter('wheel_base').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.control_freq = self.get_parameter('control_frequency').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.lookahead_dist = self.get_parameter('lookahead_distance').value
        
        # 🔧 Hardware constraint: max 0.4 m/s per wheel
        self.max_wheel_vel = 0.4
        
        # Robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.current_path = None
        self.path_received = False
        self.is_shutdown = False
        
        # Current target index on path
        self.current_target_idx = 0
        
        # ROS2 interfaces
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/diff_cont/cmd_vel', 10)
        self.pose_sub = self.create_subscription(
            Odometry, '/odometry/filtered', 
            self.odometry_callback, 10)
        self.path_sub = self.create_subscription(
            Path, '/trajectory', 
            self.path_callback, 10)
        
        # Control timer
        timer_period = 1.0 / self.control_freq
        self.control_timer = self.create_timer(timer_period, self.control_loop)
        
        self.get_logger().info('🚀 Fuzzy Controller v13 started!')
        self.get_logger().info(f'   Lookahead: {self.lookahead_dist}m, Max v: {self.max_linear_vel}m/s')
    
    def stop_robot(self):
        """Stop the robot"""
        cmd_vel = TwistStamped()
        cmd_vel.header.stamp = self.get_clock().now().to_msg()
        cmd_vel.header.frame_id = 'base_link'
        cmd_vel.twist.linear.x = 0.0
        cmd_vel.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)
    
    def odometry_callback(self, msg):
        """Update robot pose"""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.robot_theta = quaternion_to_euler(q.x, q.y, q.z, q.w)
    
    def path_callback(self, msg):
        """Receive trajectory"""
        self.current_path = msg
        self.path_received = True
        self.current_target_idx = 0  # Reset target index
        self.get_logger().info(f'📍 Trajectory received: {len(msg.poses)} points')
    
    def find_lookahead_point(self):
        """Find target point using Pure Pursuit lookahead
        
        Returns: (target_x, target_y, target_idx) or None if no path
        """
        if not self.path_received or self.current_path is None:
            return None
        
        path = self.current_path
        n = len(path.poses)
        if n == 0:
            return None
        
        # Start searching from current target index
        start_idx = max(0, self.current_target_idx - 5)
        
        # Find closest point first
        min_dist = float('inf')
        closest_idx = start_idx
        
        for i in range(start_idx, n):
            pose = path.poses[i].pose.position
            dist = math.sqrt((pose.x - self.robot_x)**2 + (pose.y - self.robot_y)**2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        # Now find lookahead point (first point >= lookahead distance from robot)
        target_idx = closest_idx
        for i in range(closest_idx, n):
            pose = path.poses[i].pose.position
            dist = math.sqrt((pose.x - self.robot_x)**2 + (pose.y - self.robot_y)**2)
            if dist >= self.lookahead_dist:
                target_idx = i
                break
        else:
            # If no point is far enough, use last point
            target_idx = n - 1
        
        # Update current target (only move forward, never backward)
        if target_idx > self.current_target_idx:
            self.current_target_idx = target_idx
        
        target_pose = path.poses[target_idx].pose.position
        return (target_pose.x, target_pose.y, target_idx)
    
    def fuzzy_velocity(self, heading_error_deg, distance):
        """Simple fuzzy controller for velocity
        
        Args:
            heading_error_deg: Angle error in degrees [-180, 180]
            distance: Distance to target point in meters
        
        Returns:
            (linear_vel, angular_vel)
        """
        abs_heading = abs(heading_error_deg)
        
        # === LINEAR VELOCITY ===
        # Slow down when heading error is large, speed up when aligned
        if abs_heading > 60:
            # Big heading error: stop and turn
            v = 0.0
        elif abs_heading > 30:
            # Medium heading error: slow down
            v = 0.08
        elif abs_heading > 15:
            # Small heading error: moderate speed
            v = 0.15
        elif abs_heading > 5:
            # Very small error: good speed
            v = 0.22
        else:
            # Aligned: full speed
            v = 0.28
        
        # Also slow down when close to target
        if distance < 0.15:
            v = min(v, 0.12)
        elif distance < 0.3:
            v = min(v, 0.20)
        
        # === ANGULAR VELOCITY ===
        # Proportional to heading error with saturation
        Kp = 0.025  # rad/s per degree
        omega = Kp * heading_error_deg
        
        # Boost turning when heading error is large but robot is moving slow
        if abs_heading > 30 and v < 0.1:
            omega = 0.6 * (1 if heading_error_deg > 0 else -1)
        
        # Clamp velocities
        v = max(0.0, min(self.max_linear_vel, v))
        omega = max(-self.max_angular_vel, min(self.max_angular_vel, omega))
        
        return v, omega
    
    def limit_wheel_velocities(self, v, omega):
        """Ensure wheel velocities don't exceed max_wheel_vel"""
        half_wb = self.wheel_base / 2.0
        v_left = v - omega * half_wb
        v_right = v + omega * half_wb
        
        max_wheel = max(abs(v_left), abs(v_right))
        if max_wheel > self.max_wheel_vel:
            scale = self.max_wheel_vel / max_wheel
            v *= scale
            omega *= scale
        
        return v, omega
    
    def control_loop(self):
        """Main control loop"""
        if self.is_shutdown:
            return
        
        if not self.path_received or self.current_path is None:
            return
        
        # Find lookahead target point
        result = self.find_lookahead_point()
        if result is None:
            return
        
        target_x, target_y, target_idx = result
        
        # Check if reached goal
        goal = self.current_path.poses[-1].pose.position
        dist_to_goal = math.sqrt((goal.x - self.robot_x)**2 + (goal.y - self.robot_y)**2)
        
        if dist_to_goal < self.goal_tolerance:
            self.stop_robot()
            self.get_logger().info('🎯 Goal reached!', throttle_duration_sec=2.0)
            return
        
        # Calculate heading error
        dx = target_x - self.robot_x
        dy = target_y - self.robot_y
        target_angle = math.atan2(dy, dx)
        heading_error = normalize_angle(target_angle - self.robot_theta)
        heading_error_deg = math.degrees(heading_error)
        
        # Distance to target
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Get velocities from fuzzy controller
        v, omega = self.fuzzy_velocity(heading_error_deg, distance)
        
        # Limit wheel velocities
        v, omega = self.limit_wheel_velocities(v, omega)
        
        # Publish command
        cmd_vel = TwistStamped()
        cmd_vel.header.stamp = self.get_clock().now().to_msg()
        cmd_vel.header.frame_id = 'base_link'
        cmd_vel.twist.linear.x = v
        cmd_vel.twist.angular.z = omega
        self.cmd_vel_pub.publish(cmd_vel)
        
        # Log
        self.get_logger().info(
            f'idx={target_idx}, d={distance:.2f}m, θ={heading_error_deg:.1f}° | '
            f'v={v:.2f}m/s, ω={omega:.2f}rad/s',
            throttle_duration_sec=0.5
        )


def main(args=None):
    rclpy.init(args=args)
    controller = None
    
    def signal_handler(sig, frame):
        print('\n🛑 Stopping robot...')
        if controller:
            controller.is_shutdown = True
            controller.stop_robot()
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        controller = FuzzyTrajectoryController()
        rclpy.spin(controller)
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if controller:
            controller.stop_robot()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
