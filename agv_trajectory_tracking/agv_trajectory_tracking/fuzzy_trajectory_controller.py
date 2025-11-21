#!/usr/bin/env python3
"""
Fuzzy Trajectory Tracking Controller for AGV (Pure Python Implementation)
Author: Thuong Tran Dinh
Date: October 25, 2025

Implements Sugeno fuzzy logic controller without external fuzzy libraries.
Uses custom membership functions and Sugeno inference engine with constant outputs.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import TwistStamped, PoseWithCovarianceStamped
import math
import signal
import sys


def quaternion_to_euler(x, y, z, w):
    """Convert quaternion (x,y,z,w) to Euler angles (roll, pitch, yaw).
    Returns angles in radians.
    """
    # roll (x-axis rotation)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    # pitch (y-axis rotation)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    # yaw (z-axis rotation)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw


def trimf(x, params):
    """Triangular membership function"""
    a, b, c = params
    if x <= a or x >= c:
        return 0.0
    elif x == b:
        return 1.0
    elif a < x < b:
        return (x - a) / (b - a)
    else:  # b < x < c
        return (c - x) / (c - b)


def trapmf(x, params):
    """Trapezoidal membership function"""
    a, b, c, d = params
    if x <= a or x >= d:
        return 0.0
    elif b <= x <= c:
        return 1.0
    elif a < x < b:
        return (x - a) / (b - a)
    else:  # c < x < d
        return (d - x) / (d - c)


class FuzzyTrajectoryController(Node):
    """Fuzzy Logic Trajectory Tracking Controller (Pure Python - Sugeno Method)"""
    
    def __init__(self):
        super().__init__('fuzzy_trajectory_controller')
        
        # Parameters
        self.declare_parameter('wheel_base', 0.46)
        self.declare_parameter('max_linear_vel', 1.0)
        self.declare_parameter('max_angular_vel', 1.5)  # Increased from 1.0 for better corner turning
        self.declare_parameter('control_frequency', 20.0)
        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('enable_path_publish', True)  # Enable/disable path publishing
        self.declare_parameter('verbose_logging', True)       # Enable detailed velocity logging
        
        self.wheel_base = self.get_parameter('wheel_base').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.control_freq = self.get_parameter('control_frequency').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.enable_path_publish = self.get_parameter('enable_path_publish').value
        self.verbose_logging = self.get_parameter('verbose_logging').value
        
        # 🔧 CRITICAL: Maximum wheel velocity constraint (1.0 m/s per wheel)
        self.max_wheel_vel = 1.0  # m/s - hardware limit for each motor
        
        # Robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.current_path = None
        self.path_received = False
        
        # Shutdown flag
        self.is_shutdown = False
        
        # ROS2 interfaces
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/diff_cont/cmd_vel', 10)
        
        # Subscribe to AMCL pose for robot localization
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', 
            self.amcl_pose_callback, 10)
        
        self.path_sub = self.create_subscription(Path, '/trajectory', 
                                                  self.path_callback, 10)
        
        # Initialize fuzzy system
        self.get_logger().info('Initializing fuzzy system...')
        self.setup_fuzzy_system()
        self.get_logger().info('Fuzzy system ready!')
        
        # Control timer
        timer_period = 1.0 / self.control_freq
        self.control_timer = self.create_timer(timer_period, self.control_loop)
        
        self.get_logger().info('Controller started (Pure Python Fuzzy - Sugeno Method)!')
    
    def stop_robot(self):
        """Stop the robot by sending zero velocity"""
        cmd_vel = TwistStamped()
        cmd_vel.header.stamp = self.get_clock().now().to_msg()
        cmd_vel.header.frame_id = 'base_link'
        cmd_vel.twist.linear.x = 0.0
        cmd_vel.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)
        self.get_logger().info('🛑 Robot stopped!')
    
    def shutdown_callback(self):
        """Cleanup when shutting down"""
        self.get_logger().info('Shutting down controller...')
        self.is_shutdown = True
        self.stop_robot()
    
    def setup_fuzzy_system(self):
        """Setup fuzzy membership functions and rules"""
        # 🔧 IMPROVED v2: Even tighter distance MFs for aggressive tracking
        # More responsive to small errors
        self.e_d_mf = {
            'VS': ('trap', [0, 0, 0.5, 0.8]),      # Very Small: 0-0.5m
            'S': ('tri', [0.5, 0.8, 1.2]),         # Small: 0.2-1.2m
            'M': ('tri', [0.8, 1.2, 1.5]),         # Medium: 0.6-2.5m
            'B': ('tri', [1.2, 1.5, 3.0]),         # Big: 1.5-4.5m
            'VB': ('trap', [1.5, 3.0, 10, 20]),    # Very Big: >3.0m
        }
        
        # 🔧 TUNED v5: BALANCED angle thresholds for all trajectory types
        # Works well for sharp corners (Square) and smooth curves (Figure-8)
        self.e_theta_mf = {
            'NB': ('trap', [-180, -180, -30, -15]),  # Negative Big: <-40° - wider range
            'NM': ('tri', [-30, -15, -5]),          # Negative Medium: -60 to -18°
            'NS': ('tri', [-15, -5, -3]),           # Negative Small: -28 to -4°
            'ZE': ('tri', [-3, 0, 3]),               # Zero: -6 to 6° - balanced tolerance
            'PS': ('tri', [3, 5, 15]),              # Positive Small: 4 to 28°
            'PM': ('tri', [5, 15, 30]),             # Positive Medium: 18 to 60°
            'PB': ('trap', [15, 30, 180, 180])       # Positive Big: >40° - wider range
        }
        
        # 🔧 TUNED v8: AGGRESSIVE angular velocity for SQUARE corners
        # Much stronger rotation for 90° corners
        self.angular_vel_constants = {
            'NB': -1.2,    # Turn VERY hard left - max for 90° corners
            'NM': -0.9,    # Turn medium-strong left
            'NS': -0.5,    # Turn soft left
            'Z': 0.0,      # Straight
            'PS': 0.5,     # Turn soft right
            'PM': 0.9,     # Turn medium-strong right
            'PB': 1.2,     # Turn VERY hard right - max for 90° corners
        }
        
        # 🚀 NEW: Fuzzy rule table for angular velocity based on angle error ONLY
        # Distance affects linear velocity separately
        self.angular_rules = {
            'NB': 'NB',  # Very negative angle → Turn hard left
            'NM': 'NM',  # Medium negative → Turn medium left
            'NS': 'NS',  # Small negative → Turn soft left
            'ZE': 'Z',   # Zero angle → Straight
            'PS': 'PS',  # Small positive → Turn soft right
            'PM': 'PM',  # Medium positive → Turn medium right
            'PB': 'PB',  # Very positive → Turn hard right
        }
        
        self.get_logger().info(f'Fuzzy system initialized with {len(self.angular_rules)} angular rules')
    
    def fuzzify(self, value, mf_dict):
        """Compute membership degrees for a crisp value"""
        memberships = {}
        for label, (mf_type, params) in mf_dict.items():
            if mf_type == 'tri':
                memberships[label] = trimf(value, params)
            elif mf_type == 'trap':
                memberships[label] = trapmf(value, params)
        return memberships
    
    def fuzzy_inference(self, e_d, e_theta_deg):
        """Apply fuzzy inference using simplified Sugeno method
        
        🚀 NEW APPROACH:
        - Angular velocity (ω) controlled by fuzzy logic based on angle error
        - Linear velocity (v) controlled by proportional gain based on distance
        - This decoupling simplifies control and improves stability
        """
        # Fuzzify angle error for angular velocity control
        e_theta_fuzz = self.fuzzify(e_theta_deg, self.e_theta_mf)
        
        # Compute angular velocity using fuzzy rules
        omega_numerator = 0.0
        omega_denominator = 0.0
        
        for angle_label, omega_label in self.angular_rules.items():
            strength = e_theta_fuzz[angle_label]
            
            if strength > 0:
                omega_output = self.angular_vel_constants[omega_label]
                omega_numerator += strength * omega_output
                omega_denominator += strength
        
        # Final angular velocity (weighted average)
        if omega_denominator > 0:
            omega = omega_numerator / omega_denominator
        else:
            omega = 0.0
        
        # 🚀 TUNED v8: IMPROVED velocity for SQUARE corners
        # Key insight: Must SLOW DOWN or STOP at 90° corners to allow rotation!
        
        abs_angle = abs(e_theta_deg)
        
        # 🔥 CORNER DETECTION: Detect sharp 90° corners (typical in square trajectory)
        is_sharp_corner = abs_angle > 75  # Detect corners near 90°
        
        if is_sharp_corner:
            # 🎯 CORNER MODE: Prioritize rotation over forward motion
            # Stop or crawl forward while rotating at corner
            if e_d < 0.15:  # Very close to corner point
                v = 0.08  # Almost stop - just rotate in place
            elif e_d < 0.35:  # Near corner
                v = 0.12  # Crawl forward slowly while turning
            else:  # Approaching corner from distance
                v = 0.18  # Slow approach to corner
            
            # 🔥 BOOST angular velocity for sharp corners
            omega = omega * 1.3  # Increase rotation speed by 30%
            
        else:
            # 🎯 NORMAL MODE: Standard tracking for straight/gentle curves
            # Base linear velocity - balanced for most situations
            if e_d < 0.08:  # Very close - slow down to avoid overshoot
                v_base = 0.30
            elif e_d < 0.25:  # Close - steady tracking speed
                v_base = 0.32 + 0.24 * e_d  # 0.32 to 0.38 m/s
            elif e_d < 0.6:  # Medium distance - increase speed
                v_base = 0.36 + 0.10 * e_d  # 0.36 to 0.42 m/s
            else:  # Far - max tracking speed
                v_base = 0.45  # Cap at 0.45 m/s for stable tracking
            
            # 🔧 SMOOTH velocity adaptation for gentle curves
            # Gradual reduction based on angle error
            if abs_angle < 10:  # Near straight - full speed
                angle_factor = 1.0
            elif abs_angle < 30:  # Gentle curve - minimal reduction
                angle_factor = 0.95
            elif abs_angle < 55:  # Medium curve - moderate reduction
                angle_factor = 0.85
            else:  # Sharper curve (55-75°) - more reduction
                angle_factor = 0.70
            
            v = v_base * angle_factor
        
        # Clip to limits
        v = max(0.0, min(self.max_linear_vel, v))
        omega = max(-self.max_angular_vel, min(self.max_angular_vel, omega))
        
        return v, omega
    
    def amcl_pose_callback(self, msg):
        """Update robot pose from AMCL localization
        
        AMCL provides pose estimation in the map frame, which is more
        accurate than raw odometry as it corrects for drift using
        particle filter localization.
        """
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, self.robot_theta = quaternion_to_euler(
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        )
    
    def path_callback(self, msg):
        """Receive trajectory"""
        self.current_path = msg
        self.path_received = True
        self.get_logger().info(f'Trajectory: {len(msg.poses)} points')
    
    def find_closest_point(self, path, x, y):
        """Find closest point on path ahead of robot"""
        min_dist = float('inf')
        closest_idx = 0
        
        # First, find the actual closest point
        for i, pose_stamped in enumerate(path.poses):
            pose = pose_stamped.pose.position
            dist = math.sqrt((pose.x - x)**2 + (pose.y - y)**2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        # 🔧 IMPROVED v8: SMARTER lookahead - minimal for tight corners
        # Reduced lookahead to track corners more precisely
        if min_dist < 0.10:  # Very close to path - MINIMAL lookahead
            lookahead_points = 2  # Almost no lookahead for tight tracking
        elif min_dist < 0.25:  # Close - small lookahead
            lookahead_points = 3  # Stay close to path
        elif min_dist < 0.5:  # Moderate distance
            lookahead_points = 5  # Balance tracking
        else:  # Far from path - larger lookahead
            lookahead_points = 8  # Get back to path
        
        target_idx = min(closest_idx + lookahead_points, len(path.poses) - 1)
        
        return target_idx
    
    def compute_tracking_errors(self):
        """Compute e_D (distance to target) and e_Theta (heading error)"""
        if not self.path_received or self.current_path is None:
            return 0.0, 0.0
        if len(self.current_path.poses) == 0:
            return 0.0, 0.0
        
        # Find target point (closest + lookahead)
        target_idx = self.find_closest_point(self.current_path, self.robot_x, self.robot_y)
        ref_pose = self.current_path.poses[target_idx].pose
        ref_x = ref_pose.position.x
        ref_y = ref_pose.position.y
        
        # Get reference orientation
        ref_orientation = ref_pose.orientation
        _, _, ref_theta = quaternion_to_euler(
            ref_orientation.x, ref_orientation.y,
            ref_orientation.z, ref_orientation.w
        )
        
        # Distance to target point (Euclidean distance)
        dx = ref_x - self.robot_x
        dy = ref_y - self.robot_y
        e_d = math.sqrt(dx*dx + dy*dy)
        
        # Heading error: angle between robot heading and direction to target
        target_angle = math.atan2(dy, dx)
        e_theta_rad = target_angle - self.robot_theta
        
        # Normalize angle to [-pi, pi]
        while e_theta_rad > math.pi:
            e_theta_rad -= 2.0 * math.pi
        while e_theta_rad < -math.pi:
            e_theta_rad += 2.0 * math.pi
        e_theta_deg = math.degrees(e_theta_rad)
        
        return e_d, e_theta_deg
    
    def wheel_to_twist(self, v_left, v_right):
        """Convert wheel velocities to twist"""
        v = (v_left + v_right) / 2.0
        omega = (v_right - v_left) / self.wheel_base
        v = max(-self.max_linear_vel, min(self.max_linear_vel, v))
        omega = max(-self.max_angular_vel, min(self.max_angular_vel, omega))
        return v, omega
    
    def limit_wheel_velocities(self, v_linear, omega):
        """Limit twist commands to respect maximum wheel velocity constraint
        
        Args:
            v_linear: Desired linear velocity (m/s)
            omega: Desired angular velocity (rad/s)
            
        Returns:
            (v_limited, omega_limited): Safe velocities that keep wheel speeds ≤ max_wheel_vel
            
        Wheel velocities are:
            v_left = v_linear - (omega * wheelbase / 2)
            v_right = v_linear + (omega * wheelbase / 2)
            
        We need: max(|v_left|, |v_right|) ≤ max_wheel_vel
        """
        # Calculate wheel velocities from desired twist
        half_wheelbase = self.wheel_base / 2.0
        v_left = v_linear - (omega * half_wheelbase)
        v_right = v_linear + (omega * half_wheelbase)
        
        # Find maximum wheel velocity
        max_wheel = max(abs(v_left), abs(v_right))
        
        # If exceeds limit, scale down both v and omega proportionally
        if max_wheel > self.max_wheel_vel:
            scale = self.max_wheel_vel / max_wheel
            v_linear *= scale
            omega *= scale
            
            # Recalculate to verify
            v_left = v_linear - (omega * half_wheelbase)
            v_right = v_linear + (omega * half_wheelbase)
        
        return v_linear, omega
    
    def control_loop(self):
        """Main control loop"""
        # Check if shutting down
        if self.is_shutdown:
            return
            
        if not self.path_received or self.current_path is None:
            return
        if len(self.current_path.poses) == 0:
            return
        
        # Compute errors
        e_d, e_theta_deg = self.compute_tracking_errors()
        
        # Check goal
        goal_pose = self.current_path.poses[-1].pose.position
        dist_to_goal = math.sqrt(
            (goal_pose.x - self.robot_x)**2 + 
            (goal_pose.y - self.robot_y)**2
        )
        
        if dist_to_goal < self.goal_tolerance:
            cmd_vel = TwistStamped()
            cmd_vel.header.stamp = self.get_clock().now().to_msg()
            cmd_vel.header.frame_id = 'base_link'
            cmd_vel.twist.linear.x = 0.0
            cmd_vel.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd_vel)
            self.get_logger().info('Goal reached!', throttle_duration_sec=2.0)
            return
        
        # Apply fuzzy controller (now returns v and omega directly)
        v, omega = self.fuzzy_inference(e_d, e_theta_deg)
        
        # 🔧 CRITICAL: Limit velocities to respect wheel velocity constraints
        v, omega = self.limit_wheel_velocities(v, omega)
        
        # Calculate wheel velocities for logging
        half_wheelbase = self.wheel_base / 2.0
        v_left = v - (omega * half_wheelbase)
        v_right = v + (omega * half_wheelbase)
        
        # Publish
        cmd_vel = TwistStamped()
        cmd_vel.header.stamp = self.get_clock().now().to_msg()
        cmd_vel.header.frame_id = 'base_link'
        cmd_vel.twist.linear.x = v
        cmd_vel.twist.angular.z = omega
        self.cmd_vel_pub.publish(cmd_vel)
        
        # Log with detailed velocity information
        if self.verbose_logging:
            self.get_logger().info(
                f'eD={e_d:.3f}m, eT={e_theta_deg:.1f}° | '
                f'v={v:.3f}m/s, ω={omega:.3f}rad/s | '
                f'vL={v_left:.3f}m/s, vR={v_right:.3f}m/s',
                throttle_duration_sec=0.5
            )
        else:
            self.get_logger().info(
                f'eD={e_d:.3f}m, eT={e_theta_deg:.1f}° | '
                f'v={v:.3f}m/s, ω={omega:.3f}rad/s',
                throttle_duration_sec=1.0
            )


def main(args=None):
    rclpy.init(args=args)
    
    controller = None
    
    def signal_handler(sig, frame):
        """Handle Ctrl+C (SIGINT) to gracefully stop the robot"""
        print('\n🛑 Ctrl+C detected! Stopping robot...')
        if controller is not None:
            controller.shutdown_callback()
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(0)
    
    # Register signal handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        controller = FuzzyTrajectoryController()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        print('\n🛑 Keyboard interrupt detected!')
        if controller is not None:
            controller.shutdown_callback()
    except Exception as e:
        print(f'❌ Error: {e}')
    finally:
        if controller is not None:
            controller.stop_robot()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
