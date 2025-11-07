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
        self.declare_parameter('max_angular_vel', 1.0)
        self.declare_parameter('control_frequency', 20.0)
        self.declare_parameter('goal_tolerance', 0.15)
        
        self.wheel_base = self.get_parameter('wheel_base').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.control_frequency = self.get_parameter('control_frequency').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        
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
        timer_period = 1.0 / self.control_frequency
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
            'VS': ('trap', [0, 0, 0.2, 0.5]),      # Very Small: 0-0.5m
            'S': ('tri', [0.2, 0.6, 1.2]),         # Small: 0.2-1.2m
            'M': ('tri', [0.6, 1.5, 2.5]),         # Medium: 0.6-2.5m
            'B': ('tri', [1.5, 3.0, 4.5]),         # Big: 1.5-4.5m
            'VB': ('trap', [3.0, 5.0, 10, 20]),    # Very Big: >3.0m
        }
        
        # 🔧 TUNED v3: Tighter angle thresholds for more responsive turning
        self.e_theta_mf = {
            'NB': ('trap', [-180, -180, -45, -25]),  # Negative Big: <-25° (was -30°)
            'NM': ('tri', [-45, -25, -10]),          # Negative Medium: -45 to -10° (tighter)
            'NS': ('tri', [-25, -10, -2]),           # Negative Small: -25 to -2° (tighter)
            'ZE': ('tri', [-3, 0, 3]),               # Zero: -3 to 3° (tighter tolerance)
            'PS': ('tri', [2, 10, 25]),              # Positive Small: 2 to 25° (tighter)
            'PM': ('tri', [10, 25, 45]),             # Positive Medium: 10 to 45° (tighter)
            'PB': ('trap', [25, 45, 180, 180])       # Positive Big: >25° (was 30°)
        }
        
        # 🔧 TUNED v3: More aggressive angular velocity for tighter curves
        self.angular_vel_constants = {
            'NB': -1.0,    # Turn hard left (was -0.8)
            'NM': -0.6,    # Turn medium left (was -0.5)
            'NS': -0.3,    # Turn soft left (was -0.25)
            'Z': 0.0,      # Straight
            'PS': 0.3,     # Turn soft right (was 0.25)
            'PM': 0.6,     # Turn medium right (was 0.5)
            'PB': 1.0,     # Turn hard right (was 0.8)
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
        
        # 🚀 TUNED v3: Faster linear velocity for better tracking
        # Aggressive speed profile to reduce tracking error
        if e_d < 0.2:  # Very close - maintain minimum speed
            v = 0.25
        elif e_d < 0.8:  # Close - ramp up smoothly
            v = 0.35 + 0.45 * e_d  # 0.35 to 0.71 m/s
        elif e_d < 2.5:  # Medium distance - high speed
            v = 0.65 + 0.2 * e_d  # 0.65 to 1.15 m/s (capped at max)
        else:  # Far - max speed
            v = 1.0  # Full speed
        
        # Reduce linear velocity when turning sharply (smoother but less aggressive)
        angle_factor = 1.0 - 0.2 * min(abs(e_theta_deg) / 90.0, 1.0)  # Reduce up to 20% (was 30%)
        v = v * angle_factor
        
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
        
        # 🔧 IMPROVED v2: Much smaller lookahead for tighter tracking
        # Pure pursuit with very short lookahead distance
        if min_dist < 0.3:  # Very close to path
            lookahead_points = 1  # Almost no lookahead - follow exactly
        elif min_dist < 1.0:  # Close
            lookahead_points = 2
        elif min_dist < 2.0:  # Moderate distance
            lookahead_points = 3
        else:  # Far from path
            lookahead_points = 5  # Larger lookahead to get back on track
        
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
        
        # Publish
        cmd_vel = TwistStamped()
        cmd_vel.header.stamp = self.get_clock().now().to_msg()
        cmd_vel.header.frame_id = 'base_link'
        cmd_vel.twist.linear.x = v
        cmd_vel.twist.angular.z = omega
        self.cmd_vel_pub.publish(cmd_vel)
        
        # Log
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
