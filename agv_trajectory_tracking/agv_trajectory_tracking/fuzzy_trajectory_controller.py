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
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import TwistStamped
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
        
        # 🚀 CRITICAL: Wait for trajectory before controlling
        self.startup_delay = 4.0  # seconds - Wait for trajectory publisher to start
        self.controller_ready = False
        
        # ROS2 interfaces
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/diff_cont/cmd_vel', 10)
        
        # Subscribe to filtered odometry for robot localization
        self.pose_sub = self.create_subscription(
            Odometry, '/odometry/filtered', 
            self.odometry_callback, 10)
        
        self.path_sub = self.create_subscription(Path, '/trajectory', 
                                                  self.path_callback, 10)
        
        # Initialize fuzzy system
        self.get_logger().info('Initializing fuzzy system...')
        self.setup_fuzzy_system()
        self.get_logger().info('Fuzzy system ready!')
        
        # Startup timer (one-shot)
        self.startup_timer = self.create_timer(self.startup_delay, self.on_controller_ready)
        
        # Control timer
        timer_period = 1.0 / self.control_freq
        self.control_timer = self.create_timer(timer_period, self.control_loop)
        
        self.get_logger().info('Controller started (Pure Python Fuzzy - Sugeno Method)!')
        self.get_logger().info(f'⏳ Waiting {self.startup_delay}s for trajectory to be ready...')
    
    def on_controller_ready(self):
        """Called after startup delay - start controlling"""
        self.controller_ready = True
        self.startup_timer.cancel()
        self.get_logger().info('✅ Controller ready! Waiting for trajectory data...')
    
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
        """Setup fuzzy membership functions and rules
        
        🎯 FULL FUZZY SYSTEM:
        - 2 Inputs: e_d (distance error), e_theta (angle error)
        - 2 Outputs: v (linear velocity), ω (angular velocity)
        - Rule base: 5 x 7 = 35 rules
        """
        
        # ============================================
        # INPUT 1: e_d (Distance Error) - 5 MFs
        # ============================================
        self.e_d_mf = {
            'VS': ('trap', [0, 0, 0.1, 0.2]),       # Very Small: 0-0.2m
            'S':  ('tri', [0.1, 0.25, 0.4]),        # Small: 0.1-0.4m
            'M':  ('tri', [0.3, 0.5, 0.8]),         # Medium: 0.3-0.8m
            'B':  ('tri', [0.6, 1.0, 1.5]),         # Big: 0.6-1.5m
            'VB': ('trap', [1.2, 1.8, 10, 20]),     # Very Big: >1.2m
        }
        
        # ============================================
        # INPUT 2: e_theta (Angle Error) - 7 MFs
        # ============================================
        self.e_theta_mf = {
            'NB': ('trap', [-180, -180, -45, -25]), # Negative Big: < -25°
            'NM': ('tri', [-40, -20, -8]),          # Negative Medium: -40 to -8°
            'NS': ('tri', [-15, -5, 0]),            # Negative Small: -15 to 0°
            'ZE': ('tri', [-5, 0, 5]),              # Zero: -5 to 5°
            'PS': ('tri', [0, 5, 15]),              # Positive Small: 0 to 15°
            'PM': ('tri', [8, 20, 40]),             # Positive Medium: 8 to 40°
            'PB': ('trap', [25, 45, 180, 180])      # Positive Big: > 25°
        }
        
        # ============================================
        # OUTPUT 1: Linear Velocity (v) - Sugeno constants
        # ============================================
        self.linear_vel_constants = {
            'STOP':  0.0,    # Stop
            'VS':    0.08,   # Very Slow
            'S':     0.15,   # Slow
            'M':     0.25,   # Medium
            'F':     0.32,   # Fast
            'VF':    0.38,   # Very Fast
        }
        
        # ============================================
        # OUTPUT 2: Angular Velocity (ω) - Sugeno constants
        # ============================================
        self.angular_vel_constants = {
            'NB': -1.2,    # Turn hard left
            'NM': -0.8,    # Turn medium left
            'NS': -0.4,    # Turn soft left
            'Z':   0.0,    # Straight
            'PS':  0.4,    # Turn soft right
            'PM':  0.8,    # Turn medium right
            'PB':  1.2,    # Turn hard right
        }
        
        # ============================================
        # RULE BASE: 35 rules (5 e_d × 7 e_theta)
        # Format: (e_d, e_theta) -> (v, omega)
        # ============================================
        # Rule table for LINEAR VELOCITY (v)
        # Rows: e_d (VS, S, M, B, VB)
        # Cols: e_theta (NB, NM, NS, ZE, PS, PM, PB)
        self.v_rules = {
            # e_d=VS (very close to path)
            ('VS', 'NB'): 'STOP', ('VS', 'NM'): 'VS',  ('VS', 'NS'): 'S',
            ('VS', 'ZE'): 'M',    ('VS', 'PS'): 'S',   ('VS', 'PM'): 'VS',  ('VS', 'PB'): 'STOP',
            
            # e_d=S (small distance)
            ('S', 'NB'): 'VS',   ('S', 'NM'): 'S',    ('S', 'NS'): 'M',
            ('S', 'ZE'): 'F',    ('S', 'PS'): 'M',    ('S', 'PM'): 'S',    ('S', 'PB'): 'VS',
            
            # e_d=M (medium distance)
            ('M', 'NB'): 'S',    ('M', 'NM'): 'M',    ('M', 'NS'): 'F',
            ('M', 'ZE'): 'VF',   ('M', 'PS'): 'F',    ('M', 'PM'): 'M',    ('M', 'PB'): 'S',
            
            # e_d=B (big distance)
            ('B', 'NB'): 'M',    ('B', 'NM'): 'F',    ('B', 'NS'): 'VF',
            ('B', 'ZE'): 'VF',   ('B', 'PS'): 'VF',   ('B', 'PM'): 'F',    ('B', 'PB'): 'M',
            
            # e_d=VB (very big distance)
            ('VB', 'NB'): 'M',   ('VB', 'NM'): 'F',   ('VB', 'NS'): 'VF',
            ('VB', 'ZE'): 'VF',  ('VB', 'PS'): 'VF',  ('VB', 'PM'): 'F',   ('VB', 'PB'): 'M',
        }
        
        # Rule table for ANGULAR VELOCITY (ω)
        # Rows: e_d (VS, S, M, B, VB)  
        # Cols: e_theta (NB, NM, NS, ZE, PS, PM, PB)
        self.omega_rules = {
            # e_d=VS (very close - rotate in place if needed)
            ('VS', 'NB'): 'NB', ('VS', 'NM'): 'NM', ('VS', 'NS'): 'NS',
            ('VS', 'ZE'): 'Z',  ('VS', 'PS'): 'PS', ('VS', 'PM'): 'PM', ('VS', 'PB'): 'PB',
            
            # e_d=S (small distance - moderate turning)
            ('S', 'NB'): 'NB',  ('S', 'NM'): 'NM',  ('S', 'NS'): 'NS',
            ('S', 'ZE'): 'Z',   ('S', 'PS'): 'PS',  ('S', 'PM'): 'PM',  ('S', 'PB'): 'PB',
            
            # e_d=M (medium distance - balanced turning)
            ('M', 'NB'): 'NB',  ('M', 'NM'): 'NM',  ('M', 'NS'): 'NS',
            ('M', 'ZE'): 'Z',   ('M', 'PS'): 'PS',  ('M', 'PM'): 'PM',  ('M', 'PB'): 'PB',
            
            # e_d=B (big distance - softer turning while moving fast)
            ('B', 'NB'): 'NM',  ('B', 'NM'): 'NM',  ('B', 'NS'): 'NS',
            ('B', 'ZE'): 'Z',   ('B', 'PS'): 'PS',  ('B', 'PM'): 'PM',  ('B', 'PB'): 'PM',
            
            # e_d=VB (very big - focus on catching up, moderate turning)
            ('VB', 'NB'): 'NM', ('VB', 'NM'): 'NS', ('VB', 'NS'): 'NS',
            ('VB', 'ZE'): 'Z',  ('VB', 'PS'): 'PS', ('VB', 'PM'): 'PS', ('VB', 'PB'): 'PM',
        }
        
        self.get_logger().info(f'✅ Fuzzy system initialized:')
        self.get_logger().info(f'   - Inputs: e_d (5 MFs), e_theta (7 MFs)')
        self.get_logger().info(f'   - Outputs: v (6 levels), ω (7 levels)')
        self.get_logger().info(f'   - Rules: {len(self.v_rules)} for v, {len(self.omega_rules)} for ω')
    
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
        """Apply fuzzy inference using Sugeno method
        
        🎯 FULL 2-INPUT 2-OUTPUT FUZZY SYSTEM:
        - Inputs: e_d (distance error), e_theta (angle error)
        - Outputs: v (linear velocity), ω (angular velocity)
        - Method: Weighted average (Sugeno)
        
        Steps:
        1. Fuzzify both inputs
        2. Apply rules using AND (min) operator
        3. Compute weighted average for each output
        """
        # ============================================
        # Step 1: FUZZIFY both inputs
        # ============================================
        e_d_fuzz = self.fuzzify(e_d, self.e_d_mf)
        e_theta_fuzz = self.fuzzify(e_theta_deg, self.e_theta_mf)
        
        # ============================================
        # Step 2 & 3: Apply rules and compute outputs
        # ============================================
        v_numerator = 0.0
        v_denominator = 0.0
        omega_numerator = 0.0
        omega_denominator = 0.0
        
        # Iterate through all rules
        for (d_label, theta_label), v_output_label in self.v_rules.items():
            # Get membership degrees for both inputs
            d_strength = e_d_fuzz.get(d_label, 0.0)
            theta_strength = e_theta_fuzz.get(theta_label, 0.0)
            
            # AND operator (min) for rule strength
            rule_strength = min(d_strength, theta_strength)
            
            if rule_strength > 0:
                # Get output values
                v_output = self.linear_vel_constants[v_output_label]
                omega_output_label = self.omega_rules[(d_label, theta_label)]
                omega_output = self.angular_vel_constants[omega_output_label]
                
                # Accumulate for weighted average
                v_numerator += rule_strength * v_output
                v_denominator += rule_strength
                omega_numerator += rule_strength * omega_output
                omega_denominator += rule_strength
        
        # ============================================
        # Step 4: Defuzzify (weighted average)
        # ============================================
        if v_denominator > 0:
            v = v_numerator / v_denominator
        else:
            v = 0.0
        
        if omega_denominator > 0:
            omega = omega_numerator / omega_denominator
        else:
            omega = 0.0
        
        # ============================================
        # Step 5: Apply constraints
        # ============================================
        # Clip to parameter limits
        v = max(0.0, min(self.max_linear_vel, v))
        omega = max(-self.max_angular_vel, min(self.max_angular_vel, omega))
        
        return v, omega
    
    def odometry_callback(self, msg):
        """Update robot pose from filtered odometry
        
        Uses /odometry/filtered topic which provides filtered odometry
        estimation combining multiple sensor sources.
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
        """Find closest point on path AHEAD of robot (critical for Figure-8 crossover)"""
        min_dist = float('inf')
        closest_idx = 0
        
        # 🔥 v10: CRITICAL FIX for Figure-8 crossover point
        # First pass: Find closest point that is AHEAD of robot
        best_idx = -1
        best_dist = float('inf')
        
        for i, pose_stamped in enumerate(path.poses):
            pose = pose_stamped.pose.position
            dist = math.sqrt((pose.x - x)**2 + (pose.y - y)**2)
            
            # Calculate if point is ahead of robot
            dx = pose.x - x
            dy = pose.y - y
            angle_to_point = math.atan2(dy, dx)
            angle_diff = angle_to_point - self.robot_theta
            
            # Normalize angle difference
            while angle_diff > math.pi:
                angle_diff -= 2.0 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2.0 * math.pi
            
            # Point is "ahead" if angle difference is within [-90°, +90°]
            is_ahead = abs(angle_diff) < math.pi / 2.0
            
            # Prefer points ahead, but if robot is far off track, accept any closest
            if is_ahead or dist < 0.3:  # Accept close points even if behind
                if dist < best_dist:
                    best_dist = dist
                    best_idx = i
        
        # Fallback: if no ahead point found (shouldn't happen), use absolute closest
        if best_idx == -1:
            for i, pose_stamped in enumerate(path.poses):
                pose = pose_stamped.pose.position
                dist = math.sqrt((pose.x - x)**2 + (pose.y - y)**2)
                if dist < min_dist:
                    min_dist = dist
                    closest_idx = i
            best_idx = closest_idx
            best_dist = min_dist
        
        closest_idx = best_idx
        min_dist = best_dist
        
        # 🔧 IMPROVED v11: MODERATE lookahead for smooth tracking
        # With ahead detection, we can safely use more lookahead
        if min_dist < 0.10:  # Very close to path
            lookahead_points = 4  # Small lookahead for smooth following
        elif min_dist < 0.20:  # Close
            lookahead_points = 5  # Moderate lookahead
        elif min_dist < 0.40:  # Medium distance
            lookahead_points = 6  # More lookahead
        else:  # Far from path
            lookahead_points = 7  # Larger lookahead to catch up
        
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
        # 🚀 CRITICAL: Wait for startup delay before controlling
        if not self.controller_ready:
            return
        
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
