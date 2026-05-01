#!/usr/bin/env python3
"""
Fuzzy Trajectory Tracking Controller for AGV (Pure Python Implementation)
Author: Thuong Tran Dinh
Date: October 25, 2025

Implements Sugeno fuzzy logic controller without external fuzzy libraries.
Uses custom membership functions and Sugeno inference engine with constant outputs.

Robot constraints (differential drive):
- Wheel base: 0.46m (fixed)
- Max wheel velocity: 0.4 m/s
- Max linear velocity: 0.4 m/s (when both wheels max speed same direction)
- Max angular velocity: 0.87 rad/s (when wheels spin opposite at max speed)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import TwistStamped
import math
import signal
import sys

# Import tf_transformations for quaternion to euler conversion
try:
    from tf_transformations import euler_from_quaternion
    USE_TF2 = True
except ImportError:
    USE_TF2 = False
    print("Warning: tf_transformations not found, using manual quaternion conversion")


def quaternion_to_yaw(qx, qy, qz, qw):
    """Convert quaternion to yaw angle using tf2 or fallback to manual calculation.
    
    Args:
        qx, qy, qz, qw: Quaternion components
        
    Returns:
        yaw: Rotation around Z-axis in radians
    """
    if USE_TF2:
        # Use tf_transformations library
        _, _, yaw = euler_from_quaternion([qx, qy, qz, qw])
        return yaw
    else:
        # Manual calculation (fallback)
        t3 = 2.0 * (qw * qz + qx * qy)
        t4 = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(t3, t4)


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
    """Fuzzy Logic Trajectory Tracking Controller (Pure Python - Sugeno Method)
    
    Differential Drive Robot Kinematics:
    - v = (v_left + v_right) / 2
    - omega = (v_right - v_left) / L
    
    With max wheel velocity = 0.4 m/s and L = 0.46m:
    - v_max = 0.4 m/s (both wheels same direction)
    - omega_max = 2 * 0.4 / 0.46 = 1.74 rad/s (wheels opposite direction)
    - Practical omega_max = 0.87 rad/s (for smooth control)
    """
    
    # Fixed robot parameters
    WHEEL_BASE = 0.46  # meters (fixed, do not change)
    MAX_WHEEL_VEL = 0.4  # m/s per wheel (hardware limit)
    MAX_LINEAR_VEL = 0.4  # m/s (when both wheels at max same direction)
    MAX_ANGULAR_VEL = 0.87  # rad/s (practical limit for smooth control)
    
    def __init__(self):
        super().__init__('fuzzy_trajectory_controller')
        
        # Only configurable parameters
        self.declare_parameter('control_frequency', 200.0)
        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('lookahead_points', 8)  # Configurable lookahead
        
        self.control_freq = self.get_parameter('control_frequency').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.lookahead_points = self.get_parameter('lookahead_points').value
        
        # Robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.current_path = None
        self.path_received = False
        
        # Shutdown flag
        self.is_shutdown = False
        
        # Wait for trajectory before controlling
        self.startup_delay = 4.0
        self.controller_ready = False
        
        # ROS2 interfaces
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/diff_cont/cmd_vel', 10)
        
        self.pose_sub = self.create_subscription(
            Odometry, '/odometry/filtered', 
            self.odometry_callback, 10)
        
        self.path_sub = self.create_subscription(
            Path, '/trajectory', 
            self.path_callback, 10)
        
        # Initialize fuzzy system
        self.get_logger().info('Initializing fuzzy system...')
        self.setup_fuzzy_system()
        self.get_logger().info('Fuzzy system ready!')
        
        # Startup timer
        self.startup_timer = self.create_timer(self.startup_delay, self.on_controller_ready)
        
        # Control timer
        timer_period = 1.0 / self.control_freq
        self.control_timer = self.create_timer(timer_period, self.control_loop)
        
        self.get_logger().info(f'Controller started! (L={self.WHEEL_BASE}m, Vmax={self.MAX_WHEEL_VEL}m/s)')
        self.get_logger().info(f'Waiting {self.startup_delay}s for trajectory...')
    
    def on_controller_ready(self):
        """Called after startup delay"""
        self.controller_ready = True
        self.startup_timer.cancel()
        self.get_logger().info('Controller ready! Waiting for trajectory data...')
    
    def stop_robot(self):
        """Stop the robot"""
        cmd_vel = TwistStamped()
        cmd_vel.header.stamp = self.get_clock().now().to_msg()
        cmd_vel.header.frame_id = 'base_link'
        cmd_vel.twist.linear.x = 0.0
        cmd_vel.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)
        self.get_logger().info('Robot stopped!')
    
    def shutdown_callback(self):
        """Cleanup when shutting down"""
        self.get_logger().info('Shutting down controller...')
        self.is_shutdown = True
        self.stop_robot()
    
    def setup_fuzzy_system(self):
        """Setup fuzzy membership functions and rules.
        
        Inputs:
        - e_d: Distance error (0 to infinity meters)
        - e_theta: Heading error (-180 to +180 degrees)
        
        Outputs (Sugeno constants):
        - v: Linear velocity (0 to 0.4 m/s)
        - omega: Angular velocity (-0.87 to +0.87 rad/s)
        """
        # Distance error membership functions
        self.e_d_mf = {
            'VS': ('trap', [0, 0, 0.1, 0.2]),       # Very Small: 0-0.2m
            'S':  ('tri', [0.1, 0.25, 0.4]),        # Small: 0.1-0.4m
            'M':  ('tri', [0.3, 0.5, 0.8]),         # Medium: 0.3-0.8m
            'B':  ('tri', [0.6, 1.0, 1.5]),         # Big: 0.6-1.5m
            'VB': ('trap', [1.2, 1.8, 10, 20]),     # Very Big: >1.2m
        }
      
        # Heading error membership functions (degrees)
        self.e_theta_mf = {
            'NB': ('trap', [-180, -180, -45, -25]), # Negative Big: < -25 deg
            'NM': ('tri', [-40, -20, -8]),          # Negative Medium
            'NS': ('tri', [-15, -5, 0]),            # Negative Small
            'ZE': ('tri', [-5, 0, 5]),              # Zero
            'PS': ('tri', [0, 5, 15]),              # Positive Small
            'PM': ('tri', [8, 20, 40]),             # Positive Medium
            'PB': ('trap', [25, 45, 180, 180])      # Positive Big: > 25 deg
        }
        
        # Linear velocity constants (scaled for max 0.4 m/s)
        self.linear_vel_constants = {
            'STOP':  0.0,    # Stop
            'VS':    0.05,   # Very Slow
            'S':     0.10,   # Slow
            'M':     0.20,   # Medium
            'F':     0.30,   # Fast
            'VF':    0.38,   # Very Fast (near max)
        }
        
        # Angular velocity constants (scaled for max 0.87 rad/s)
        self.angular_vel_constants = {
            'NB': -0.80,   # Turn hard left
            'NM': -0.50,   # Turn medium left
            'NS': -0.25,   # Turn soft left
            'Z':   0.0,    # Straight
            'PS':  0.25,   # Turn soft right
            'PM':  0.50,   # Turn medium right
            'PB':  0.80,   # Turn hard right
        }
        
        # Rule tables for LINEAR VELOCITY (v)
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
        
        # Rule tables for ANGULAR VELOCITY (omega)
        self.omega_rules = {
            # e_d=VS (very close - rotate in place if needed)
            ('VS', 'NB'): 'NB', ('VS', 'NM'): 'NM', ('VS', 'NS'): 'NS',
            ('VS', 'ZE'): 'Z',  ('VS', 'PS'): 'PS', ('VS', 'PM'): 'PM', ('VS', 'PB'): 'PB',
            
            # e_d=S (small distance)
            ('S', 'NB'): 'NB',  ('S', 'NM'): 'NM',  ('S', 'NS'): 'NS',
            ('S', 'ZE'): 'Z',   ('S', 'PS'): 'PS',  ('S', 'PM'): 'PM',  ('S', 'PB'): 'PB',
            
            # e_d=M (medium distance)
            ('M', 'NB'): 'NB',  ('M', 'NM'): 'NM',  ('M', 'NS'): 'NS',
            ('M', 'ZE'): 'Z',   ('M', 'PS'): 'PS',  ('M', 'PM'): 'PM',  ('M', 'PB'): 'PB',
            
            # e_d=B (big distance - softer turning)
            ('B', 'NB'): 'NM',  ('B', 'NM'): 'NM',  ('B', 'NS'): 'NS',
            ('B', 'ZE'): 'Z',   ('B', 'PS'): 'PS',  ('B', 'PM'): 'PM',  ('B', 'PB'): 'PM',
            
            # e_d=VB (very big - moderate turning)
            ('VB', 'NB'): 'NM', ('VB', 'NM'): 'NS', ('VB', 'NS'): 'NS',
            ('VB', 'ZE'): 'Z',  ('VB', 'PS'): 'PS', ('VB', 'PM'): 'PS', ('VB', 'PB'): 'PM',
        }
        
        self.get_logger().info(f'Fuzzy system: {len(self.v_rules)} rules')
    
    def fuzzify(self, value, mf_dict):
        """Compute normalized membership degrees for a crisp value.
        
        The membership degrees are normalized so that their sum equals 1.
        This ensures proper fuzzy partition of unity.
        """
        memberships = {}
        total = 0.0
        
        # Calculate raw memberships
        for label, (mf_type, params) in mf_dict.items():
            if mf_type == 'tri':
                mu = trimf(value, params)
            elif mf_type == 'trap':
                mu = trapmf(value, params)
            else:
                mu = 0.0
            memberships[label] = mu
            total += mu
        
        # Normalize to sum = 1 (if total > 0)
        if total > 0:
            for label in memberships:
                memberships[label] /= total
        
        return memberships
    
    def fuzzy_inference(self, e_d, e_theta_deg):
        """Sugeno fuzzy inference with normalized memberships.
        
        Args:
            e_d: Distance error in meters
            e_theta_deg: Heading error in degrees
            
        Returns:
            (v, omega): Linear and angular velocities
        """
        # Fuzzify inputs (normalized)
        e_d_fuzz = self.fuzzify(e_d, self.e_d_mf)
        e_theta_fuzz = self.fuzzify(e_theta_deg, self.e_theta_mf)
        
        v_numerator = 0.0
        v_denominator = 0.0
        omega_numerator = 0.0
        omega_denominator = 0.0
        
        # Iterate through all rules
        for (d_label, theta_label), v_output_label in self.v_rules.items():
            d_strength = e_d_fuzz.get(d_label, 0.0)
            theta_strength = e_theta_fuzz.get(theta_label, 0.0)
            
            # AND operator (min) for rule strength
            rule_strength = min(d_strength, theta_strength)
            
            if rule_strength > 0:
                v_output = self.linear_vel_constants[v_output_label]
                omega_output_label = self.omega_rules[(d_label, theta_label)]
                omega_output = self.angular_vel_constants[omega_output_label]
                
                # Accumulate for weighted average
                v_numerator += rule_strength * v_output
                v_denominator += rule_strength
                omega_numerator += rule_strength * omega_output
                omega_denominator += rule_strength
        
        # Defuzzify (weighted average)
        v = v_numerator / v_denominator if v_denominator > 0 else 0.0
        omega = omega_numerator / omega_denominator if omega_denominator > 0 else 0.0
        
        # Apply constraints and normalize output
        v, omega = self.limit_velocities(v, omega)
        
        return v, omega
    
    def limit_velocities(self, v_linear, omega):
        """Limit velocities to respect wheel velocity constraints.
        
        Combines velocity limiting logic to ensure:
        1. Linear velocity <= MAX_LINEAR_VEL
        2. Angular velocity <= MAX_ANGULAR_VEL
        3. Individual wheel velocities <= MAX_WHEEL_VEL
        
        Differential drive kinematics:
            v_left = v - (omega * L / 2)
            v_right = v + (omega * L / 2)
        """
        # First, clip to parameter limits
        v_linear = max(0.0, min(self.MAX_LINEAR_VEL, v_linear))
        omega = max(-self.MAX_ANGULAR_VEL, min(self.MAX_ANGULAR_VEL, omega))
        
        # Calculate wheel velocities
        half_L = self.WHEEL_BASE / 2.0
        v_left = v_linear - (omega * half_L)
        v_right = v_linear + (omega * half_L)
        
        # Find maximum wheel velocity
        max_wheel = max(abs(v_left), abs(v_right))
        
        # Scale down if exceeding wheel limit
        if max_wheel > self.MAX_WHEEL_VEL:
            scale = self.MAX_WHEEL_VEL / max_wheel
            v_linear *= scale
            omega *= scale
        
        return v_linear, omega
    
    def odometry_callback(self, msg):
        """Update robot pose from filtered odometry using tf2."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        self.robot_theta = quaternion_to_yaw(q.x, q.y, q.z, q.w)
    
    def path_callback(self, msg):
        """Receive trajectory"""
        self.current_path = msg
        self.path_received = True
        self.get_logger().info(f'Trajectory: {len(msg.poses)} points', 
                              throttle_duration_sec=5.0)
    
    def find_target_point(self, path, x, y):
        """Find target point on path ahead of robot.
        
        Uses larger lookahead for smoother trajectory tracking.
        More lookahead points = smoother following but less accurate at corners.
        """
        best_idx = -1
        best_dist = float('inf')
        
        # Find closest point that is AHEAD of robot
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
            
            # Point is "ahead" if angle difference is within [-90, +90] degrees
            is_ahead = abs(angle_diff) < math.pi / 2.0
            
            # Prefer points ahead, but accept close points even if behind
            if is_ahead or dist < 0.3:
                if dist < best_dist:
                    best_dist = dist
                    best_idx = i
        
        # Fallback: use absolute closest if no ahead point found
        if best_idx == -1:
            for i, pose_stamped in enumerate(path.poses):
                pose = pose_stamped.pose.position
                dist = math.sqrt((pose.x - x)**2 + (pose.y - y)**2)
                if dist < best_dist:
                    best_dist = dist
                    best_idx = i
        
        # Apply dynamic lookahead based on distance to path
        if best_dist < 0.10:
            lookahead = self.lookahead_points
        elif best_dist < 0.20:
            lookahead = self.lookahead_points + 2
        elif best_dist < 0.40:
            lookahead = self.lookahead_points + 4
        else:
            lookahead = self.lookahead_points + 6
        
        target_idx = min(best_idx + lookahead, len(path.poses) - 1)
        
        return target_idx, best_dist
    
    def compute_tracking_errors(self):
        """Compute e_D (distance to target) and e_Theta (heading error)"""
        if not self.path_received or self.current_path is None:
            return 0.0, 0.0
        if len(self.current_path.poses) == 0:
            return 0.0, 0.0
        
        # Find target point
        target_idx, _ = self.find_target_point(
            self.current_path, self.robot_x, self.robot_y)
        
        ref_pose = self.current_path.poses[target_idx].pose
        ref_x = ref_pose.position.x
        ref_y = ref_pose.position.y
        
        # Distance to target point
        dx = ref_x - self.robot_x
        dy = ref_y - self.robot_y
        e_d = math.sqrt(dx*dx + dy*dy)
        
        # Heading error: angle to target minus current heading
        target_angle = math.atan2(dy, dx)
        e_theta_rad = target_angle - self.robot_theta
        
        # Normalize to [-pi, pi]
        while e_theta_rad > math.pi:
            e_theta_rad -= 2.0 * math.pi
        while e_theta_rad < -math.pi:
            e_theta_rad += 2.0 * math.pi
        
        return e_d, math.degrees(e_theta_rad)
    
    def control_loop(self):
        """Main control loop"""
        if not self.controller_ready or self.is_shutdown:
            return
            
        if not self.path_received or self.current_path is None:
            return
        if len(self.current_path.poses) == 0:
            return
        
        # Compute errors
        e_d, e_theta_deg = self.compute_tracking_errors()
        
        # Check if reached goal
        goal_pose = self.current_path.poses[-1].pose.position
        dist_to_goal = math.sqrt(
            (goal_pose.x - self.robot_x)**2 + 
            (goal_pose.y - self.robot_y)**2
        )
        
        if dist_to_goal < self.goal_tolerance:
            self.stop_robot()
            self.get_logger().info('Goal reached!', throttle_duration_sec=2.0)
            return
        
        # Fuzzy inference
        v, omega = self.fuzzy_inference(e_d, e_theta_deg)
        
        # Publish command
        cmd_vel = TwistStamped()
        cmd_vel.header.stamp = self.get_clock().now().to_msg()
        cmd_vel.header.frame_id = 'base_link'
        cmd_vel.twist.linear.x = v
        cmd_vel.twist.angular.z = omega
        self.cmd_vel_pub.publish(cmd_vel)
        
        # Log (throttled)
        self.get_logger().info(
            f'eD={e_d:.3f}m eT={e_theta_deg:.1f}deg | v={v:.3f} w={omega:.3f}',
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)
    
    controller = None
    
    def signal_handler(sig, frame):
        print('\nCtrl+C detected! Stopping robot...')
        if controller is not None:
            controller.shutdown_callback()
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        controller = FuzzyTrajectoryController()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        if controller is not None:
            controller.shutdown_callback()
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if controller is not None:
            controller.stop_robot()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
