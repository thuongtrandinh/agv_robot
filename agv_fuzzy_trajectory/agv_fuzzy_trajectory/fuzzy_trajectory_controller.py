#!/usr/bin/env python3
"""
Fuzzy Trajectory Tracking Controller for AGV
Based on Omrane2016 paper with added S (Small) level = 15mm/s
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry, Path
import numpy as np
from tf_transformations import euler_from_quaternion
import math


class FuzzyTrajectoryController(Node):
    def __init__(self):
        super().__init__('fuzzy_trajectory_controller')
        
        # Declare parameters
        self.declare_parameter('wheel_base', 0.35)  # meters
        self.declare_parameter('max_linear_vel', 0.5)  # m/s
        self.declare_parameter('max_angular_vel', 1.0)  # rad/s
        self.declare_parameter('control_frequency', 20.0)  # Hz
        self.declare_parameter('goal_tolerance', 0.1)  # meters
        
        # Get parameters
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
        
        # Fuzzy output singletons (mm/s) - Based on Omrane2016 with added S
        self.velocity_singletons = {
            'Z': 0.0,      # Zero
            'S': 15.0,     # Small (NEW ADDED)
            'F': 30.0,     # Forward
            'M': 40.0,     # Medium
            'B': 50.0,     # Big
            'VB': 70.0     # Very Big
        }
        
        # Create subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.path_sub = self.create_subscription(
            Path, '/trajectory', self.path_callback, 10)
        
        # Create publisher - publish TwistStamped to /diff_cont/cmd_vel for ros2_control
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/diff_cont/cmd_vel', 10)
        
        # Create control timer
        timer_period = 1.0 / self.control_frequency
        self.control_timer = self.create_timer(timer_period, self.control_loop)
        
        self.get_logger().info('Fuzzy Trajectory Controller (Python) initialized')
        self.get_logger().info(f'Wheel base: {self.wheel_base:.3f} m')
        self.get_logger().info(f'Control frequency: {self.control_frequency:.1f} Hz')
    
    def odom_callback(self, msg):
        """Update robot pose from odometry"""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        # Convert quaternion to yaw
        orientation_q = msg.pose.pose.orientation
        _, _, self.robot_theta = euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])
    
    def path_callback(self, msg):
        """Receive trajectory to follow"""
        self.current_path = msg
        self.path_received = True
        self.get_logger().info(f'Received trajectory with {len(msg.poses)} points')
    
    # ==================== MEMBERSHIP FUNCTIONS ====================
    
    def triangular_mf(self, x, a, b, c):
        """Triangular membership function"""
        if x <= a or x >= c:
            return 0.0
        elif x == b:
            return 1.0
        elif a < x < b:
            return (x - a) / (b - a)
        else:  # b < x < c
            return (c - x) / (c - b)
    
    def trapezoidal_mf(self, x, a, b, c, d):
        """Trapezoidal membership function"""
        if x <= a or x >= d:
            return 0.0
        elif b <= x <= c:
            return 1.0
        elif a < x < b:
            return (x - a) / (b - a)
        else:  # c < x < d
            return (d - x) / (d - c)
    
    # ==================== FUZZIFICATION ====================
    
    def fuzzify_distance(self, d):
        """
        Fuzzify distance (mm) based on Omrane2016 Figure 5
        Input: Distance D in mm (0-500mm)
        Output: Dict of membership values for {VS, S, M, B, VB}
        """
        memberships = {}
        
        # VS (Very Small): 0-125mm
        memberships['VS'] = self.triangular_mf(d, 0, 0, 125)
        
        # S (Small): 0-250mm
        memberships['S'] = self.triangular_mf(d, 0, 125, 250)
        
        # M (Medium): 125-375mm
        memberships['M'] = self.triangular_mf(d, 125, 250, 375)
        
        # B (Big): 250-500mm
        memberships['B'] = self.triangular_mf(d, 250, 375, 500)
        
        # VB (Very Big): 375-500mm
        memberships['VB'] = self.triangular_mf(d, 375, 500, 500)
        
        return memberships
    
    def fuzzify_angle(self, phi):
        """
        Fuzzify angle (degrees) based on Omrane2016 Figure 6
        Input: Angle φ in degrees (-180 to 180)
        Output: Dict of membership values for {NB, NM, NS, Z, PS, PM, PB}
        """
        memberships = {}
        
        # NB (Negative Big): -180 to -60
        memberships['NB'] = self.triangular_mf(phi, -180, -180, -60)
        
        # NM (Negative Medium): -120 to 0
        memberships['NM'] = self.triangular_mf(phi, -120, -60, 0)
        
        # NS (Negative Small): -60 to 0
        memberships['NS'] = self.triangular_mf(phi, -60, 0, 0)
        
        # Z (Zero): -60 to 60
        memberships['Z'] = self.triangular_mf(phi, -60, 0, 60)
        
        # PS (Positive Small): 0 to 60
        memberships['PS'] = self.triangular_mf(phi, 0, 0, 60)
        
        # PM (Positive Medium): 0 to 120
        memberships['PM'] = self.triangular_mf(phi, 0, 60, 120)
        
        # PB (Positive Big): 60 to 180
        memberships['PB'] = self.triangular_mf(phi, 60, 180, 180)
        
        return memberships
    
    # ==================== FUZZY RULE BASE ====================
    
    def get_fuzzy_rules(self):
        """
        Fuzzy rule base from Omrane2016 Table 1
        Format: (Distance, Angle) -> (VR, VL)
        """
        rules = {
            # Distance VS
            ('VS', 'NB'): ('B', 'Z'),
            ('VS', 'NM'): ('M', 'Z'),
            ('VS', 'NS'): ('F', 'Z'),
            ('VS', 'Z'): ('S', 'F'),
            ('VS', 'PS'): ('Z', 'F'),
            ('VS', 'PM'): ('Z', 'M'),
            ('VS', 'PB'): ('Z', 'M'),
            
            # Distance S
            ('S', 'NB'): ('VB', 'Z'),
            ('S', 'NM'): ('B', 'Z'),
            ('S', 'NS'): ('M', 'Z'),
            ('S', 'Z'): ('S', 'F'),
            ('S', 'PS'): ('Z', 'M'),
            ('S', 'PM'): ('Z', 'B'),
            ('S', 'PB'): ('Z', 'VB'),
            
            # Distance M
            ('M', 'NB'): ('VB', 'Z'),
            ('M', 'NM'): ('VB', 'Z'),
            ('M', 'NS'): ('B', 'Z'),
            ('M', 'Z'): ('M', 'M'),
            ('M', 'PS'): ('Z', 'B'),
            ('M', 'PM'): ('Z', 'VB'),
            ('M', 'PB'): ('Z', 'VB'),
            
            # Distance B
            ('B', 'NB'): ('VB', 'Z'),
            ('B', 'NM'): ('VB', 'Z'),
            ('B', 'NS'): ('VB', 'Z'),
            ('B', 'Z'): ('B', 'B'),
            ('B', 'PS'): ('Z', 'VB'),
            ('B', 'PM'): ('Z', 'VB'),
            ('B', 'PB'): ('Z', 'VB'),
            
            # Distance VB
            ('VB', 'NB'): ('VB', 'Z'),
            ('VB', 'NM'): ('VB', 'Z'),
            ('VB', 'NS'): ('VB', 'Z'),
            ('VB', 'Z'): ('VB', 'VB'),
            ('VB', 'PS'): ('Z', 'VB'),
            ('VB', 'PM'): ('Z', 'VB'),
            ('VB', 'PB'): ('Z', 'VB'),
        }
        return rules
    
    # ==================== INFERENCE ENGINE ====================
    
    def fuzzy_inference(self, distance_mm, angle_deg):
        """
        Fuzzy inference engine (Mamdani)
        Input: distance (mm), angle (degrees)
        Output: (v_left, v_right) in m/s
        """
        # Fuzzify inputs
        dist_mf = self.fuzzify_distance(distance_mm)
        angle_mf = self.fuzzify_angle(angle_deg)
        
        # Get fuzzy rules
        rules = self.get_fuzzy_rules()
        
        # Initialize output aggregation
        vr_aggregated = {k: 0.0 for k in self.velocity_singletons.keys()}
        vl_aggregated = {k: 0.0 for k in self.velocity_singletons.keys()}
        
        # Apply all rules
        for (dist_label, angle_label), (vr_output, vl_output) in rules.items():
            # Calculate rule strength (AND operation = min)
            rule_strength = min(dist_mf.get(dist_label, 0.0), 
                              angle_mf.get(angle_label, 0.0))
            
            if rule_strength > 0.0:
                # Aggregate outputs (OR operation = max)
                vr_aggregated[vr_output] = max(vr_aggregated[vr_output], rule_strength)
                vl_aggregated[vl_output] = max(vl_aggregated[vl_output], rule_strength)
        
        # Defuzzify using Center of Gravity (CoG)
        v_right = self.defuzzify(vr_aggregated)
        v_left = self.defuzzify(vl_aggregated)
        
        # Convert mm/s to m/s
        return v_left / 1000.0, v_right / 1000.0
    
    def defuzzify(self, aggregated_output):
        """
        Defuzzification using Center of Gravity method
        Input: Dict of {label: membership_value}
        Output: Crisp value in mm/s
        """
        numerator = 0.0
        denominator = 0.0
        
        for label, membership in aggregated_output.items():
            singleton_value = self.velocity_singletons[label]
            numerator += membership * singleton_value
            denominator += membership
        
        if denominator < 1e-6:
            return 0.0
        
        return numerator / denominator
    
    # ==================== TRAJECTORY TRACKING ====================
    
    def find_closest_point(self, path, x, y):
        """Find closest point on trajectory"""
        min_dist = float('inf')
        closest_idx = 0
        
        for i, pose_stamped in enumerate(path.poses):
            pose = pose_stamped.pose.position
            dist = math.sqrt((pose.x - x)**2 + (pose.y - y)**2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        return closest_idx
    
    def compute_errors(self):
        """
        Compute distance and angle errors
        Returns: (distance_mm, angle_deg)
        """
        if not self.path_received or self.current_path is None:
            return 0.0, 0.0
        
        if len(self.current_path.poses) == 0:
            return 0.0, 0.0
        
        # Find closest point on trajectory
        closest_idx = self.find_closest_point(
            self.current_path, self.robot_x, self.robot_y)
        
        # Get reference point
        ref_pose = self.current_path.poses[closest_idx].pose
        ref_x = ref_pose.position.x
        ref_y = ref_pose.position.y
        
        # Get reference orientation
        orientation_q = ref_pose.orientation
        _, _, ref_theta = euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])
        
        # Compute distance (D) - Euclidean distance to target
        dx = ref_x - self.robot_x
        dy = ref_y - self.robot_y
        distance = math.sqrt(dx**2 + dy**2) * 1000.0  # Convert to mm
        
        # Compute angle (φ) - angle error
        # Angle from robot to target
        angle_to_target = math.atan2(dy, dx)
        angle_error = angle_to_target - self.robot_theta
        
        # Normalize to [-pi, pi]
        while angle_error > math.pi:
            angle_error -= 2.0 * math.pi
        while angle_error < -math.pi:
            angle_error += 2.0 * math.pi
        
        # Convert to degrees
        angle_error_deg = math.degrees(angle_error)
        
        return distance, angle_error_deg
    
    def wheel_to_global_velocity(self, v_left, v_right):
        """
        Convert wheel velocities to global velocities (v, ω)
        """
        # Differential drive kinematics
        v = (v_left + v_right) / 2.0
        omega = (v_right - v_left) / self.wheel_base
        
        # Apply velocity limits
        v = np.clip(v, -self.max_linear_vel, self.max_linear_vel)
        omega = np.clip(omega, -self.max_angular_vel, self.max_angular_vel)
        
        return v, omega
    
    # ==================== CONTROL LOOP ====================
    
    def control_loop(self):
        """Main control loop"""
        if not self.path_received or self.current_path is None:
            self.get_logger().warn('No trajectory received yet!', throttle_duration_sec=2.0)
            return
        
        if len(self.current_path.poses) == 0:
            self.get_logger().warn('Empty trajectory!', throttle_duration_sec=2.0)
            return
        
        # Compute tracking errors
        distance_mm, angle_deg = self.compute_errors()
        
        self.get_logger().info(
            f'Robot: ({self.robot_x:.2f}, {self.robot_y:.2f}, {math.degrees(self.robot_theta):.1f}°) | '
            f'Error: D={distance_mm:.1f}mm, φ={angle_deg:.1f}°',
            throttle_duration_sec=1.0)
        
        # Check if goal reached (use last point of trajectory)
        goal = self.current_path.poses[-1].pose.position
        dist_to_goal = math.sqrt(
            (self.robot_x - goal.x)**2 + (self.robot_y - goal.y)**2)
        
        if dist_to_goal < self.goal_tolerance:
            # Stop the robot
            cmd_vel = TwistStamped()
            cmd_vel.header.stamp = self.get_clock().now().to_msg()
            cmd_vel.header.frame_id = "base_link"
            cmd_vel.twist.linear.x = 0.0
            cmd_vel.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd_vel)
            self.get_logger().info(f'Goal reached! Distance to goal: {dist_to_goal:.3f}m', throttle_duration_sec=1.0)
            return
        
        # Apply fuzzy logic controller
        v_left, v_right = self.fuzzy_inference(distance_mm, angle_deg)
        
        self.get_logger().info(
            f'Fuzzy output: VL={v_left*1000:.1f}mm/s, VR={v_right*1000:.1f}mm/s',
            throttle_duration_sec=1.0)
        
        # Convert to global velocities
        v, omega = self.wheel_to_global_velocity(v_left, v_right)
        
        # Publish command with timestamp (same as keyboard)
        cmd_vel = TwistStamped()
        cmd_vel.header.stamp = self.get_clock().now().to_msg()
        cmd_vel.header.frame_id = "base_link"
        cmd_vel.twist.linear.x = v
        cmd_vel.twist.angular.z = omega
        self.cmd_vel_pub.publish(cmd_vel)
        
        self.get_logger().info(
            f'Published: v={v:.3f}m/s, ω={omega:.3f}rad/s',
            throttle_duration_sec=1.0)
        
        # Debug output
        self.get_logger().debug(
            f'D={distance_mm:.1f}mm, φ={angle_deg:.1f}°, '
            f'VL={v_left*1000:.1f}mm/s, VR={v_right*1000:.1f}mm/s, '
            f'v={v:.3f}m/s, ω={omega:.3f}rad/s'
        )


def main(args=None):
    rclpy.init(args=args)
    node = FuzzyTrajectoryController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
