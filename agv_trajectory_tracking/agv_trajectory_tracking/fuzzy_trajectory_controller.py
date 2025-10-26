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
    
    def setup_fuzzy_system(self):
        """Setup fuzzy membership functions and rules"""
        # Define membership function parameters for e_D (distance error in meters)
        self.e_d_mf = {
            'VS': ('trap', [0, 0, 5, 1.25]),
            'S': ('tri', [0, 1.25, 2.5]),
            'M': ('tri', [1.25, 2.5, 3.75]),
            'B': ('tri', [2.5, 3.75, 5.0]),
            'VB': ('trap', [3.75, 5.0, 10, 20]),
        }
        
        # Define membership function parameters for e_Theta (angle error in degrees)
        self.e_theta_mf = {
            'NB': ('trap', [-180, -180, -60, -30]),
            'NM': ('tri', [-60, -30, -5]),
            'NS': ('tri', [-30, -5, 0]),
            'ZE': ('tri', [-5, 0, 5]),
            'PS': ('tri', [0, 5, 30]),
            'PM': ('tri', [5, 30, 60]),
            'PB': ('trap', [30, 60, 180, 180])
        }
        
        # Sugeno: Output constants (singleton values) for wheel velocities in m/s
        self.output_constants = {
            'Z': 0, 'S': 0.15, 'F': 0.25, 'M': 0.35,
            'B': 0.45, 'B': 0.45, 'VB': 0.55
        }
        
        # Fuzzy rule table (Sugeno): (e_D, e_Theta) -> (VL_constant, VR_constant)
        self.rule_table = {
            ('VS', 'NB'): ('PM', 'NB'), ('VS', 'NM'): ('PM', 'NS'),
            ('VS', 'NS'): ('PM', 'ZE'), ('VS', 'ZE'): ('PB', 'PB'),
            ('VS', 'PS'): ('PM', 'ZE'), ('VS', 'PM'): ('PM', 'NS'),
            ('VS', 'PB'): ('PM', 'NB'),
            
            ('S', 'NB'): ('PS', 'NM'), ('S', 'NM'): ('PM', 'NS'),
            ('S', 'NS'): ('PM', 'ZE'), ('S', 'ZE'): ('PM', 'PM'),
            ('S', 'PS'): ('PM', 'ZE'), ('S', 'PM'): ('PM', 'NS'),
            ('S', 'PB'): ('PS', 'NM'),
            
            ('M', 'NB'): ('ZE', 'NM'), ('M', 'NM'): ('PS', 'NS'),
            ('M', 'NS'): ('PM', 'PS'), ('M', 'ZE'): ('PS', 'PS'),
            ('M', 'PS'): ('PM', 'PS'), ('M', 'PM'): ('PS', 'NS'),
            ('M', 'PB'): ('ZE', 'NM'),
            
            ('B', 'NB'): ('PS', 'NS'), ('B', 'NM'): ('PS', 'ZE'),
            ('B', 'NS'): ('PS', 'ZE'), ('B', 'ZE'): ('PS', 'PS'),
            ('B', 'PS'): ('ZE', 'PS'), ('B', 'PM'): ('ZE', 'PS'),
            ('B', 'PB'): ('NS', 'PS'),
            
            ('VB', 'NB'): ('NM', 'ZE'), ('VB', 'NM'): ('NS', 'PS'),
            ('VB', 'NS'): ('PS', 'PM'), ('VB', 'ZE'): ('PS', 'PS'),
            ('VB', 'PS'): ('PS', 'PM'), ('VB', 'PM'): ('NS', 'PS'),
            ('VB', 'PB'): ('NM', 'ZE'),
        }
        
        self.get_logger().info(f'Created {len(self.rule_table)} fuzzy rules')
    
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
        
        Sugeno method uses weighted average of constant outputs
        instead of defuzzifying membership functions like Mamdani.
        """
        # Fuzzify inputs
        e_d_fuzz = self.fuzzify(e_d, self.e_d_mf)
        e_theta_fuzz = self.fuzzify(e_theta_deg, self.e_theta_mf)
        
        # Apply rules and compute weighted outputs (Sugeno)
        vl_numerator = 0.0
        vl_denominator = 0.0
        vr_numerator = 0.0
        vr_denominator = 0.0
        
        for (e_d_label, e_theta_label), (vl_label, vr_label) in self.rule_table.items():
            # Rule firing strength (min for AND operation)
            strength = min(e_d_fuzz[e_d_label], e_theta_fuzz[e_theta_label])
            
            if strength > 0:
                # Get constant output values for this rule
                vl_output = self.output_constants[vl_label]
                vr_output = self.output_constants[vr_label]
                
                # Accumulate weighted sum (Sugeno weighted average)
                vl_numerator += strength * vl_output
                vl_denominator += strength
                
                vr_numerator += strength * vr_output
                vr_denominator += strength
        
        # Compute final outputs (weighted average)
        if vl_denominator > 0:
            v_left = vl_numerator / vl_denominator
        else:
            v_left = 0.0
            
        if vr_denominator > 0:
            v_right = vr_numerator / vr_denominator
        else:
            v_right = 0.0
        
        # Clip to output range
        v_left = max(-0.5, min(0.5, v_left))
        v_right = max(-0.5, min(0.5, v_right))
        
        return v_left, v_right
    
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
        """Find closest point on path"""
        min_dist = float('inf')
        closest_idx = 0
        for i, pose_stamped in enumerate(path.poses):
            pose = pose_stamped.pose.position
            dist = math.sqrt((pose.x - x)**2 + (pose.y - y)**2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        return closest_idx
    
    def compute_tracking_errors(self):
        """Compute e_D and e_Theta"""
        if not self.path_received or self.current_path is None:
            return 0.0, 0.0
        if len(self.current_path.poses) == 0:
            return 0.0, 0.0
        
        # Find closest point
        closest_idx = self.find_closest_point(self.current_path, self.robot_x, self.robot_y)
        ref_pose = self.current_path.poses[closest_idx].pose
        ref_x = ref_pose.position.x
        ref_y = ref_pose.position.y
        
        # Get reference orientation
        ref_orientation = ref_pose.orientation
        _, _, ref_theta = quaternion_to_euler(
            ref_orientation.x, ref_orientation.y,
            ref_orientation.z, ref_orientation.w
        )
        
        # Distance error (projected)
        dx = self.robot_x - ref_x
        dy = self.robot_y - ref_y
        e_d = dx * math.cos(ref_theta) + dy * math.sin(ref_theta)
        
        # Angle error (in degrees)
        e_theta_rad = self.robot_theta - ref_theta
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
        
        # Apply fuzzy controller
        v_left, v_right = self.fuzzy_inference(e_d, e_theta_deg)
        v, omega = self.wheel_to_twist(v_left, v_right)
        
        # Publish
        cmd_vel = TwistStamped()
        cmd_vel.header.stamp = self.get_clock().now().to_msg()
        cmd_vel.header.frame_id = 'base_link'
        cmd_vel.twist.linear.x = v
        cmd_vel.twist.angular.z = omega
        self.cmd_vel_pub.publish(cmd_vel)
        
        # Log
        self.get_logger().info(
            f'eD={e_d:.3f}m, eT={e_theta_deg:.1f}deg | '
            f'VL={v_left:.3f}, VR={v_right:.3f} | '
            f'v={v:.3f}, w={omega:.3f}',
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)
    try:
        controller = FuzzyTrajectoryController()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
