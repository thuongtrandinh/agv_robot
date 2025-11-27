#!/usr/bin/env python3
"""
Fuzzy Trajectory Tracking Controller (Direct Control Mode)
Author: Thuong Tran Dinh
Updated: November 27, 2025

Logic: 
- Pure Sugeno Fuzzy Logic -> Direct Twist Command
- Relies on STM32 PID for dynamic smoothing
- Relies on Trajectory Publisher's 'ramp_time' for reference smoothing
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import TwistStamped, PoseWithCovarianceStamped
import math
import signal
import sys

# ==========================================
# HELPER FUNCTIONS
# ==========================================
def quaternion_to_euler(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return roll, pitch, yaw

def trimf(x, params):
    a, b, c = params
    if x <= a or x >= c: return 0.0
    elif x == b: return 1.0
    elif a < x < b: return (x - a) / (b - a)
    else: return (c - x) / (c - b)

def trapmf(x, params):
    a, b, c, d = params
    if x <= a or x >= d: return 0.0
    elif b <= x <= c: return 1.0
    elif a < x < b: return (x - a) / (b - a)
    else: return (d - x) / (d - c)

# ==========================================
# MAIN CONTROLLER CLASS
# ==========================================
class FuzzyTrajectoryController(Node):
    def __init__(self):
        super().__init__('fuzzy_trajectory_controller')
        
        # --- PARAMETERS ---
        self.declare_parameter('wheel_base', 0.42)
        self.declare_parameter('max_linear_vel', 0.4) 
        self.declare_parameter('max_angular_vel', 0.8) 
        self.declare_parameter('control_frequency', 20.0) # Đồng bộ với tần số STM32
        self.declare_parameter('goal_tolerance', 0.10)
        self.declare_parameter('verbose_logging', True)
        
        self.wheel_base = self.get_parameter('wheel_base').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.control_freq = self.get_parameter('control_frequency').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.verbose_logging = self.get_parameter('verbose_logging').value
        
        # Giới hạn phần cứng (Safety only)
        self.max_wheel_vel = 0.5 
        
        # State
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.current_path = None
        self.path_received = False
        self.is_shutdown = False
        self.controller_ready = False
        
        # --- ROS2 INTERFACES ---
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/diff_cont/cmd_vel', 10)
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_callback, 10)
        self.path_sub = self.create_subscription(Path, '/trajectory', self.path_callback, 10)
            
        # Initialize Fuzzy Logic
        self.setup_fuzzy_system()
        
        # Startup Delays
        self.startup_delay = 4.0
        self.startup_timer = self.create_timer(self.startup_delay, self.on_controller_ready)
        
        # Control Loop Timer
        self.control_timer = self.create_timer(1.0/self.control_freq, self.control_loop)
        
        self.get_logger().info('✅ Direct Fuzzy Controller Initialized (Targeting STM32 PID)')

    def on_controller_ready(self):
        self.controller_ready = True
        self.startup_timer.cancel()
        self.get_logger().info('🚀 Controller Active! Waiting for path...')

    def setup_fuzzy_system(self):
        # Distance MFs
        self.e_d_mf = {
            'VS': ('trap', [0, 0, 0.5, 0.8]),
            'S': ('tri', [0.5, 0.8, 1.2]),
            'M': ('tri', [0.8, 1.2, 1.5]),
            'B': ('tri', [1.2, 1.5, 3.0]),
            'VB': ('trap', [1.5, 3.0, 10, 20]),
        }
        # Angle MFs
        self.e_theta_mf = {
            'NB': ('trap', [-180, -180, -30, -15]),
            'NM': ('tri', [-30, -15, -5]),
            'NS': ('tri', [-15, -5, -3]),
            'ZE': ('tri', [-3, 0, 3]),
            'PS': ('tri', [3, 5, 15]),
            'PM': ('tri', [5, 15, 30]),
            'PB': ('trap', [15, 30, 180, 180])
        }
        # Angular Velocity Outputs
        self.angular_vel_constants = {
            'NB': -1.0, 'NM': -0.8, 'NS': -0.08, 'Z': 0.0,
            'PS': 0.08, 'PM': 0.8, 'PB': 1.0, 
        }
        # Rule Base
        self.angular_rules = {
            'NB': 'NB', 'NM': 'NM', 'NS': 'NS', 'ZE': 'Z',
            'PS': 'PS', 'PM': 'PM', 'PB': 'PB',
        }

    def fuzzify(self, value, mf_dict):
        memberships = {}
        for label, (mf_type, params) in mf_dict.items():
            if mf_type == 'tri': memberships[label] = trimf(value, params)
            elif mf_type == 'trap': memberships[label] = trapmf(value, params)
        return memberships

    def fuzzy_inference(self, e_d, e_theta_deg):
        # 1. Angular Velocity
        e_theta_fuzz = self.fuzzify(e_theta_deg, self.e_theta_mf)
        omega_num, omega_den = 0.0, 0.0
        for angle_label, omega_label in self.angular_rules.items():
            strength = e_theta_fuzz[angle_label]
            if strength > 0:
                omega_num += strength * self.angular_vel_constants[omega_label]
                omega_den += strength
        omega = omega_num / omega_den if omega_den > 0 else 0.0
        
        # 2. Linear Velocity Logic (Heuristic)
        abs_angle = abs(e_theta_deg)
        # Sharp corner handling
        if abs_angle > 75 and e_d < 0.4:
            if e_d < 0.15: v = 0.08
            elif e_d < 0.30: v = 0.12
            else: v = 0.16
            omega = omega * 1.2
        else:
            # Smooth curve handling
            if e_d < 0.10: v_base = 0.32
            elif e_d < 0.25: v_base = 0.33
            elif e_d < 0.50: v_base = 0.34
            else: v_base = 0.35
            
            # Slow down on turns
            if abs_angle < 15: angle_factor = 1.0
            elif abs_angle < 35: angle_factor = 0.95
            elif abs_angle < 55: angle_factor = 0.90
            elif abs_angle < 70: angle_factor = 0.80
            else: angle_factor = 0.70
            v = v_base * angle_factor
        
        return v, omega

    def amcl_pose_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.robot_theta = quaternion_to_euler(q.x, q.y, q.z, q.w)

    def path_callback(self, msg):
        self.current_path = msg
        self.path_received = True

    def find_closest_point(self, path, x, y):
        # Lookahead Logic
        best_idx, best_dist = -1, float('inf')
        for i, pose_stamped in enumerate(path.poses):
            pose = pose_stamped.pose.position
            dist = math.sqrt((pose.x - x)**2 + (pose.y - y)**2)
            dx, dy = pose.x - x, pose.y - y
            angle_diff = math.atan2(dy, dx) - self.robot_theta
            while angle_diff > math.pi: angle_diff -= 2*math.pi
            while angle_diff < -math.pi: angle_diff += 2*math.pi
            
            # Prefer points in front
            if abs(angle_diff) < math.pi/2.0 or dist < 0.3:
                if dist < best_dist: best_dist, best_idx = dist, i
        
        if best_idx == -1: 
            min_dist, closest_idx = float('inf'), 0
            for i, p in enumerate(path.poses):
                d = math.sqrt((p.pose.position.x-x)**2 + (p.pose.position.y-y)**2)
                if d < min_dist: min_dist, closest_idx = d, i
            best_idx, best_dist = closest_idx, min_dist

        min_dist = best_dist
        if min_dist < 0.10: lookahead = 5
        elif min_dist < 0.20: lookahead = 6
        elif min_dist < 0.40: lookahead = 8
        else: lookahead = 10
        return min(best_idx + lookahead, len(path.poses) - 1)

    def compute_errors(self):
        if not self.path_received or not self.current_path: return 0.0, 0.0
        target_idx = self.find_closest_point(self.current_path, self.robot_x, self.robot_y)
        ref_pose = self.current_path.poses[target_idx].pose
        
        dx = ref_pose.position.x - self.robot_x
        dy = ref_pose.position.y - self.robot_y
        e_d = math.sqrt(dx*dx + dy*dy)
        
        target_angle = math.atan2(dy, dx)
        e_theta_rad = target_angle - self.robot_theta
        while e_theta_rad > math.pi: e_theta_rad -= 2*math.pi
        while e_theta_rad < -math.pi: e_theta_rad += 2*math.pi
        return e_d, math.degrees(e_theta_rad)

    def limit_wheel_velocities(self, v, w):
        """Hardware Safety Limit"""
        half_wb = self.wheel_base / 2.0
        v_left = v - (w * half_wb)
        v_right = v + (w * half_wb)
        max_v_wheel = max(abs(v_left), abs(v_right))
        if max_v_wheel > self.max_wheel_vel:
            scale = self.max_wheel_vel / max_v_wheel
            v *= scale
            w *= scale
        return v, w

    def control_loop(self):
        if not self.controller_ready or self.is_shutdown: return
        if not self.path_received or not self.current_path: return
        
        # 1. Compute Errors
        e_d, e_theta_deg = self.compute_errors()
        
        # 2. Check Goal
        goal = self.current_path.poses[-1].pose.position
        dist_to_goal = math.sqrt((goal.x - self.robot_x)**2 + (goal.y - self.robot_y)**2)
        if dist_to_goal < self.goal_tolerance:
            self.publish_cmd(0.0, 0.0)
            self.get_logger().info('🏁 Goal Reached!', throttle_duration_sec=2.0)
            return
            
        # 3. Fuzzy Inference
        target_v, target_w = self.fuzzy_inference(e_d, e_theta_deg)
        
        # 4. Safety Limit (Không phải smoothing)
        final_v, final_w = self.limit_wheel_velocities(target_v, target_w)
        
        # 5. Publish Directly
        self.publish_cmd(final_v, final_w)
        
        if self.verbose_logging:
            self.get_logger().info(
                f'Err: {e_d:.2f}m | Cmd: v={final_v:.2f}, w={final_w:.2f}',
                throttle_duration_sec=0.5)

    def publish_cmd(self, v, w):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = float(v)
        cmd.twist.angular.z = float(w)
        self.cmd_vel_pub.publish(cmd)

    def stop_robot(self):
        self.publish_cmd(0.0, 0.0)

    def shutdown_callback(self):
        self.is_shutdown = True
        self.stop_robot()

def main(args=None):
    rclpy.init(args=args)
    controller = FuzzyTrajectoryController()
    def signal_handler(sig, frame):
        if controller: controller.shutdown_callback()
        rclpy.shutdown()
        sys.exit(0)
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt: pass
    except Exception as e: print(f"Error: {e}")
    finally:
        if controller: controller.stop_robot()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()