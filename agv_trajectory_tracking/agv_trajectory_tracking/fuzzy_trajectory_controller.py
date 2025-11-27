#!/usr/bin/env python3
"""
Fuzzy Trajectory Tracking Controller (Clean Shutdown Version)
Author: Thuong Tran Dinh
Updated: November 27, 2025
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import TwistStamped
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import math
import time
import sys

# ... (Giữ nguyên các hàm helper: quaternion_to_euler, trimf, trapmf) ...
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

# ... (Class Controller giữ nguyên logic, chỉ xóa hàm shutdown_callback cũ nếu có) ...
class FuzzyTrajectoryController(Node):
    def __init__(self):
        super().__init__('fuzzy_trajectory_controller')
        
        self.declare_parameter('wheel_base', 0.42)
        self.declare_parameter('max_linear_vel', 0.4) 
        self.declare_parameter('max_angular_vel', 0.8) 
        self.declare_parameter('control_frequency', 20.0)
        self.declare_parameter('goal_tolerance', 0.10)
        self.declare_parameter('verbose_logging', True)
        
        self.wheel_base = self.get_parameter('wheel_base').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.control_freq = self.get_parameter('control_frequency').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.verbose_logging = self.get_parameter('verbose_logging').value
        
        self.max_wheel_vel = 0.5 
        
        self.robot_x = 0.0; self.robot_y = 0.0; self.robot_theta = 0.0
        self.current_path = None; self.path_received = False
        self.controller_ready = False
        
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/diff_cont/cmd_vel', 10)
        self.path_sub = self.create_subscription(Path, '/trajectory', self.path_callback, 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
            
        self.setup_fuzzy_system()
        
        self.startup_timer = self.create_timer(4.0, self.on_controller_ready)
        self.control_timer = self.create_timer(1.0/self.control_freq, self.control_loop)
        
        self.get_logger().info('✅ Fuzzy Controller Initialized')

    def on_controller_ready(self):
        self.controller_ready = True
        self.startup_timer.cancel()
        self.get_logger().info('🚀 Controller Active!')

    def setup_fuzzy_system(self):
        self.e_d_mf = {'VS': ('trap', [0, 0, 0.5, 0.8]), 'S': ('tri', [0.5, 0.8, 1.2]), 'M': ('tri', [0.8, 1.2, 1.5]), 'B': ('tri', [1.2, 1.5, 3.0]), 'VB': ('trap', [1.5, 3.0, 10, 20])}
        self.e_theta_mf = {'NB': ('trap', [-180, -180, -30, -15]), 'NM': ('tri', [-30, -15, -5]), 'NS': ('tri', [-15, -5, -3]), 'ZE': ('tri', [-3, 0, 3]), 'PS': ('tri', [3, 5, 15]), 'PM': ('tri', [5, 15, 30]), 'PB': ('trap', [15, 30, 180, 180])}
        self.angular_vel_constants = {'NB': -1.0, 'NM': -0.7, 'NS': -0.1, 'Z': 0.0, 'PS': 0.1, 'PM': 0.7, 'PB': 1.0}
        self.angular_rules = {'NB': 'NB', 'NM': 'NM', 'NS': 'NS', 'ZE': 'Z', 'PS': 'PS', 'PM': 'PM', 'PB': 'PB'}

    def fuzzify(self, value, mf_dict):
        memberships = {}
        for label, (mf_type, params) in mf_dict.items():
            if mf_type == 'tri': memberships[label] = trimf(value, params)
            elif mf_type == 'trap': memberships[label] = trapmf(value, params)
        return memberships

    def fuzzy_inference(self, e_d, e_theta_deg):
        if abs(e_theta_deg) < 2.0: e_theta_deg = 0.0
        e_theta_fuzz = self.fuzzify(e_theta_deg, self.e_theta_mf)
        omega_num, omega_den = 0.0, 0.0
        for angle_label, omega_label in self.angular_rules.items():
            strength = e_theta_fuzz[angle_label]
            if strength > 0:
                omega_num += strength * self.angular_vel_constants[omega_label]
                omega_den += strength
        omega = omega_num / omega_den if omega_den > 0 else 0.0
        
        abs_angle = abs(e_theta_deg)
        if abs_angle > 75 and e_d < 0.4:
            if e_d < 0.15: v = 0.08
            elif e_d < 0.30: v = 0.12
            else: v = 0.16
            omega = omega * 1.2
        else:
            if e_d < 0.10: v_base = 0.32
            elif e_d < 0.25: v_base = 0.33
            elif e_d < 0.50: v_base = 0.34
            else: v_base = 0.35
            if abs_angle < 15: angle_factor = 1.0
            elif abs_angle < 35: angle_factor = 0.95
            elif abs_angle < 55: angle_factor = 0.90
            elif abs_angle < 70: angle_factor = 0.80
            else: angle_factor = 0.70
            v = v_base * angle_factor
        return v, omega

    def get_robot_pose_from_tf(self):
        try:
            trans = self.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
            self.robot_x = trans.transform.translation.x
            self.robot_y = trans.transform.translation.y
            q = trans.transform.rotation
            _, _, self.robot_theta = quaternion_to_euler(q.x, q.y, q.z, q.w)
            return True
        except (LookupException, ConnectivityException, ExtrapolationException):
            return False

    def path_callback(self, msg):
        self.current_path = msg
        self.path_received = True

    def find_closest_point(self, path, x, y):
        best_idx, best_dist = -1, float('inf')
        for i, pose_stamped in enumerate(path.poses):
            dist = math.sqrt((pose_stamped.pose.position.x - x)**2 + (pose_stamped.pose.position.y - y)**2)
            dx, dy = pose_stamped.pose.position.x - x, pose_stamped.pose.position.y - y
            angle_diff = math.atan2(dy, dx) - self.robot_theta
            while angle_diff > math.pi: angle_diff -= 2*math.pi
            while angle_diff < -math.pi: angle_diff += 2*math.pi
            if abs(angle_diff) < math.pi/2.0 or dist < 0.3:
                if dist < best_dist: best_dist, best_idx = dist, i
        if best_idx == -1: best_idx = 0
        
        min_dist = best_dist
        if min_dist < 0.10: lookahead = 10   
        elif min_dist < 0.20: lookahead = 12 
        elif min_dist < 0.40: lookahead = 15 
        else: lookahead = 20                 
        return min(best_idx + lookahead, len(path.poses) - 1)

    def compute_errors(self):
        if not self.path_received or not self.current_path: return 0.0, 0.0
        target_idx = self.find_closest_point(self.current_path, self.robot_x, self.robot_y)
        ref_pose = self.current_path.poses[target_idx].pose
        dx = ref_pose.position.x - self.robot_x; dy = ref_pose.position.y - self.robot_y
        e_d = math.sqrt(dx*dx + dy*dy)
        target_angle = math.atan2(dy, dx)
        e_theta_rad = target_angle - self.robot_theta
        while e_theta_rad > math.pi: e_theta_rad -= 2*math.pi
        while e_theta_rad < -math.pi: e_theta_rad += 2*math.pi
        return e_d, math.degrees(e_theta_rad)

    def limit_wheel_velocities(self, v, w):
        half_wb = self.wheel_base / 2.0
        v_left = v - (w * half_wb); v_right = v + (w * half_wb)
        max_v = max(abs(v_left), abs(v_right))
        if max_v > self.max_wheel_vel:
            scale = self.max_wheel_vel / max_v; v *= scale; w *= scale
        return v, w

    def control_loop(self):
        if not self.controller_ready: return
        if not self.get_robot_pose_from_tf(): return 
        if not self.path_received or not self.current_path: return
        
        e_d, e_theta_deg = self.compute_errors()
        goal = self.current_path.poses[-1].pose.position
        dist_to_goal = math.sqrt((goal.x - self.robot_x)**2 + (goal.y - self.robot_y)**2)
        
        if dist_to_goal < self.goal_tolerance:
            self.publish_cmd(0.0, 0.0)
            self.get_logger().info('🏁 Goal Reached!', throttle_duration_sec=2.0)
            return
            
        target_v, target_w = self.fuzzy_inference(e_d, e_theta_deg)
        final_v, final_w = self.limit_wheel_velocities(target_v, target_w)
        self.publish_cmd(final_v, final_w)
        
        if self.verbose_logging:
            self.get_logger().info(f'Err: {e_d:.2f}m | Cmd: v={final_v:.2f}, w={final_w:.2f}', throttle_duration_sec=0.5)

    def publish_cmd(self, v, w):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = float(v)
        cmd.twist.angular.z = float(w)
        self.cmd_vel_pub.publish(cmd)

    def stop_robot(self):
        self.publish_cmd(0.0, 0.0)

def main(args=None):
    rclpy.init(args=args)
    controller = FuzzyTrajectoryController()
    
    try:
        # Spin until Ctrl+C
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        # === CRITICAL: GỬI LỆNH DỪNG TRƯỚC KHI SHUTDOWN ===
        try:
            print("\n🛑 STOPPING ROBOT...")
            # Gửi lệnh dừng nhiều lần để đảm bảo STM32 nhận được
            for i in range(10):
                controller.stop_robot()
                time.sleep(0.05)  # Tổng 0.5s
            print("✅ Stop commands sent!")
        except Exception as e:
            print(f"⚠️ Error stopping robot: {e}")
        
        # Sau đó mới destroy node và shutdown
        try:
            controller.destroy_node()
        except:
            pass
            
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()