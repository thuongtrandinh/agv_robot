#!/usr/bin/env python3
"""
Fuzzy Trajectory Tracking Controller (PRO VERSION)
- Subscribes /global_path
- Publishes /diff_cont/cmd_vel
- Includes:
    ✔ Curvature-based speed reduction
    ✔ Anti-overshoot for sharp turns
    ✔ Safe wheel velocity limiting
"""

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import TwistStamped, PoseWithCovarianceStamped
import math
import signal
import sys


def quaternion_to_euler(x, y, z, w):
    """Convert quaternion to roll, pitch, yaw."""
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = max(-1.0, min(+1.0, t2))
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw


# ==============================
# Membership functions
# ==============================
def trimf(x, params):
    a, b, c = params
    if x <= a or x >= c:
        return 0.0
    elif x == b:
        return 1.0
    elif x < b:
        return (x - a) / (b - a)
    else:
        return (c - x) / (c - b)


def trapmf(x, params):
    a, b, c, d = params
    if x <= a or x >= d:
        return 0.0
    elif b <= x <= c:
        return 1.0
    elif x < b:
        return (x - a) / (b - a)
    else:
        return (d - x) / (d - c)


# ============================================================
# MAIN CLASS – Fuzzy Trajectory Controller
# ============================================================
class FuzzyTrajectoryController(Node):

    def __init__(self):
        super().__init__('fuzzy_path_controller')

        # Parameters
        self.declare_parameter('wheel_base', 0.46)
        self.declare_parameter('max_linear_vel', 1.0)
        self.declare_parameter('max_angular_vel', 1.0)
        self.declare_parameter('control_frequency', 20.0)
        self.declare_parameter('goal_tolerance', 0.15)

        self.wheel_base = self.get_parameter('wheel_base').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.control_freq = self.get_parameter('control_frequency').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.max_wheel_vel = 1.0  # hardware limit

        # States
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0

        self.current_path = None
        self.path_received = False
        self.is_shutdown = False

        # ROS2 interfaces
        self.cmd_pub = self.create_publisher(TwistStamped, '/diff_cont/cmd_vel', 10)

        self.create_subscription(
            Path, '/global_path', self.path_callback, 10)

        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, 10)

        # Init fuzzy system
        self.setup_fuzzy_system()

        # Timer
        self.control_timer = self.create_timer(1.0 / self.control_freq, self.control_loop)

        self.get_logger().info("🚀 Fuzzy Controller PRO Started (Curvature + Anti-Overshoot)")

    # ============================================================
    # Membership Functions + Rules
    # ============================================================
    def setup_fuzzy_system(self):

        self.e_theta_mf = {
            'NB': ('trap', [-180, -180, -45, -25]),
            'NM': ('tri', [-45, -25, -10]),
            'NS': ('tri', [-25, -10, -2]),
            'ZE': ('tri', [-3, 0, 3]),
            'PS': ('tri', [2, 10, 25]),
            'PM': ('tri', [10, 25, 45]),
            'PB': ('trap', [25, 45, 180, 180])
        }

        self.angular_vel_constants = {
            'NB': -1.0,
            'NM': -0.6,
            'NS': -0.3,
            'Z': 0.0,
            'PS': 0.3,
            'PM': 0.6,
            'PB': 1.0,
        }

        self.angular_rules = {
            'NB': 'NB',
            'NM': 'NM',
            'NS': 'NS',
            'ZE': 'Z',
            'PS': 'PS',
            'PM': 'PM',
            'PB': 'PB',
        }

    # ============================================================
    # Callback Functions
    # ============================================================
    def amcl_callback(self, msg):
        """Update robot pose"""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        _, _, self.robot_theta = quaternion_to_euler(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )

    def path_callback(self, msg):
        self.current_path = msg
        self.path_received = True
        self.get_logger().info(f"📌 New global path: {len(msg.poses)} points")

    # ============================================================
    # Utility Functions
    # ============================================================
    def compute_curvature(self, idx):
        """Curvature κ of the path at index."""
        if self.current_path is None or len(self.current_path.poses) < 3:
            return 0.0

        idx_prev = max(0, idx - 1)
        idx_next = min(len(self.current_path.poses) - 1, idx + 1)

        p0 = self.current_path.poses[idx_prev].pose.position
        p1 = self.current_path.poses[idx].pose.position
        p2 = self.current_path.poses[idx_next].pose.position

        x1 = p1.x - p0.x
        y1 = p1.y - p0.y
        x2 = p2.x - p1.x
        y2 = p2.y - p1.y

        denom = (math.sqrt(x1*x1 + y1*y1) * math.sqrt(x2*x2 + y2*y2))
        if denom < 1e-6:
            return 0.0

        curvature = abs(x1*y2 - y1*x2) / denom
        return curvature

    def find_closest_point(self):
        """Return index of closest path point."""
        min_dist = float("inf")
        idx = 0

        for i, ps in enumerate(self.current_path.poses):
            px = ps.pose.position.x
            py = ps.pose.position.y
            d = math.sqrt((px - self.robot_x)**2 + (py - self.robot_y)**2)
            if d < min_dist:
                min_dist = d
                idx = i

        return idx

    def compute_errors(self):
        idx = self.find_closest_point()

        # Add small lookahead to avoid oscillation
        idx_target = min(idx + 2, len(self.current_path.poses) - 1)

        target = self.current_path.poses[idx_target].pose.position
        dx = target.x - self.robot_x
        dy = target.y - self.robot_y

        e_d = math.sqrt(dx*dx + dy*dy)
        heading = math.atan2(dy, dx)
        e_theta = heading - self.robot_theta

        # Normalize angle
        while e_theta > math.pi: e_theta -= 2*math.pi
        while e_theta < -math.pi: e_theta += 2*math.pi

        return e_d, math.degrees(e_theta), idx_target

    # ============================================================
    # Fuzzy Controller
    # ============================================================
    def fuzzify(self, value, mf_dict):
        out = {}
        for label, (typ, params) in mf_dict.items():
            if typ == 'tri':
                out[label] = trimf(value, params)
            else:
                out[label] = trapmf(value, params)
        return out

    def fuzzy_inference(self, e_d, e_theta_deg, curvature):
        """Sugeno fuzzy controller with curvature-based speed adjustment"""

        # ---------- ANGULAR VELOCITY ----------
        fuzz = self.fuzzify(e_theta_deg, self.e_theta_mf)

        num = 0
        den = 0
        for label, weight in fuzz.items():
            if weight > 0:
                omega_label = self.angular_rules[label]
                omega_val = self.angular_vel_constants[omega_label]
                num += weight * omega_val
                den += weight

        omega = num / den if den > 0 else 0.0

        # ---------- LINEAR VELOCITY ----------
        if e_d < 0.2:
            v = 0.25
        elif e_d < 1.0:
            v = 0.35 + 0.45 * e_d
        else:
            v = 1.0

        # Reduce by curvature  (anti-overshoot)
        curvature_factor = max(0.55, 1.0 / (1.0 + 1.2 * curvature))
        v *= curvature_factor

        # Reduce by turning angle
        angle_factor = 1 - 0.18 * min(abs(e_theta_deg) / 90.0, 1.0)

        v *= angle_factor

        # Reduce by angular velocity magnitude
        turn_factor = 1 - 0.25 * abs(omega)
        v *= max(0.55, turn_factor)

        # Clip velocities
        v = max(0, min(self.max_linear_vel, v))
        omega = max(-self.max_angular_vel, min(self.max_angular_vel, omega))

        return v, omega

    # ============================================================
    # Wheel speed limits
    # ============================================================
    def limit_wheels(self, v, omega):
        L = self.wheel_base / 2
        vL = v - omega * L
        vR = v + omega * L

        max_wheel = max(abs(vL), abs(vR))
        if max_wheel > self.max_wheel_vel:
            scale = self.max_wheel_vel / max_wheel
            v *= scale
            omega *= scale

        return v, omega

    # ============================================================
    # Control Loop
    # ============================================================
    def control_loop(self):

        if not self.path_received or self.current_path is None:
            return

        # Goal reached
        goal = self.current_path.poses[-1].pose.position
        dist_goal = math.dist((self.robot_x, self.robot_y), (goal.x, goal.y))
        if dist_goal < self.goal_tolerance:
            return self.publish_cmd(0.0, 0.0)

        # Compute errors
        e_d, e_theta_deg, idx_target = self.compute_errors()

        # Curvature
        curvature = self.compute_curvature(idx_target)

        # Controller
        v, omega = self.fuzzy_inference(e_d, e_theta_deg, curvature)

        # Wheel limit safety
        v, omega = self.limit_wheels(v, omega)

        # Publish
        self.publish_cmd(v, omega)

        self.get_logger().info(
            f"eD={e_d:.2f} | eT={e_theta_deg:.1f}° | κ={curvature:.3f} | v={v:.2f} | ω={omega:.2f}",
            throttle_duration_sec=0.3
        )

    def publish_cmd(self, v, omega):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = v
        msg.twist.angular.z = omega
        self.cmd_pub.publish(msg)

# ============================================================
# MAIN
# ============================================================
def main(args=None):
    rclpy.init(args=args)
    controller = FuzzyTrajectoryController()

    def sigint(sig, frame):
        controller.publish_cmd(0.0, 0.0)
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, sigint)

    rclpy.spin(controller)
    controller.publish_cmd(0.0, 0.0)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
