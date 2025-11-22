#!/usr/bin/env python3
import math
from enum import Enum

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import Path, Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped, Quaternion, PoseWithCovarianceStamped

# Try optional SciPy distance transform (for clearance map from /map)
try:
    from scipy.ndimage import distance_transform_edt
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False


class PlannerState(Enum):
    IDLE = 0
    TRACKING = 1      # Bám đường (path tracking)
    EVASION = 2       # Né vượt (clearance-aware overtake)
    GOAL_CAPTURE = 3  # Tiếp cận đích (dừng chính xác)
    GOAL_REACHED = 4  # Đã đến đích


class HybridLocalPlanner(Node):
    def __init__(self):
        super().__init__('agv_local_planner')

        # ==========================
        # Parameters (ROS 2)
        # ==========================
        # Robot velocity limits (cứng: 0–1 m/s, 0–1 rad/s)
        self.declare_parameter('max_v', 1.0)          # m/s
        self.declare_parameter('max_w', 1.0)          # rad/s
        self.declare_parameter('max_accel', 1.0)      # m/s^2   (tăng tốc)
        self.declare_parameter('max_decel', 3.0)      # m/s^2   (giảm tốc / phanh)
        self.declare_parameter('max_angular_accel', 3.0)  # rad/s^2
        self.declare_parameter('control_rate', 20.0)  # Hz

        # Geometry
        self.declare_parameter('robot_radius', 0.25)  # m (half width)

        # Path tracking gains (gần giống MATLAB)
        self.declare_parameter('track.k_p', 1.8)
        self.declare_parameter('track.k_d', 2.5)
        self.declare_parameter('track.k_w', 0.4)

        # Goal capture parameters
        self.declare_parameter('goal.capture_radius', 0.8)
        self.declare_parameter('goal.k_p_linear', 2.0)
        self.declare_parameter('goal.k_d_linear', 0.4)
        self.declare_parameter('goal.k_p_angular', 2.5)

        # Evasion parameters
        self.declare_parameter('evasion.side_offset', 0.5)     # khoảng cách né sang bên (m)
        self.declare_parameter('evasion.min_clearance', 0.4)   # clearance tối thiểu (m)
        self.declare_parameter('evasion.clearance_penalty', 1000.0)
        self.declare_parameter('evasion.corridor_length_base', 0.5)
        self.declare_parameter('evasion.corridor_length_scale', 1.1)
        self.declare_parameter('evasion.corridor_width_margin', 0.2)
        self.declare_parameter('evasion.emergency_dist', 1.2)

        # Braking behavior (dừng siêu êm)
        self.declare_parameter('brake.start_dist', 2.0)       # bắt đầu phanh mềm khi cách đích < 2m
        self.declare_parameter('brake.safety_factor', 0.6)    # hệ số an toàn cho v² = 2 a s

        # ==========================
        # Load parameter values
        # ==========================
        self.MAX_V = float(self.get_parameter('max_v').value)
        self.MAX_W = float(self.get_parameter('max_w').value)
        self.MAX_ACCEL = float(self.get_parameter('max_accel').value)
        self.MAX_DECEL = float(self.get_parameter('max_decel').value)
        self.MAX_ANG_ACCEL = float(self.get_parameter('max_angular_accel').value)
        self.CONTROL_RATE = float(self.get_parameter('control_rate').value)
        self.DT = 1.0 / self.CONTROL_RATE

        self.ROBOT_RADIUS = float(self.get_parameter('robot_radius').value)

        self.TRACK_KP = float(self.get_parameter('track.k_p').value)
        self.TRACK_KD = float(self.get_parameter('track.k_d').value)
        self.TRACK_KW = float(self.get_parameter('track.k_w').value)

        self.GOAL_CAPTURE_RADIUS = float(self.get_parameter('goal.capture_radius').value)
        self.GOAL_KP_LIN = float(self.get_parameter('goal.k_p_linear').value)
        self.GOAL_KD_LIN = float(self.get_parameter('goal.k_d_linear').value)
        self.GOAL_KP_ANG = float(self.get_parameter('goal.k_p_angular').value)

        self.EV_SIDE_OFFSET = float(self.get_parameter('evasion.side_offset').value)
        self.EV_MIN_CLEARANCE = float(self.get_parameter('evasion.min_clearance').value)
        self.EV_CLEARANCE_PENALTY = float(self.get_parameter('evasion.clearance_penalty').value)
        self.EV_CORRIDOR_LEN_BASE = float(self.get_parameter('evasion.corridor_length_base').value)
        self.EV_CORRIDOR_LEN_SCALE = float(self.get_parameter('evasion.corridor_length_scale').value)
        self.EV_CORRIDOR_WIDTH_MARGIN = float(self.get_parameter('evasion.corridor_width_margin').value)
        self.EV_EMERGENCY_DIST = float(self.get_parameter('evasion.emergency_dist').value)

        self.BRAKE_START_DIST = float(self.get_parameter('brake.start_dist').value)
        self.BRAKE_SAFETY_FACTOR = float(self.get_parameter('brake.safety_factor').value)

        # ==========================
        # Internal state variables
        # ==========================
        self.state = PlannerState.IDLE

        self.path_points = None       # Nx2 (x,y) trong frame map
        self.current_pose = None      # [x, y, yaw] trong map
        self.current_v = 0.0
        self.current_w = 0.0

        self.last_cte = 0.0           # cross-track error trước đó (cho D term)

        # LaserScan → obstacle points in robot frame (Nx2)
        self.obstacle_points_robot = np.empty((0, 2), dtype=float)

        # Evasion state
        self.is_evading = False
        self.evasion_target_world = None   # [x,y]
        self.evaded_obstacle_world = None  # [x,y]

        # Clearance map từ OccupancyGrid (tùy chọn)
        self.dist_field = None
        self.map_res = None
        self.map_width = None
        self.map_height = None
        self.map_origin_x = None
        self.map_origin_y = None

        # ==========================
        # ROS 2 Interfaces
        # ==========================
        
        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers
        self.cmd_pub = self.create_publisher(TwistStamped, '/diff_cont/cmd_vel', reliable_qos)

        # Subscribers
        self.create_subscription(Path, '/global_path', self.path_callback, reliable_qos)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_callback, reliable_qos)
        self.create_subscription(Odometry, '/odometry/filtered', self.odom_velocity_callback, reliable_qos)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, sensor_qos)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)

        self.timer = self.create_timer(self.DT, self.control_loop)

        if not SCIPY_AVAILABLE:
            self.get_logger().warn(
                "SciPy not found. Clearance map from /map disabled. "
                "Evasion will still work but without wall-aware clearance penalty."
            )

        self.get_logger().info(
            f"🚀 Hybrid Local Planner started. "
            f"MAX_V={self.MAX_V:.2f} m/s, MAX_W={self.MAX_W:.2f} rad/s"
        )

    # ======================================================
    # ROS Callbacks
    # ======================================================
    def path_callback(self, msg: Path):
        if len(msg.poses) == 0:
            self.path_points = None
            self.state = PlannerState.IDLE
            self.get_logger().warn("Received empty /global_path. Switching to IDLE.")
            return

        pts = []
        for ps in msg.poses:
            pts.append([ps.pose.position.x, ps.pose.position.y])
        self.path_points = np.array(pts, dtype=float)

        self.state = PlannerState.TRACKING
        self.is_evading = False
        self.evasion_target_world = None
        self.evaded_obstacle_world = None

        self.get_logger().info(f"📌 New global path received: {len(pts)} points. State → TRACKING.")

    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        """Callback for /amcl_pose - get corrected pose from AMCL"""
        pos = msg.pose.pose.position
        ori: Quaternion = msg.pose.pose.orientation
        _, _, yaw = self.quaternion_to_euler(ori)

        self.current_pose = np.array([pos.x, pos.y, yaw], dtype=float)

    def odom_velocity_callback(self, msg: Odometry):
        """Callback for /odometry/filtered - get velocity only"""
        self.current_v = float(msg.twist.twist.linear.x)
        self.current_w = float(msg.twist.twist.angular.z)

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=float)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        valid = np.isfinite(ranges)
        ranges = ranges[valid]
        angles = angles[valid]

        xs = ranges * np.cos(angles)
        ys = ranges * np.sin(angles)

        self.obstacle_points_robot = np.stack([xs, ys], axis=1)

    def map_callback(self, msg: OccupancyGrid):
        if not SCIPY_AVAILABLE:
            return

        self.map_res = msg.info.resolution
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y

        data = np.array(msg.data, dtype=np.int16).reshape((self.map_height, self.map_width))
        # occupied = value > 50
        occ = (data > 50)

        # Free = ~occ → distance to nearest obstacle
        free = ~occ
        # distance in cells
        dist_cells = distance_transform_edt(free)
        # distance in meters
        self.dist_field = dist_cells * self.map_res

        self.get_logger().info("✅ Clearance map computed from /map (distance transform ready).")

    # ======================================================
    # Main Control Loop
    # ======================================================
    def control_loop(self):
        # Need pose + path
        if self.current_pose is None or self.path_points is None:
            self.publish_cmd(0.0, 0.0)
            return

        # Distance to final goal
        final_goal = self.path_points[-1, :]
        dist_to_goal = np.linalg.norm(self.current_pose[:2] - final_goal)

        # Update state machine
        self.update_state(dist_to_goal)

        # Compute target v, w from controllers
        target_v, target_w = 0.0, 0.0

        if self.state == PlannerState.IDLE:
            target_v, target_w = 0.0, 0.0

        elif self.state == PlannerState.GOAL_REACHED:
            target_v, target_w = 0.0, 0.0

        elif self.state == PlannerState.GOAL_CAPTURE:
            target_v, target_w = self.goal_capture_controller(dist_to_goal, final_goal)

        elif self.state == PlannerState.EVASION:
            target_v, target_w = self.evasion_controller(dist_to_goal, final_goal)

        elif self.state == PlannerState.TRACKING:
            target_v, target_w = self.path_tracking_controller(dist_to_goal)

        # Apply acceleration limits and clamp to [0, MAX]
        cmd_v, cmd_w = self.rate_limit_and_clamp(target_v, target_w)

        # Publish
        self.publish_cmd(cmd_v, cmd_w)

    # ======================================================
    # State Machine Logic
    # ======================================================
    def update_state(self, dist_to_goal: float):
        # Already IDLE or GOAL_REACHED: nothing to do until new path
        if self.state in [PlannerState.GOAL_REACHED, PlannerState.IDLE] and self.path_points is None:
            return

        # If no path anymore
        if self.path_points is None:
            self.state = PlannerState.IDLE
            return

        # 1. Check goal reached condition (hard)
        if dist_to_goal < 0.15 and abs(self.current_v) < 0.05:
            if self.state != PlannerState.GOAL_REACHED:
                self.get_logger().info("✅ Goal reached. Switching to GOAL_REACHED.")
            self.state = PlannerState.GOAL_REACHED
            return

        # 2. If we are currently evading, check if we can exit EVASION
        if self.state == PlannerState.EVASION and self.is_evading:
            if self.evasion_target_world is not None:
                if np.linalg.norm(self.current_pose[:2] - self.evasion_target_world) < 0.5:
                    # Reached evasion target → back to tracking
                    self.is_evading = False
                    self.evasion_target_world = None
                    self.evaded_obstacle_world = None
                    self.get_logger().info("Evasion target reached. Switching back to TRACKING.")
                    self.state = PlannerState.TRACKING
                    return

        # 3. Emergency check for EVASION
        emergency, obs_world = self.check_emergency_threat()
        if emergency and not self.is_evading:
            # Plan evasion target
            self.is_evading = True
            self.evaded_obstacle_world = obs_world
            self.evasion_target_world = self.plan_evasion_target(obs_world, self.path_points[-1, :])
            if self.evasion_target_world is None:
                # fallback: cannot plan -> just stay in TRACKING
                self.is_evading = False
            else:
                self.get_logger().warn("⚠️ Emergency threat detected. Switching to EVASION.")
                self.state = PlannerState.EVASION
                return

        # 4. If close to goal → GOAL_CAPTURE
        if dist_to_goal < self.GOAL_CAPTURE_RADIUS and not self.is_evading:
            if self.state != PlannerState.GOAL_CAPTURE:
                self.get_logger().info("Near goal. Switching to GOAL_CAPTURE.")
            self.state = PlannerState.GOAL_CAPTURE
            return

        # 5. Otherwise → TRACKING
        if not self.is_evading:
            if self.state != PlannerState.TRACKING:
                self.get_logger().info("Switching to TRACKING.")
            self.state = PlannerState.TRACKING

    # ======================================================
    # EMERGENCY / EVASION
    # ======================================================
    def check_emergency_threat(self):
        """
        Kiểm tra xem trong "forward corridor" trước robot
        có vật cản nào quá gần không.
        Trả về: (bool, obstacle_world_pos or None)
        """
        if self.obstacle_points_robot.shape[0] == 0:
            return False, None

        # Corridor length phụ thuộc v hiện tại (giống MATLAB)
        corridor_length = self.EV_CORRIDOR_LEN_BASE + self.current_v * self.EV_CORRIDOR_LEN_SCALE
        corridor_width = self.ROBOT_RADIUS + self.EV_CORRIDOR_WIDTH_MARGIN

        pts = self.obstacle_points_robot
        xs = pts[:, 0]
        ys = pts[:, 1]
        dists = np.linalg.norm(pts, axis=1)

        # Các điểm phía trước, trong khoảng (0, emergency_dist) và gần trục x hơn
        mask = (xs > 0.0) & \
               (xs < min(corridor_length, self.EV_EMERGENCY_DIST)) & \
               (np.abs(ys) < (corridor_width / 2.0)) & \
               (dists > 0.05)

        if not np.any(mask):
            return False, None

        # Chọn obstacle gần nhất
        idxs = np.where(mask)[0]
        closest_idx = idxs[np.argmin(dists[idxs])]
        obs_robot = pts[closest_idx]

        # Chuyển sang world
        obs_world = self.robot_to_world(obs_robot)

        return True, obs_world

    def plan_evasion_target(self, obs_world: np.ndarray, final_goal_world: np.ndarray):
        """
        Giống MATLAB: sinh 2 điểm né trái/phải quanh obstacle,
        rồi chọn cái:
        - đường đi ngắn hơn
        - clearance với tường tốt hơn (nếu có dist_field từ /map).
        """
        robot_pos = self.current_pose[:2]
        vec_to_obs = obs_world - robot_pos
        dist_obs = np.linalg.norm(vec_to_obs)
        if dist_obs < 1e-6:
            return None

        # Perpendicular vectors
        perp = np.array([-vec_to_obs[1], vec_to_obs[0]]) / dist_obs

        # Giả sử bán kính obstacle xấp xỉ robot_radius (Scan không cho rõ)
        obs_radius = self.ROBOT_RADIUS
        total_offset = obs_radius + self.EV_SIDE_OFFSET

        evade1 = obs_world + perp * total_offset
        evade2 = obs_world - perp * total_offset

        # COST: distance to goal + clearance penalty
        cost1 = self.evasion_cost(evade1, robot_pos, final_goal_world)
        cost2 = self.evasion_cost(evade2, robot_pos, final_goal_world)

        # Chọn tốt hơn
        if cost1 < cost2:
            return evade1
        else:
            return evade2

    def evasion_cost(self, evade_point, robot_pos, goal_pos):
        # Distance cost
        dist_cost = np.linalg.norm(robot_pos - evade_point) + np.linalg.norm(evade_point - goal_pos)

        # Clearance from static walls (nếu có map)
        if self.dist_field is not None:
            clearance = self.get_clearance_at_world(evade_point)
            if clearance < self.EV_MIN_CLEARANCE:
                # Coi như cực xấu
                return float('inf')
            clear_penalty = self.EV_CLEARANCE_PENALTY / max(clearance, 0.1)
            return dist_cost + clear_penalty
        else:
            # Không có map: chỉ dùng distance cost
            return dist_cost

    def evasion_controller(self, dist_to_goal, final_goal):
        """
        Khi đang ở trạng thái EVASION:
        - Hướng tới evasion_target_world
        - Dùng controller giống PATH_TRACKING nhưng không dùng crosstrack,
          chỉ dùng alpha_err tới target.
        """
        if self.evasion_target_world is None:
            # Không có target, fallback TRACKING
            return self.path_tracking_controller(dist_to_goal)

        target = self.evasion_target_world
        dx = target[0] - self.current_pose[0]
        dy = target[1] - self.current_pose[1]
        angle_to_target = math.atan2(dy, dx)

        alpha_err = self.normalize_angle(angle_to_target - self.current_pose[2])

        # Giống MATLAB: target_w = (2 * max_v * sin(alpha) / L) - k_w * w
        # Ở đây ta coi "L hiệu dụng" ~ 1.0
        d_term_w = self.TRACK_KW * self.current_w
        target_w = 2.0 * self.MAX_V * math.sin(alpha_err) / 1.0 - d_term_w
        target_w = max(-self.MAX_W, min(self.MAX_W, target_w))

        # Linear velocity giảm khi góc lớn
        v_factor = max(0.1, 1.0 - 1.5 * abs(alpha_err))
        target_v = self.MAX_V * v_factor

        # Nếu sắp tới goal, ghim thêm braking curve
        if dist_to_goal < self.BRAKE_START_DIST:
            v_brake = self.compute_braking_speed(dist_to_goal)
            target_v = min(target_v, v_brake)

        return target_v, target_w

    # ======================================================
    # PATH TRACKING CONTROLLER (MATLAB-like)
    # ======================================================
    def path_tracking_controller(self, dist_to_goal):
        """
        PID-like controller bám theo path:
        - cross-track error + heading error
        - dùng công thức alpha_err giống MATLAB
        """
        if self.path_points is None or self.path_points.shape[0] < 2:
            return 0.0, 0.0

        crosstrack_error, heading_error, _ = self.compute_path_errors()
        cte_dot = (crosstrack_error - self.last_cte) / self.DT
        self.last_cte = crosstrack_error

        v_forward = max(self.current_v, 0.4)  # tránh chia cho 0

        alpha_err = heading_error + math.atan2(
            self.TRACK_KP * crosstrack_error + self.TRACK_KD * cte_dot,
            max(v_forward, 0.1)
        )

        # Angular control
        d_term_w = self.TRACK_KW * self.current_w
        target_w = 2.0 * self.MAX_V * math.sin(alpha_err) / 1.0 - d_term_w
        target_w = max(-self.MAX_W, min(self.MAX_W, target_w))

        # Linear control: giảm tốc khi quay mạnh
        v_factor = max(0.1, 1.0 - 1.5 * abs(alpha_err))
        base_v = self.MAX_V * v_factor

        # Nếu gần goal: dùng braking curve v² = 2 a s
        if dist_to_goal < self.BRAKE_START_DIST:
            v_brake = self.compute_braking_speed(dist_to_goal)
            target_v = min(base_v, v_brake)
        else:
            target_v = base_v

        return target_v, target_w

    def compute_path_errors(self):
        """
        Tương đương path_tracking_errors trong MATLAB:
        - tìm điểm gần nhất trên path
        - lấy segment để tính heading_error
        - cross-track error = signed distance tới segment
        """
        pos = self.current_pose[:2]
        pts = self.path_points

        dx = pos[0] - pts[:, 0]
        dy = pos[1] - pts[:, 1]
        dist_sq = dx * dx + dy * dy
        closest_idx = int(np.argmin(dist_sq))

        # target_idx dùng lookahead (ở đây không cần trả về, nhưng giữ cho giống MATLAB)
        target_idx = min(closest_idx + 10, pts.shape[0] - 1)

        # lấy 2 điểm để tính vector path
        if closest_idx < pts.shape[0] - 1:
            p1 = pts[closest_idx, :]
            p2 = pts[closest_idx + 1, :]
        else:
            p1 = pts[closest_idx - 1, :]
            p2 = pts[closest_idx, :]

        path_vec = p2 - p1
        path_angle = math.atan2(path_vec[1], path_vec[0])

        # vector từ path tới robot
        robot_vec = pos - p1

        # cross-track error: signed
        crosstrack_error = (robot_vec[0] * path_vec[1] - robot_vec[1] * path_vec[0]) / max(
            np.linalg.norm(path_vec), 1e-6
        )

        heading_error = self.normalize_angle(path_angle - self.current_pose[2])

        return crosstrack_error, heading_error, target_idx

    # ======================================================
    # GOAL CAPTURE CONTROLLER
    # ======================================================
    def goal_capture_controller(self, dist_to_goal, final_goal):
        """
        Dừng chính xác tại đích (không overshoot):
        - Điều khiển P trên góc + khoảng cách
        - Giảm v khi góc lệch lớn
        - Áp dụng braking curve v² = 2 a s
        """
        dx = final_goal[0] - self.current_pose[0]
        dy = final_goal[1] - self.current_pose[1]
        angle_to_goal = math.atan2(dy, dx)

        angle_error = self.normalize_angle(angle_to_goal - self.current_pose[2])

        # Angular P-control
        target_w = self.GOAL_KP_ANG * angle_error
        target_w = max(-self.MAX_W, min(self.MAX_W, target_w))

        # Linear P-control với damping + giảm theo góc
        v_scale = max(0.0, 1.0 - 2.5 * abs(angle_error) / math.pi)
        base_v = self.GOAL_KP_LIN * dist_to_goal * v_scale
        base_v = max(0.0, min(self.MAX_V, base_v))

        # Thêm braking curve v² = 2 a s
        v_brake = self.compute_braking_speed(dist_to_goal)
        target_v = min(base_v, v_brake)

        return target_v, target_w

    # ======================================================
    # BRAKING SPEED HELPER
    # ======================================================
    def compute_braking_speed(self, dist_to_goal: float) -> float:
        """
        Tính vận tốc "an toàn" để còn dừng kịp với gia tốc phanh MAX_DECEL:
            v^2 = 2 * a * s   =>  v = sqrt(2 a s)
        Thêm safety_factor để robot dừng sớm hơn một chút (êm hơn).
        """
        if dist_to_goal <= 0.0 or self.MAX_DECEL <= 0.0:
            return 0.0

        s_eff = max(0.0, dist_to_goal * self.BRAKE_SAFETY_FACTOR)
        v_brake = math.sqrt(2.0 * self.MAX_DECEL * s_eff)

        return min(self.MAX_V, v_brake)

    # ======================================================
    # VELOCITY SHAPING / LIMITING
    # ======================================================
    def rate_limit_and_clamp(self, target_v, target_w):
        """
        Giới hạn tăng/giảm tốc:
          - Nếu tăng tốc: dùng MAX_ACCEL
          - Nếu phanh (giảm tốc): dùng MAX_DECEL
        """
        # Linear velocity rate limit
        dv = target_v - self.current_v
        if dv >= 0.0:
            max_dv = self.MAX_ACCEL * self.DT
        else:
            max_dv = self.MAX_DECEL * self.DT

        dv_clamped = max(-max_dv, min(max_dv, dv))
        new_v = self.current_v + dv_clamped

        # Angular velocity rate limit
        dw = target_w - self.current_w
        max_dw = self.MAX_ANG_ACCEL * self.DT
        dw_clamped = max(-max_dw, min(max_dw, dw))
        new_w = self.current_w + dw_clamped

        # Clamp to allowed ranges
        new_v = max(0.0, min(self.MAX_V, new_v))  # không cho chạy lùi ở đây
        new_w = max(-self.MAX_W, min(self.MAX_W, new_w))

        # Cập nhật trạng thái
        self.current_v = new_v
        self.current_w = new_w

        return new_v, new_w

    # ======================================================
    # MAP CLEARANCE UTILITIES
    # ======================================================
    def get_clearance_at_world(self, point):
        if self.dist_field is None:
            return 0.0

        x, y = point[0], point[1]

        ix = int((x - self.map_origin_x) / self.map_res)
        iy = int((y - self.map_origin_y) / self.map_res)

        if ix < 0 or iy < 0 or ix >= self.map_width or iy >= self.map_height:
            return 0.0

        # dist_field đã là mét
        return float(self.dist_field[iy, ix])

    # ======================================================
    # Helper / Utils
    # ======================================================
    def publish_cmd(self, v, w):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        msg.twist.linear.x = float(v)
        msg.twist.angular.z = float(w)

        self.cmd_pub.publish(msg)

    @staticmethod
    def quaternion_to_euler(q: Quaternion):
        x, y, z, w = q.x, q.y, q.z, q.w
        t0 = 2.0 * (w * x + y * z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = 2.0 * (w * y - z * x)
        t2 = max(-1.0, min(1.0, t2))
        pitch = math.asin(t2)

        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw

    @staticmethod
    def normalize_angle(angle):
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    def robot_to_world(self, p_robot):
        """p_robot: [x,y] trong frame base_link"""
        x_r, y_r = p_robot[0], p_robot[1]
        x, y, yaw = self.current_pose

        world_x = x_r * math.cos(yaw) - y_r * math.sin(yaw) + x
        world_y = x_r * math.sin(yaw) + y_r * math.cos(yaw) + y
        return np.array([world_x, world_y])

    def world_to_robot(self, p_world):
        """p_world: [x,y] trong frame map, trả về [x,y] trong base_link."""
        x_w, y_w = p_world[0], p_world[1]
        x, y, yaw = self.current_pose

        dx = x_w - x
        dy = y_w - y

        local_x = dx * math.cos(yaw) + dy * math.sin(yaw)
        local_y = -dx * math.sin(yaw) + dy * math.cos(yaw)
        return np.array([local_x, local_y])


def main(args=None):
    rclpy.init(args=args)
    node = HybridLocalPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.publish_cmd(0.0, 0.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
