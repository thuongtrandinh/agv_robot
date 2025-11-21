#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path, OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Quaternion

from tf2_ros import Buffer, TransformListener

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import numpy as np
import math
from scipy.spatial import cKDTree
from scipy.interpolate import splprep, splev
from scipy.ndimage import binary_dilation


class AgvIrttStarNode(Node):
    def __init__(self):
        super().__init__('agv_global_planner')

        # =======================
        # Tham số ROS 2
        # =======================
        self.declare_parameter('max_iter', 2000)
        self.declare_parameter('step_len', 0.15)
        self.declare_parameter('goal_sample_rate', 0.15)
        self.declare_parameter('search_radius', 1.0)
        self.declare_parameter('path_resolution', 0.1)
        self.declare_parameter('enable_smoothing', True)
        self.declare_parameter('robot_radius', 0.25)     # giảm mạnh cho map 5x10m
        self.declare_parameter('safety_margin', 0.01)    # giảm mạnh inflation
        self.declare_parameter('alpha', 2.0)


        self.max_iter = self.get_parameter('max_iter').value
        self.step_len = self.get_parameter('step_len').value
        self.goal_sample_rate = self.get_parameter('goal_sample_rate').value
        self.search_radius = self.get_parameter('search_radius').value
        self.path_resolution = self.get_parameter('path_resolution').value
        self.enable_smoothing = self.get_parameter('enable_smoothing').value
        self.robot_radius = self.get_parameter('robot_radius').value
        self.safety_margin = self.get_parameter('safety_margin').value
        self.alpha = self.get_parameter('alpha').value

        # Wall margin = robot footprint + safety margin (y chang Nav2 inflation)
        self.wall_margin = self.robot_radius + self.safety_margin
        # =======================
        # Publisher
        # =======================
        self.path_pub = self.create_publisher(Path, '/global_path', 10)

        # =======================
        # QoS cho /map
        # =======================
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        # =======================
        # Subscribers
        # =======================
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos
        )

        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)

        # =======================
        # Variables
        # =======================
        self.robot_position = None
        self.robot_orientation = None  # yaw

        self.start = None
        self.goal = None

        self.map_data = None          # binary map đã inflate (Nav2 style)
        self.map_res = None
        self.map_origin = None
        self.map_width = None
        self.map_height = None
        self.map_frame_id = "map"

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info("🚀 Global planner started. Waiting for map...")

    # ============================================
    # MAP CALLBACK
    # ============================================
    def map_callback(self, msg: OccupancyGrid):
        self.get_logger().info(f"🗺 Map received {msg.info.width}x{msg.info.height}")

        self.map_res = msg.info.resolution
        self.map_origin = msg.info.origin.position
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_frame_id = msg.header.frame_id

        data = np.array(msg.data, dtype=np.int8).reshape((self.map_height, self.map_width))
        # Đảm bảo chiều dữ liệu đúng với file .pgm/.yaml
        # Nếu vẫn lỗi, thử transpose để kiểm tra
        # data = np.transpose(data)  # Nếu cần, thử mở dòng này

        # Obstacles gốc
        obstacles = (data > 50)

        # ✅ Nav2-like: inflation_layer = obstacles nở ra theo wall_margin
        inflate_cells = max(1, int(self.wall_margin / self.map_res))
        self.get_logger().info(f"🔧 Inflation: wall_margin={self.wall_margin:.3f} m -> {inflate_cells} cells")

        self.map_data = binary_dilation(obstacles, iterations=inflate_cells)

        # Free space points (dùng cho RRT sampling)
        free_ys, free_xs = np.where(~self.map_data)
        self.free_points = np.stack([
            free_xs * self.map_res + self.map_origin.x,
            free_ys * self.map_res + self.map_origin.y
        ], axis=1)
        
        total_cells = self.map_width * self.map_height
        free_cells = len(free_ys)
        occupied_ratio = 100.0 * (1 - free_cells / total_cells)
        self.get_logger().info(f"📊 Map stats: {free_cells}/{total_cells} free cells ({occupied_ratio:.1f}% occupied after inflation)")

    # ============================================
    # ODOM CALLBACK
    # ============================================
    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation

        _, _, yaw = self.quaternion_to_euler(ori)

        self.robot_position = np.array([pos.x, pos.y])
        self.robot_orientation = yaw

    def quaternion_to_euler(self, q: Quaternion):
        x, y, z, w = q.x, q.y, q.z, q.w

        t0 = 2*(w*x + y*z)
        t1 = 1 - 2*(x*x + y*y)
        roll = math.atan2(t0, t1)

        t2 = 2*(w*y - z*x)
        t2 = np.clip(t2, -1, 1)
        pitch = math.asin(t2)

        t3 = 2*(w*z + x*y)
        t4 = 1 - 2*(y*y + z*z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw

    # ============================================
    # GOAL CALLBACK
    # ============================================
    def goal_callback(self, msg: PoseStamped):
        if self.map_data is None:
            self.get_logger().warn("❌ No map. Cannot plan.")
            return

        # Lấy start pose từ TF
        try:
            tr = self.tf_buffer.lookup_transform(self.map_frame_id, "base_footprint", rclpy.time.Time())
            self.start = np.array([tr.transform.translation.x, tr.transform.translation.y])

            q = tr.transform.rotation
            _, _, self.start_yaw = self.quaternion_to_euler(q)

        except Exception as e:
            self.get_logger().error(f"TF error: {e}")
            return

        # Goal position
        self.goal = np.array([msg.pose.position.x, msg.pose.position.y])

        # Goal yaw
        _, _, self.goal_yaw = self.quaternion_to_euler(msg.pose.orientation)

        self.get_logger().info(f"🎯 Planning from {self.start} to {self.goal}")
        
        # Kiểm tra start/goal có hợp lệ không
        if self.is_collision(self.start):
            self.get_logger().error(f"❌ START position {self.start} is in collision! Cannot plan.")
            return
        if self.is_collision(self.goal):
            self.get_logger().error(f"❌ GOAL position {self.goal} is in collision! Cannot plan.")
            return
        
        self.get_logger().info("✅ Start and goal are collision-free")
        self.run_planner()

    # ============================================
    # UTILS
    # ============================================
    def world_to_map(self, p):
        mx = int((p[0] - self.map_origin.x) / self.map_res)
        my = int((p[1] - self.map_origin.y) / self.map_res)
        return mx, my

    def is_collision(self, p):
        """
        Chuẩn Nav2 style:
        - map_data đã inflate theo robot footprint + safety.
        - Ở đây chỉ cần check 1 ô (cell) là đủ.
        """
        mx, my = self.world_to_map(p)

        if 0 <= mx < self.map_width and 0 <= my < self.map_height:
            return self.map_data[my, mx]   # True = collision
        return True                        # ngoài map = tường

    def line_of_sight(self, p1, p2):
        """
        Giống kiểu planner check costmap:
        - Sample dọc đoạn thẳng.
        - Mỗi điểm gọi is_collision().
        - Phần clearance đã encode trong inflation.
        """

        dist = np.linalg.norm(p1 - p2)
        if dist < 1e-6:
            return not self.is_collision(p1)

        steps = max(3, int(dist / (self.map_res * 0.5)))
        t = np.linspace(0, 1, steps)
        pts = p1 + (p2 - p1)[None, :] * t[:, None]

        for p in pts:
            if self.is_collision(p):
                return False

        return True

    # ============================================
    # IRRT*
    # ============================================
    def sample_informed_region(self, c_best, c_min):
        a = c_best / 2
        b = np.sqrt(abs(c_best**2 - c_min**2)) / 2

        angle = np.arctan2(self.goal[1] - self.start[1],
                           self.goal[0] - self.start[0])
        C = np.array([
            [np.cos(angle), -np.sin(angle)],
            [np.sin(angle), np.cos(angle)]
        ])

        r = np.sqrt(np.random.rand())
        th = np.random.rand() * 2*np.pi
        x = a * r * np.cos(th)
        y = b * r * np.sin(th)

        return (C @ np.array([x, y])) + (self.start + self.goal) / 2

    def irrt_star(self):
        nodes = [{'point': self.start, 'parent': -1, 'cost': 0}]
        c_min = np.linalg.norm(self.goal - self.start)

        best_goal_idx = -1
        best_cost = float("inf")
        last_improve = 0

        for i in range(self.max_iter):

            # sampling
            if best_cost < float("inf") and np.random.rand() < self.goal_sample_rate:
                rnd = self.goal
            else:
                if best_goal_idx == -1:
                    rnd = self.free_points[np.random.randint(len(self.free_points))]
                else:
                    rnd = self.sample_informed_region(best_cost, c_min)

            # nearest
            pts = np.array([n['point'] for n in nodes])
            kdt = cKDTree(pts)
            _, nearest_idx = kdt.query(rnd)
            nearest = nodes[nearest_idx]

            # steer
            direction = rnd - nearest['point']
            dist = np.linalg.norm(direction)
            if dist < 1e-6:
                continue
            direction /= dist
            new = nearest['point'] + direction * min(dist, self.step_len)

            # collision check (Nav2 style)
            if self.is_collision(new) or not self.line_of_sight(nearest['point'], new):
                continue

            # choose parent
            near_idxs = kdt.query_ball_point(new, self.search_radius)
            best_parent = nearest_idx
            best_new_cost = nearest['cost'] + np.linalg.norm(new - nearest['point'])

            for ni in near_idxs:
                n = nodes[ni]
                c = n['cost'] + np.linalg.norm(new - n['point'])
                if c < best_new_cost and self.line_of_sight(n['point'], new):
                    best_parent = ni
                    best_new_cost = c

            new_idx = len(nodes)
            nodes.append({'point': new, 'parent': best_parent, 'cost': best_new_cost})

            # rewire
            for ni in near_idxs:
                n = nodes[ni]
                c = best_new_cost + np.linalg.norm(new - n['point'])
                if c < n['cost'] and self.line_of_sight(new, n['point']):
                    n['cost'] = c
                    n['parent'] = new_idx

            # goal check
            if np.linalg.norm(new - self.goal) < self.search_radius:
                if self.line_of_sight(new, self.goal):
                    total_cost = best_new_cost + np.linalg.norm(new - self.goal)
                    if total_cost < best_cost:
                        best_cost = total_cost
                        best_goal_idx = new_idx
                        last_improve = i

            if best_goal_idx != -1 and (i - last_improve > 200):
                break

        if best_goal_idx == -1:
            self.get_logger().warn("❌ No path found")
            return None

        path = [self.goal]
        idx = best_goal_idx
        while idx != -1:
            path.append(nodes[idx]['point'])
            idx = nodes[idx]['parent']

        return np.array(path[::-1])

    # ============================================
    # SMOOTH ORIENTATION (NEW)
    # ============================================
    def compute_smooth_orientation(self, path):

        N = len(path)

        if N < 4:
            yaws = []
            for i in range(N - 1):
                dp = path[i+1] - path[i]
                yaws.append(math.atan2(dp[1], dp[0]))
            yaws.append(yaws[-1])
            return np.array(yaws)

        raw = []
        for i in range(N - 1):
            dp = path[i+1] - path[i]
            raw.append(math.atan2(dp[1], dp[0]))
        raw.append(raw[-1])
        raw = np.unwrap(raw)

        t = np.linspace(0, 1, N)

        try:
            tck = splprep([raw], s=0.4, k=3)[0]
            yaw_smooth = splev(t, tck)[0]
        except Exception:
            yaw_smooth = raw

        yaw_smooth[0] = self.start_yaw
        yaw_smooth[-1] = self.goal_yaw

        blend = min(max(3, N // 10), N // 2)

        for i in range(blend):
            a = i / blend
            yaw_smooth[i] = (1 - a) * self.start_yaw + a * yaw_smooth[i]

        for i in range(blend):
            a = i / blend
            yaw_smooth[-1 - i] = (1 - a) * self.goal_yaw + a * yaw_smooth[-1 - i]

        return yaw_smooth

    # ============================================
    # PUBLISH PATH
    # ============================================
    def publish_path(self, path_points):

        yaw_list = self.compute_smooth_orientation(path_points)

        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame_id

        for i in range(len(path_points)):
            pose = PoseStamped()
            pose.header = msg.header

            pose.pose.position.x = float(path_points[i][0])
            pose.pose.position.y = float(path_points[i][1])

            yaw = float(yaw_list[i])
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)

            msg.poses.append(pose)

        self.path_pub.publish(msg)
        self.get_logger().info(f"📤 Published smoothed global path ({len(msg.poses)} poses).")

    # ============================================
    # MAIN PLANNER ENTRY
    # ============================================
    def run_planner(self):
        self.get_logger().info("🧠 Running IRRT* Planner...")

        path = self.irrt_star()
        if path is None:
            return

        # Không cần filter thêm — mọi node đã collision-free theo costmap
        self.publish_path(path)


def main():
    rclpy.init()
    node = AgvIrttStarNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
