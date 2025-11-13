#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path, OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Point, PoseWithCovarianceStamped, Quaternion

from tf2_ros import Buffer, TransformListener

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
from scipy.interpolate import splprep, splev
from scipy.ndimage import binary_dilation


class AgvIrttStarNode(Node):
    def __init__(self):
        super().__init__('agv_global_planner')
        
        # =======================
        # Tham số ROS 2
        # =======================
        self.declare_parameter('max_iter', 3000)
        self.declare_parameter('step_len', 0.5)
        self.declare_parameter('goal_sample_rate', 0.1)
        self.declare_parameter('search_radius', 2.0)
        self.declare_parameter('path_resolution', 0.1)
        self.declare_parameter('enable_smoothing', True)
        self.declare_parameter('robot_radius', 0.25)  # Bán kính robot để inflate

        # Lấy giá trị tham số
        self.max_iter = self.get_parameter('max_iter').get_parameter_value().integer_value
        self.step_len = self.get_parameter('step_len').get_parameter_value().double_value
        self.goal_sample_rate = self.get_parameter('goal_sample_rate').get_parameter_value().double_value
        self.search_radius = self.get_parameter('search_radius').get_parameter_value().double_value
        self.path_resolution = self.get_parameter('path_resolution').get_parameter_value().double_value
        self.enable_smoothing = self.get_parameter('enable_smoothing').get_parameter_value().bool_value
        self.robot_radius = self.get_parameter('robot_radius').get_parameter_value().double_value

        # =======================
        # Publisher
        # =======================
        self.path_pub = self.create_publisher(Path, '/global_path', 10)

        # =======================
        # QoS CHUẨN CHO /map (Nav2 map_server)
        # =======================
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        # =======================
        # Subscribers
        # =======================
        # /map dùng QoS TRANSIENT_LOCAL nên phải match
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos
        )

        # goal & odom để mặc định là được
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)

        # Biến lưu trữ
        self.robot_position = None
        self.robot_orientation = None

        self.start = None
        self.goal = None
        self.map_data = None
        self.map_res = None
        self.map_origin = None
        self.map_width = None
        self.map_height = None
        self.map_frame_id = "map"  # default

        # TF Buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info("✅ Global planner node started. Waiting for /map and /goal_pose ...")

    # =======================
    # MAP CALLBACK
    # =======================
    def map_callback(self, msg: OccupancyGrid):
        self.get_logger().info(f"✅ Map received: {msg.info.width}x{msg.info.height} in frame '{msg.header.frame_id}'")
        self.map_res = msg.info.resolution
        self.map_origin = msg.info.origin.position
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_frame_id = msg.header.frame_id

        if not msg.data:
            self.get_logger().warn("⚠️ Received empty map data!")
            return

        data = np.array(msg.data, dtype=np.int8).reshape((self.map_height, self.map_width))
        data = np.flipud(data)  # đảo Y cho đúng

        raw_map_data = (data > 50)  # True = obstacle

        # Inflate theo bán kính robot
        inflate_cells = int(self.robot_radius / self.map_res)
        self.get_logger().info(
            f"Inflating map by {inflate_cells} cells for robot_radius={self.robot_radius} m."
        )
        self.map_data = binary_dilation(raw_map_data, iterations=inflate_cells)

        # Precompute free points để sample nhanh
        ys, xs = np.where(~self.map_data)  # free cells
        self.free_points = np.stack(
            [
                xs * self.map_res + self.map_origin.x,
                (self.map_height - 1 - ys) * self.map_res + self.map_origin.y,
            ],
            axis=1,
        )

    # =======================
    # ODOM CALLBACK
    # =======================
    def odom_callback(self, msg: Odometry):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        euler = self.quaternion_to_euler(orientation)

        self.robot_position = np.array([position.x, position.y])
        self.robot_orientation = euler[2]  # yaw

        # Có thể comment log này nếu spam
        # self.get_logger().info(f"Robot position: {self.robot_position}, Orientation: {self.robot_orientation}")

    def quaternion_to_euler(self, quaternion: Quaternion):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z

    # =======================
    # GOAL CALLBACK
    # =======================
    def goal_callback(self, msg: PoseStamped):
        self.get_logger().info(f"🎯 Goal received: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})")

        if self.map_data is None:
            self.get_logger().warn("❌ Map not received yet. Cannot plan.")
            return

        # Lấy pose robot từ TF: map -> base_footprint
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                self.map_frame_id,
                'base_footprint',
                now
            )
            self.start = np.array(
                [trans.transform.translation.x, trans.transform.translation.y]
            )
            self.get_logger().info(f"🚗 Start (current robot pose): {self.start}")
        except Exception as e:
            self.get_logger().error(f"❌ Could not get robot pose from TF: {e}")
            return

        self.goal = np.array([msg.pose.position.x, msg.pose.position.y])
        self.get_logger().info(f"🎯 Goal point set at: {self.goal}")
        self.run_planner()

    # =======================
    # MAP / COLLISION UTILS
    # =======================
    def world_to_map(self, point):
        mx = int((point[0] - self.map_origin.x) / self.map_res)
        my = int(self.map_height - 1 - (point[1] - self.map_origin.y) / self.map_res)
        return mx, my

    def is_collision(self, point):
        if self.map_data is None:
            return True
        mx, my = self.world_to_map(point)
        if 0 <= mx < self.map_width and 0 <= my < self.map_height:
            return self.map_data[my, mx]
        return True

    def line_of_sight(self, p1, p2):
        dist = np.linalg.norm(p1 - p2)
        if np.isnan(dist) or dist < 1e-6:
            return not self.is_collision(p1)

        num_steps = max(5, int(dist / (self.map_res * 0.5)))
        t = np.linspace(0, 1, num_steps)
        points = p1 + np.outer(t, (p2 - p1))

        mx = ((points[:, 0] - self.map_origin.x) / self.map_res).astype(int)
        my = (self.map_height - 1 - (points[:, 1] - self.map_origin.y) / self.map_res).astype(int)

        mx = np.clip(mx, 0, self.map_width - 1)
        my = np.clip(my, 0, self.map_height - 1)

        return not np.any(self.map_data[my, mx])

    # =======================
    # IRRT* CORE
    # =======================
    def sample_informed_region(self, c_best, c_min):
        a = c_best / 2.0
        b = np.sqrt(max(0.0, c_best**2 - c_min**2)) / 2.0

        angle = np.arctan2(self.goal[1] - self.start[1],
                           self.goal[0] - self.start[0])
        C = np.array([[np.cos(angle), -np.sin(angle)],
                      [np.sin(angle), np.cos(angle)]])

        r = np.random.uniform(0, 1)
        theta = np.random.uniform(0, 2 * np.pi)
        x_unit_circle = np.sqrt(r) * np.cos(theta)
        y_unit_circle = np.sqrt(r) * np.sin(theta)

        x_ellipse = a * x_unit_circle
        y_ellipse = b * y_unit_circle

        point_rotated = np.dot(C, np.array([x_ellipse, y_ellipse]))
        center = (self.start + self.goal) / 2.0
        return center + point_rotated

    def irrt_star(self):
        nodes = [{'point': self.start, 'parent': -1, 'cost': 0.0}]
        c_min = np.linalg.norm(self.start - self.goal)
        best_goal_node_idx = -1

        last_improvement_iter = 0
        current_best_path_cost = float('inf')

        kdtree = None
        node_points = None

        for i in range(self.max_iter):
            if i % 20 == 0 or i < 10:
                node_points = np.array([n['point'] for n in nodes])
                kdtree = cKDTree(node_points)

            if best_goal_node_idx == -1 or np.random.rand() > self.goal_sample_rate:
                if best_goal_node_idx == -1:
                    if hasattr(self, 'free_points') and len(self.free_points) > 0:
                        rand_point = self.free_points[np.random.randint(len(self.free_points))]
                    else:
                        rand_x = np.random.uniform(
                            self.map_origin.x,
                            self.map_origin.x + self.map_width * self.map_res
                        )
                        rand_y = np.random.uniform(
                            self.map_origin.y,
                            self.map_origin.y + self.map_height * self.map_res
                        )
                        rand_point = np.array([rand_x, rand_y])
                else:
                    c_best = current_best_path_cost
                    rand_point = self.sample_informed_region(c_best, c_min)
            else:
                rand_point = self.goal

            if kdtree is None:
                node_points = np.array([n['point'] for n in nodes])
                kdtree = cKDTree(node_points)

            _, nearest_idx = kdtree.query(rand_point)
            nearest_node = nodes[nearest_idx]

            direction = rand_point - nearest_node['point']
            dist = np.linalg.norm(direction)
            if dist < 1e-6:
                continue
            direction /= dist
            new_point = nearest_node['point'] + direction * min(self.step_len, dist)

            if self.is_collision(new_point) or not self.line_of_sight(nearest_node['point'], new_point):
                continue

            near_indices = kdtree.query_ball_point(new_point, self.search_radius)
            min_cost = nearest_node['cost'] + np.linalg.norm(new_point - nearest_node['point'])
            best_parent_idx = nearest_idx

            for near_idx in near_indices:
                near_node = nodes[near_idx]
                cost = near_node['cost'] + np.linalg.norm(new_point - near_node['point'])
                if cost < min_cost and self.line_of_sight(near_node['point'], new_point):
                    min_cost = cost
                    best_parent_idx = near_idx

            new_node_idx = len(nodes)
            nodes.append({'point': new_point, 'parent': best_parent_idx, 'cost': min_cost})

            for near_idx in near_indices:
                if near_idx != best_parent_idx:
                    near_node = nodes[near_idx]
                    new_cost = min_cost + np.linalg.norm(new_point - near_node['point'])
                    if new_cost < near_node['cost'] and self.line_of_sight(new_point, near_node['point']):
                        nodes[near_idx]['parent'] = new_node_idx
                        nodes[near_idx]['cost'] = new_cost

            dist_to_goal = np.linalg.norm(new_point - self.goal)
            if dist_to_goal < self.search_radius:
                if self.line_of_sight(new_point, self.goal):
                    current_node_path_cost = nodes[new_node_idx]['cost'] + dist_to_goal
                    if current_node_path_cost < current_best_path_cost:
                        if best_goal_node_idx == -1:
                            self.get_logger().info("✅ Initial path found! Optimizing...")
                        best_goal_node_idx = new_node_idx
                        current_best_path_cost = current_node_path_cost
                        last_improvement_iter = i

            if best_goal_node_idx != -1 and i > 500:
                if (i - last_improvement_iter) > 200:
                    self.get_logger().info(
                        f"⏹ Stop early at iter {i} (no better path for {i - last_improvement_iter} iterations)."
                    )
                    break

        if best_goal_node_idx != -1:
            self.get_logger().info("🎉 Finished planning. Found an optimized path.")
            path = [self.goal]
            curr_idx = best_goal_node_idx
            while curr_idx != -1:
                path.append(nodes[curr_idx]['point'])
                curr_idx = nodes[curr_idx]['parent']
            return np.array(path[::-1])
        else:
            self.get_logger().warn("❌ IRRT* failed to find a path.")
            return None

    # =======================
    # PRUNE / SMOOTH / PUBLISH
    # =======================
    def prune_path(self, path):
        if path is None or len(path) < 3:
            return path

        pruned_path = [path[0]]
        current_idx = 0
        while current_idx < len(path) - 1:
            for next_idx in range(len(path) - 1, current_idx, -1):
                if self.line_of_sight(path[current_idx], path[next_idx]):
                    pruned_path.append(path[next_idx])
                    current_idx = next_idx
                    break
        return np.array(pruned_path)

    def densify_path(self, path, resolution):
        if path is None or len(path) < 2:
            return path

        densified_path = []
        for i in range(len(path) - 1):
            p1 = path[i]
            p2 = path[i + 1]
            densified_path.append(p1)
            dist = np.linalg.norm(p2 - p1)
            num_points = int(dist / resolution)
            if num_points > 0:
                for j in range(1, num_points):
                    densified_path.append(p1 + (p2 - p1) * (j / num_points))
        densified_path.append(path[-1])
        return np.array(densified_path)

    def relaxation_smoothing(self, path, alpha=0.45, beta=0.2, iterations=40):
        if path is None or len(path) < 3:
            return path

        smoothed = path.copy()

        for _ in range(iterations):
            updated = smoothed.copy()
            for i in range(1, len(smoothed) - 1):
                correction = alpha * (smoothed[i - 1] + smoothed[i + 1] - 2 * smoothed[i])
                correction += beta * (path[i] - smoothed[i])

                candidate = smoothed[i] + correction

                if self.is_collision(candidate):
                    continue
                if not self.line_of_sight(smoothed[i - 1], candidate) or \
                   not self.line_of_sight(candidate, smoothed[i + 1]):
                    continue

                updated[i] = candidate

            smoothed = updated

        for i in range(len(smoothed) - 1):
            if not self.line_of_sight(smoothed[i], smoothed[i + 1]):
                self.get_logger().warn(
                    "⚠️ Smoothed path segment still collides! Falling back to the pruned path."
                )
                return None

        return smoothed

    def fine_smooth_bspline(self, path, s=0.1):
        if path is None or len(path) < 4:
            return path
        x = path[:, 0]
        y = path[:, 1]
        tck, u = splprep([x, y], s=s, k=3)
        u_new = np.linspace(u.min(), u.max(), int(len(path) * 1.5))
        x_new, y_new = splev(u_new, tck)

        smoothed_bspline_path = np.c_[x_new, y_new]

        for p in smoothed_bspline_path:
            if self.is_collision(p):
                self.get_logger().warn("⚠️ Fine-smooth spline collided! Keeping coarse path.")
                return path
        return smoothed_bspline_path

    def publish_path(self, path_points):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.map_frame_id

        for i in range(len(path_points)):
            point = path_points[i]
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]

            if i < len(path_points) - 1:
                next_point = path_points[i + 1]
                angle = np.arctan2(next_point[1] - point[1],
                                   next_point[0] - point[0])
                pose.pose.orientation.z = np.sin(angle / 2.0)
                pose.pose.orientation.w = np.cos(angle / 2.0)
            else:
                pose.pose.orientation = path_msg.poses[-1].pose.orientation

            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.get_logger().info(f"📤 Published path with {len(path_points)} points.")

    def run_planner(self):
        if self.start is None or self.goal is None:
            self.get_logger().warn("Start or goal not set. Cannot run planner.")
            return

        self.get_logger().info(f"🧠 Planning path from {self.start} to {self.goal}")
        path = self.irrt_star()

        if path is None:
            self.get_logger().error("Could not find a path.")
            return

        for p in path:
            if self.is_collision(p):
                self.get_logger().warn(
                    f"⚠️ Final path point is in collision at ({p[0]:.2f}, {p[1]:.2f})"
                )

        self.get_logger().info(f"Pruning path with {len(path)} points...")
        pruned_path = self.prune_path(path)
        self.get_logger().info(f"Path pruned to {len(pruned_path)} points.")

        if len(pruned_path) == 2 and self.enable_smoothing:
            self.get_logger().info("Path is straight. Skipping smoothing.")
            path_to_publish = pruned_path
            path_to_plot = pruned_path
        elif self.enable_smoothing:
            densified_pruned_path = self.densify_path(pruned_path, self.step_len)
            self.get_logger().info(
                f"Path densified from {len(pruned_path)} to {len(densified_pruned_path)} points."
            )

            self.get_logger().info("Stage 1: Relaxation smoothing...")
            smoothed_path = self.relaxation_smoothing(densified_pruned_path)

            if smoothed_path is not None:
                self.get_logger().info("Stage 2: Fine B-spline smoothing...")
                smoothed_path = self.fine_smooth_bspline(smoothed_path, s=0.1)
                self.get_logger().info("✅ Dual-stage smoothing complete.")
                path_to_publish = smoothed_path
                path_to_plot = smoothed_path
            else:
                path_to_publish = pruned_path
                path_to_plot = pruned_path
        else:
            path_to_publish = pruned_path
            path_to_plot = pruned_path

        self.publish_path(path_to_publish)

        # Plot debug (có thể tắt nếu không cần)
        plt.figure()
        if self.map_data is not None:
            ox, oy = [], []
            for i in range(self.map_width):
                for j in range(self.map_height):
                    if self.map_data[j, i]:
                        ox.append(i * self.map_res + self.map_origin.x)
                        oy.append(self.map_origin.y + (self.map_height - 1 - j) * self.map_res)
            plt.plot(ox, oy, ".k", markersize=2, label='Obstacles')

        plt.plot(pruned_path[:, 0], pruned_path[:, 1], 'c--', label='Pruned Path')
        if path_to_plot is not pruned_path:
            plt.plot(path_to_plot[:, 0], path_to_plot[:, 1], 'b-', label='Smoothed Path')

        plt.plot(self.start[0], self.start[1], 'go', label='Start')
        plt.plot(self.goal[0], self.goal[1], 'ro', label='Goal')
        plt.title('IRRT* Path Planning')
        plt.legend()
        plt.axis("equal")
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = AgvIrttStarNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()