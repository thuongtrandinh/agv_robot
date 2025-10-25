#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point, PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
from scipy.interpolate import splprep, splev
from scipy.ndimage import binary_dilation

class AgvIrttStarNode(Node):
    def __init__(self):
        super().__init__('agv_global_planner')
        
        # Khai báo tham số ROS 2
        self.declare_parameter('max_iter', 3000)
        self.declare_parameter('step_len', 0.5)
        self.declare_parameter('goal_sample_rate', 0.1)
        self.declare_parameter('search_radius', 2.0)
        self.declare_parameter('path_resolution', 0.1)
        self.declare_parameter('enable_smoothing', True)
        self.declare_parameter('robot_radius', 0.25) # Bán kính robot để inflate

        # Lấy giá trị tham số
        self.max_iter = self.get_parameter('max_iter').get_parameter_value().integer_value
        self.step_len = self.get_parameter('step_len').get_parameter_value().double_value
        self.goal_sample_rate = self.get_parameter('goal_sample_rate').get_parameter_value().double_value
        self.search_radius = self.get_parameter('search_radius').get_parameter_value().double_value
        self.path_resolution = self.get_parameter('path_resolution').get_parameter_value().double_value
        self.enable_smoothing = self.get_parameter('enable_smoothing').get_parameter_value().bool_value
        self.robot_radius = self.get_parameter('robot_radius').get_parameter_value().double_value

        # Tạo Publisher để xuất đường đi
        self.path_pub = self.create_publisher(Path, '/global_path', 10)

        # Subscriptions
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        # Loại bỏ sub 'initialpose', thay vào đó sẽ dùng TF
        # self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.start_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

        self.start = None
        self.goal = None
        self.map_data = None
        self.map_res = None
        self.map_origin = None
        self.map_width = None
        self.map_height = None
        self.map_frame_id = "map" # Mặc định

        # TF Listener để lấy vị trí robot
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info("Global planner node started. Waiting for map and goal.")

    def map_callback(self, msg: OccupancyGrid):
        self.map_res = msg.info.resolution
        self.map_origin = msg.info.origin.position
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_frame_id = msg.header.frame_id # Lấy frame_id từ bản đồ

        data = np.array(msg.data, dtype=np.int8).reshape((self.map_height, self.map_width))
        data = np.flipud(data)  # 🔁 đảo ngược theo trục y để (0,0) là top-left

        raw_map_data = (data > 50)  # True = obstacle

        # (B) Inflate (giãn) vật cản theo bán kính robot
        # Sử dụng toàn bộ bán kính robot để tính toán vùng phồng, đảm bảo an toàn tối đa
        inflate_cells = int(self.robot_radius / self.map_res)
        self.get_logger().info(f"Inflating map by {inflate_cells} cells for a robot radius of {self.robot_radius}m.")
        self.map_data = binary_dilation(raw_map_data, iterations=inflate_cells)

        # (F) Precompute free-space mask để lấy mẫu nhanh
        ys, xs = np.where(~self.map_data) # ~self.map_data for free cells (False)
        # Cập nhật công thức tính Y để khớp với việc đảo trục Y
        self.free_points = np.stack([xs * self.map_res + self.map_origin.x,
                                     (self.map_height - 1 - ys) * self.map_res + self.map_origin.y], axis=1)
        self.get_logger().info(f"Map received: {self.map_width}x{self.map_height} in frame '{self.map_frame_id}'")

    def goal_callback(self, msg: PoseStamped):
        # Khi nhận được goal, lấy vị trí hiện tại của robot làm điểm bắt đầu
        if self.map_data is None:
            self.get_logger().warn("Map not received yet. Cannot plan.")
            return

        try:
            # Lấy transform từ frame của bản đồ tới frame của robot
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                self.map_frame_id,
                'base_footprint', # Frame của robot
                now)
            self.start = np.array([trans.transform.translation.x, trans.transform.translation.y])
            self.get_logger().info(f"Current robot pose set as start: {self.start}")
        except Exception as e:
            self.get_logger().error(f"Could not get robot pose from TF: {e}")
            return

        self.goal = np.array([msg.pose.position.x, msg.pose.position.y])
        self.get_logger().info(f"Goal point set at: {self.goal}")
        # Chạy planner sau khi đã có start và goal
        self.run_planner()

    def world_to_map(self, point):
        mx = int((point[0] - self.map_origin.x) / self.map_res)
        # Sửa world_to_map() cho đúng hướng y sau khi đã flipud map_data
        my = int(self.map_height - 1 - (point[1] - self.map_origin.y) / self.map_res)
        return mx, my

    def is_collision(self, point):
        if self.map_data is None: return True
        mx, my = self.world_to_map(point)
        # (B) Dùng map nhị phân boolean
        if 0 <= mx < self.map_width and 0 <= my < self.map_height: # Kiểm tra biên
            # if self.map_data[my, mx]:
            #     self.get_logger().info(f"⚠️ Collision at world ({point[0]:.2f},{point[1]:.2f}) -> map ({mx},{my})")
            return self.map_data[my, mx] # Trả về giá trị boolean trực tiếp
        return True # Ngoài bản đồ coi là va chạm

    # (C) Tăng tốc kiểm tra line-of-sight (collision) bằng vector hóa NumPy
    def line_of_sight(self, p1, p2):
        dist = np.linalg.norm(p1 - p2)
        # Handle edge case where points are identical or very close, or dist is NaN
        # if dist is NaN, it means p1 or p2 contains NaN, which should be treated as collision
        if np.isnan(dist) or dist < 1e-6:
            return not self.is_collision(p1)

        # (A) Lấy mẫu dày hơn bản đồ (mỗi ½ cell một lần) để không bỏ sót vật cản
        num_steps = max(5, int(dist / (self.map_res * 0.5)))

        t = np.linspace(0, 1, num_steps)
        points = p1 + np.outer(t, (p2 - p1))

        # Convert world coordinates to map coordinates
        mx = ((points[:, 0] - self.map_origin.x) / self.map_res).astype(int)
        # Sửa lại phép tính 'my' để nhất quán với world_to_map và map đã bị lật
        my = (self.map_height - 1 - (points[:, 1] - self.map_origin.y) / self.map_res).astype(int)
        
        # Clip coordinates to map boundaries and check for collision using the boolean map_data
        mx = np.clip(mx, 0, self.map_width - 1)
        my = np.clip(my, 0, self.map_height - 1)
        
        return not np.any(self.map_data[my, mx])

    def sample_informed_region(self, c_best, c_min):
        """Lấy mẫu trong vùng elip được xác định bởi đường đi tốt nhất hiện tại."""
        # Tính toán các tham số của elip
        a = c_best / 2.0
        # Đảm bảo đối số của sqrt không âm để tránh RuntimeWarning
        b = np.sqrt(max(0.0, c_best**2 - c_min**2)) / 2.0
        
        # Ma trận xoay để căn chỉnh elip với start và goal
        angle = np.arctan2(self.goal[1] - self.start[1], self.goal[0] - self.start[0])
        C = np.array([[np.cos(angle), -np.sin(angle)], 
                      [np.sin(angle), np.cos(angle)]])
        
        # Lấy mẫu ngẫu nhiên trong một vòng tròn đơn vị và biến đổi nó
        r = np.random.uniform(0, 1)
        theta = np.random.uniform(0, 2 * np.pi)
        x_unit_circle = np.sqrt(r) * np.cos(theta)
        y_unit_circle = np.sqrt(r) * np.sin(theta)
        
        # Biến đổi điểm mẫu vào elip
        x_ellipse = a * x_unit_circle
        y_ellipse = b * y_unit_circle
        
        # Xoay và dịch chuyển điểm về hệ tọa độ thế giới
        point_rotated = np.dot(C, np.array([x_ellipse, y_ellipse]))
        center = (self.start + self.goal) / 2.0
        return center + point_rotated

    def irrt_star(self):
        # Sử dụng list thay vì dict để truy cập nhanh hơn bằng chỉ số
        nodes = [{'point': self.start, 'parent': -1, 'cost': 0.0}] # node 0 là điểm bắt đầu
        c_min = np.linalg.norm(self.start - self.goal)
        best_goal_node_idx = -1
        
        # (D) Dừng sớm khi tìm được path tốt
        last_improvement_iter = 0 
        current_best_path_cost = float('inf') 

        # (A) Giảm tần suất xây dựng KDTree
        kdtree = None
        node_points = None
        
        for i in range(self.max_iter):
            # (A) Giảm tần suất xây dựng KDTree
            if i % 20 == 0 or i < 10: # chỉ rebuild KDTree sau mỗi 20 iteration hoặc trong 10 iter đầu
                node_points = np.array([n['point'] for n in nodes])
                kdtree = cKDTree(node_points)

            # Lấy mẫu
            rand_point = None
            if best_goal_node_idx == -1 or np.random.rand() > self.goal_sample_rate:
                # Nếu chưa có đường đi, hoặc không lấy mẫu goal -> lấy mẫu ngẫu nhiên
                if best_goal_node_idx == -1: # Giai đoạn 1: RRT*
                    # (F) Precompute free-space mask để lấy mẫu nhanh
                    if hasattr(self, 'free_points') and len(self.free_points) > 0:
                        rand_point = self.free_points[np.random.randint(len(self.free_points))]
                    else: # Fallback nếu free_points chưa có (ví dụ: bản đồ chưa được tải)
                        rand_x = np.random.uniform(self.map_origin.x, self.map_origin.x + self.map_width * self.map_res)
                        rand_y = np.random.uniform(self.map_origin.y, self.map_origin.y + self.map_height * self.map_res)
                        rand_point = np.array([rand_x, rand_y])
                else: # Giai đoạn 2: IRRT*
                    c_best = current_best_path_cost # Sử dụng chi phí tốt nhất hiện tại
                    rand_point = self.sample_informed_region(c_best, c_min)
            else:
                rand_point = self.goal

            _, nearest_idx = kdtree.query(rand_point)
            nearest_node = nodes[nearest_idx]

            # (A) Đảm bảo kdtree đã được xây dựng trước khi query
            if kdtree is None:
                 node_points = np.array([n['point'] for n in nodes])
                 kdtree = cKDTree(node_points)

            direction = rand_point - nearest_node['point']
            dist = np.linalg.norm(direction) # <--- Thêm dòng này để tính toán 'dist'
            # Bỏ qua nếu điểm mẫu quá gần hoặc trùng với điểm đã có
            if dist < 1e-6:
                continue
            direction /= dist
            new_point = nearest_node['point'] + direction * min(self.step_len, dist) # Extend by step_len or to rand_point

            # Thêm kiểm tra va chạm cho điểm mới và đường đi tới nó
            # is_collision kiểm tra điểm cuối, line_of_sight kiểm tra đường đi
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
            
            # Kiểm tra xem có thể kết nối đến goal không và cập nhật đường đi tốt nhất
            dist_to_goal = np.linalg.norm(new_point - self.goal)
            if dist_to_goal < self.search_radius:
                if self.line_of_sight(new_point, self.goal):
                    current_node_path_cost = nodes[new_node_idx]['cost'] + dist_to_goal
                    if current_node_path_cost < current_best_path_cost:
                        if best_goal_node_idx == -1: self.get_logger().info("Initial path found! Optimizing...")
                        best_goal_node_idx = new_node_idx
                        current_best_path_cost = current_node_path_cost # Cập nhật chi phí tốt nhất
                        last_improvement_iter = i # (D) Cập nhật lần cải thiện cuối cùng

            # (D) Dừng sớm khi tìm được path tốt
            if best_goal_node_idx != -1 and i > 500: # Bắt đầu kiểm tra sau 500 vòng lặp
                if (i - last_improvement_iter) > 200: # Nếu không có cải thiện trong 200 vòng lặp
                    self.get_logger().info(f"Stop early at iter {i} (no better path found for {i - last_improvement_iter} iterations).")
                    break

        if best_goal_node_idx != -1:
            self.get_logger().info("Finished planning. Found an optimized path.")
            path = [self.goal]
            curr_idx = best_goal_node_idx
            while curr_idx != -1:
                path.append(nodes[curr_idx]['point'])
                curr_idx = nodes[curr_idx]['parent']
            return np.array(path[::-1])
        else:
            self.get_logger().warn("IRRT* failed to find a path.")
            return None

    def prune_path(self, path):
        """Cắt tỉa các điểm không cần thiết trên đường đi."""
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

    def smooth_path_bspline(self, path):
        if len(path) < 4: return path # Cần ít nhất 4 điểm để tạo spline
        x = path[:, 0]
        y = path[:, 1]
        # Giảm độ căng spline thêm nữa (s=0.1) để đường đi bám sát hơn vào đường gốc
        tck, u = splprep([x, y], s=0.1, k=3)
        # Giảm số điểm nội suy để giảm khả năng va chạm
        u_new = np.linspace(u.min(), u.max(), len(path) * 3)
        x_new, y_new = splev(u_new, tck)
        
        smoothed_path = np.c_[x_new, y_new]

        # Rất quan trọng: Kiểm tra va chạm cho đường đi đã được làm mượt
        # Vì quá trình làm mượt có thể đưa đường đi vào vật cản, ngay cả khi đường ban đầu không va chạm.
        for point in smoothed_path:
            if self.is_collision(point):
                self.get_logger().warn("Smoothed path is in collision! Falling back to the pruned (unsmoothed) path.")
                return None # Trả về None để báo hiệu đường đi không hợp lệ

        return smoothed_path

    def publish_path(self, path_points):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.map_frame_id # Sử dụng frame của bản đồ

        for i in range(len(path_points)):
            point = path_points[i]
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]

            # Tính toán hướng (orientation) cho robot
            if i < len(path_points) - 1:
                next_point = path_points[i+1]
                angle = np.arctan2(next_point[1] - point[1], next_point[0] - point[0])
                pose.pose.orientation.z = np.sin(angle / 2.0)
                pose.pose.orientation.w = np.cos(angle / 2.0)
            else: # Giữ hướng của điểm cuối cùng
                pose.pose.orientation = path_msg.poses[-1].pose.orientation

            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.get_logger().info(f"Published path with {len(path_points)} points.")

    def run_planner(self):
        if self.start is None or self.goal is None:
            self.get_logger().warn("Start or goal not set. Cannot run planner.")
            return

        self.get_logger().info(f"Planning path from {self.start} to {self.goal}")
        path = self.irrt_star()

        if path is None:
            self.get_logger().error("Could not find a path.")
            return

        # Debug trực quan: Kiểm tra xem có điểm nào trên đường đi cuối cùng bị va chạm không
        for p in path:
            if self.is_collision(p):
                self.get_logger().warn(f"⚠️ Final path point is in collision at ({p[0]:.2f}, {p[1]:.2f})")
                # Có thể thêm logic để dừng hoặc xử lý lỗi ở đây

        self.get_logger().info(f"Pruning path with {len(path)} points...")
        pruned_path = self.prune_path(path)
        self.get_logger().info(f"Path pruned to {len(pruned_path)} points.")

        path_to_publish = pruned_path
        path_to_plot = pruned_path

        if self.enable_smoothing:
            self.get_logger().info("Smoothing path...")
            # Làm mượt đường đi đã được cắt tỉa
            smoothed_path = self.smooth_path_bspline(pruned_path)
            
            # Chỉ sử dụng đường đi đã làm mượt nếu nó hợp lệ (không va chạm)
            if smoothed_path is not None:
                self.get_logger().info("Successfully generated a collision-free smoothed path.")
                path_to_publish = smoothed_path
                path_to_plot = smoothed_path
        
        self.publish_path(path_to_publish)

        # Vẽ kết quả
        plt.figure()
        if self.map_data is not None:
            ox, oy = [], []
            for i in range(self.map_width):
                for j in range(self.map_height):
                    if self.map_data[j, i]: # Kiểm tra nếu ô là chướng ngại vật (True)
                        # Cập nhật công thức tính Y để khớp với việc đảo trục Y
                        ox.append(i * self.map_res + self.map_origin.x)
                        oy.append(self.map_origin.y + (self.map_height - 1 - j) * self.map_res)
            plt.plot(ox, oy, ".k", markersize=2, label='Obstacles')

        plt.plot(pruned_path[:, 0], pruned_path[:, 1], 'c--', label='Pruned Path')
        # Chỉ vẽ đường làm mượt nếu nó khác với đường đã tỉa (tức là việc làm mượt thành công)
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
