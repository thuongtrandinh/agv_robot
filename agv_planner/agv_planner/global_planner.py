#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Point, PoseWithCovarianceStamped, Quaternion
from tf2_ros import Buffer, TransformListener
import numpy as np
import math
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
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)    


        # Biến lưu trữ vị trí robot
        self.robot_position = None
        self.robot_orientation = None

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

    def odom_callback(self, msg: Odometry):
        # Lấy vị trí và orientation từ Odometry message
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        # Chuyển đổi quaternions thành góc Euler nếu cần
        euler = self.quaternion_to_euler(orientation)

        # Cập nhật vị trí và orientation của robot
        self.robot_position = np.array([position.x, position.y])
        self.robot_orientation = euler[2]  # Chỉ lấy yaw (z)

        self.get_logger().info(f"Robot position: {self.robot_position}, Orientation: {self.robot_orientation}")

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

        return roll_x, pitch_y, yaw_z  # Return (roll, pitch, yaw)

    def map_callback(self, msg: OccupancyGrid):
        self.map_res = msg.info.resolution
        self.map_origin = msg.info.origin.position
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_frame_id = msg.header.frame_id

        data = np.array(msg.data, dtype=np.int8).reshape((self.map_height, self.map_width))
        data = np.flipud(data)

        raw_map_data = (data > 50)  # True = obstacle

        # Inflate obstacles based on robot radius
        inflate_cells = int(self.robot_radius / self.map_res)
        self.get_logger().info(f"Inflating map by {inflate_cells} cells for a robot radius of {self.robot_radius}m.")
        self.map_data = binary_dilation(raw_map_data, iterations=inflate_cells)

        # Precompute free-space mask
        ys, xs = np.where(~self.map_data)
        self.free_points = np.stack([xs * self.map_res + self.map_origin.x,
                                    (self.map_height - 1 - ys) * self.map_res + self.map_origin.y], axis=1)
        self.get_logger().info(f"Map received: {self.map_width}x{self.map_height} in frame '{self.map_frame_id}'")

        if self.map_data is not None:
            self.get_logger().info("Map is successfully received and processed.")

    def goal_callback(self, msg: PoseStamped):
        if self.map_data is None:
            self.get_logger().warn("Map not received yet. Cannot plan.")
            return

        self.goal = np.array([msg.pose.position.x, msg.pose.position.y])
        self.get_logger().info(f"Goal point set at: {self.goal}")
        
        if self.start is not None:
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

    def densify_path(self, path, resolution):
        """
        Chèn thêm các điểm vào đường đi để đảm bảo có đủ điểm cho việc làm mượt.
        Hữu ích khi đường đi chỉ có vài điểm sau khi cắt tỉa (ví dụ: đường thẳng).
        """
        if path is None or len(path) < 2:
            return path

        densified_path = []
        for i in range(len(path) - 1):
            p1 = path[i]
            p2 = path[i+1]
            densified_path.append(p1)
            dist = np.linalg.norm(p2 - p1)
            num_points = int(dist / resolution)
            if num_points > 0:
                for j in range(1, num_points):
                    densified_path.append(p1 + (p2 - p1) * (j / num_points))
        densified_path.append(path[-1])
        return np.array(densified_path)

    def smooth_path_bspline(self, path):
        if len(path) < 4: return path # Cần ít nhất 4 điểm để tạo spline
        x = path[:, 0]
        y = path[:, 1]
        # Để tăng khả năng làm mượt mà không bị va chạm trong các khu vực hẹp,
        # cần đảm bảo spline bám sát đường gốc hơn.
        # Giá trị 's' càng nhỏ, spline càng bám sát các điểm dữ liệu gốc.
        tck, u = splprep([x, y], s=0.0, k=3) # Thử s=0.0 để spline nội suy qua tất cả các điểm, hoặc một giá trị rất nhỏ như 0.01
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

    def relaxation_smoothing(self, path, alpha=0.45, beta=0.2, iterations=40):
        """
        Làm mượt đường đi bằng phương pháp "relaxation" có kiểm tra va chạm chặt chẽ.
        Để làm đường đi mượt hơn và ít bám vào đường gốc hơn:
        - Tăng alpha (lực làm mượt)
        - Giảm beta (lực bám đường gốc)
        Làm mượt đường đi bằng phương pháp "relaxation" có kiểm tra va chạm chặt chẽ.
        - alpha: hệ số kéo về trung điểm (điều chỉnh độ cong)
        - beta: hệ số giữ vị trí ban đầu (tránh lệch xa path gốc)
        - iterations: số lần lặp
        """
        if path is None or len(path) < 3:
            return path

        smoothed = path.copy()

        for _ in range(iterations):
            updated = smoothed.copy()
            for i in range(1, len(smoothed) - 1):
                # Dịch chuyển về trung bình 2 điểm lân cận
                correction = alpha * (smoothed[i - 1] + smoothed[i + 1] - 2 * smoothed[i])
                correction += beta * (path[i] - smoothed[i])

                candidate = smoothed[i] + correction

                # Kiểm tra va chạm điểm
                if self.is_collision(candidate):
                    continue  # bỏ qua nếu va chạm

                # Kiểm tra va chạm với hai đoạn lân cận
                if not self.line_of_sight(smoothed[i - 1], candidate) or \
                   not self.line_of_sight(candidate, smoothed[i + 1]):
                    continue

                # Cập nhật điểm mới nếu an toàn
                updated[i] = candidate

            smoothed = updated

        # Kiểm tra toàn tuyến, nếu vẫn va chạm thì không sử dụng đường đi đã làm mượt
        for i in range(len(smoothed) - 1):
            if not self.line_of_sight(smoothed[i], smoothed[i + 1]):
                self.get_logger().warn("⚠️ Smoothed path segment still collides! Falling back to the pruned path.")
                return None

        return smoothed

    def fine_smooth_bspline(self, path, s=0.1):
        """
        Làm mượt đường đi bằng B-spline sau khi đã có đường đi an toàn từ relaxation.
        Kiểm tra va chạm và trả về đường gốc nếu spline bị va chạm.
        """
        if path is None or len(path) < 4:
            return path # Cần ít nhất 4 điểm để tạo spline
        x = path[:, 0]
        y = path[:, 1]
        tck, u = splprep([x, y], s=s, k=3)
        u_new = np.linspace(u.min(), u.max(), int(len(path) * 1.5)) # Giảm số điểm nội suy để tăng tốc
        x_new, y_new = splev(u_new, tck)
        
        smoothed_bspline_path = np.c_[x_new, y_new]

        # Rất quan trọng: Kiểm tra va chạm cho đường đi đã được làm mượt
        for p in smoothed_bspline_path:
            if self.is_collision(p):
                self.get_logger().warn("⚠️ Fine-smooth spline collided! Keeping coarse path.")
                return path # Trả về đường đi gốc (từ relaxation) nếu spline bị va chạm
        return smoothed_bspline_path

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

        # Tối ưu hóa: Nếu đường đi đã là đường thẳng trong không gian trống (chỉ có 2 điểm),
        # bỏ qua quá trình làm mượt tốn kém.
        if len(pruned_path) == 2 and self.enable_smoothing:
            self.get_logger().info("Path is an optimal straight line. Skipping smoothing.")
            path_to_publish = pruned_path
            path_to_plot = pruned_path
        elif self.enable_smoothing:
            # (Giải pháp) Chèn thêm điểm vào đường đi đã cắt tỉa để có đủ điểm làm mượt
            densified_pruned_path = self.densify_path(pruned_path, self.step_len)
            self.get_logger().info(f"Path densified from {len(pruned_path)} to {len(densified_pruned_path)} points for smoothing.")

            self.get_logger().info("Stage 1: Safe relaxation smoothing...")
            # Chỉ sử dụng đường đi đã được làm dày (densified) để làm mượt
            smoothed_path = self.relaxation_smoothing(densified_pruned_path)
            
            # Chỉ sử dụng đường đi đã làm mượt nếu nó hợp lệ (không va chạm)
            if smoothed_path is not None:
                self.get_logger().info("Stage 2: Fine B-spline smoothing...")
                smoothed_path = self.fine_smooth_bspline(smoothed_path, s=0.1)
                self.get_logger().info("Successfully generated a collision-free dual-stage smoothed path.")
                path_to_publish = smoothed_path
                path_to_plot = smoothed_path
            else: # Nếu làm mượt thất bại, dùng đường đã tỉa
                path_to_publish = pruned_path
                path_to_plot = pruned_path
        else: # Nếu không bật smoothing
            path_to_publish = pruned_path
            path_to_plot = pruned_path
        
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