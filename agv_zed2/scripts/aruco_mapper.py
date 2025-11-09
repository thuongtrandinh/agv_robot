#!/usr/bin/env python3
"""
ROS2 ArUco Mapper
Package: agv_zed2

Chức năng:
✅ Mở camera ZED2.
✅ Dò tìm các ArUco marker.
✅ Tính toán và lưu trữ pose của các marker so với camera.
✅ Khi tắt node (Ctrl+C), tự động lưu bản đồ các marker vào file YAML.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import time
import subprocess
import yaml
from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation
import os

# --- Cấu hình PyYAML để xử lý các kiểu dữ liệu của Numpy ---
# Điều này đảm bảo file YAML được lưu với định dạng số chuẩn.
def numpy_float_representer(dumper, data):
    return dumper.represent_scalar(u'tag:yaml.org,2002:float', str(data))
yaml.add_representer(np.float64, numpy_float_representer)
yaml.add_representer(np.float32, numpy_float_representer)

def load_zed_yaml(package_name: str, yaml_name: str, side: str = 'left'):
    pkg_share_path = get_package_share_directory(package_name)
    yaml_path = os.path.join(pkg_share_path, 'config', yaml_name)
    if not os.path.exists(yaml_path):
        raise FileNotFoundError(f"Không tìm thấy file calib: {yaml_path}")

    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    key = 'left_camera' if side.lower().startswith('l') else 'right_camera'
    cam = data[key]
    K = np.array(cam['camera_matrix']['data'], dtype=np.float64).reshape(3, 3)
    D = np.array(cam['distortion_coefficients']['data'], dtype=np.float64)
    return K, D

def find_zed_camera_id():
    try:
        result = subprocess.run(['v4l2-ctl', '--list-devices'], capture_output=True, text=True, check=True)
        lines = result.stdout.splitlines()
        for i, line in enumerate(lines):
            if 'ZED' in line:
                if i + 1 < len(lines):
                    dev_path = lines[i+1].strip()
                    if dev_path.startswith('/dev/video'):
                        return int(dev_path.replace('/dev/video', ''))
    except Exception as e:
        print(f"Lỗi khi tìm ZED camera: {e}")
        return -1
    return -1

class ArucoMapperNode(Node):
    def __init__(self):
        super().__init__('aruco_mapper')

        # --- Parameters ---
        self.declare_parameter('device_id', -1)
        self.declare_parameter('calib_pkg', 'agv_zed2')
        self.declare_parameter('calib_file', 'zed2_calibration_hd.yaml')
        self.declare_parameter('marker_size', 0.173)
        self.declare_parameter('aruco_dict', 'DICT_6X6_250')
        self.declare_parameter('map_file', 'aruco_map.yaml')

        device_id_param = self.get_parameter('device_id').value
        calib_pkg = self.get_parameter('calib_pkg').value
        calib_file = self.get_parameter('calib_file').value
        self.marker_length = self.get_parameter('marker_size').value
        dict_name = self.get_parameter('aruco_dict').value
        map_file = self.get_parameter('map_file').value

        # --- Map Storage ---
        self.map_save_path = os.path.join(get_package_share_directory('agv_zed2'), 'map', map_file)
        self.detected_markers = {} # {id: Pose}

        # --- State for SLAM-like mapping ---
        # T_map_cam: Transform từ map -> camera. Ban đầu là ma trận đơn vị.
        self.T_map_cam = np.eye(4) 
        self.map_initialized = False
        self.last_info_data = None

        # --- Load camera calibration ---
        self.K_orig, self.D = load_zed_yaml(calib_pkg, calib_file, 'left')
        self.D_eff = np.zeros_like(self.D)

        # --- Auto-find camera ---
        device_id = device_id_param
        if device_id == -1:
            self.get_logger().info("Đang tự động tìm ZED camera...")
            device_id = find_zed_camera_id()
            if device_id != -1:
                self.get_logger().info(f"✅ Tìm thấy ZED camera tại /dev/video{device_id}")

        # --- Open camera ---
        self.cap = cv2.VideoCapture(device_id, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().fatal(f"❌ Không mở được /dev/video{device_id}.")
            raise RuntimeError(f"Camera open failed on device {device_id}")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.cap.set(cv2.CAP_PROP_FPS, 60)

        # --- ArUco setup ---
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        # --- ROS setup ---
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.bridge = CvBridge()
        # self.pub_img = self.create_publisher(Image, '/aruco_mapper/image', qos) # Bỏ publisher ảnh
        self.pub_info = self.create_publisher(Float32MultiArray, '/aruco_mapper/info', 10)

        # --- Multi-threading setup ---
        self.frame_buffer = None
        self.frame_lock = threading.Lock()
        self.new_frame_event = threading.Event()
        self.running = True

        self.capture_thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.capture_thread.start()

        self.process_thread = threading.Thread(target=self.process_loop, daemon=True)
        self.process_thread.start()

        self.get_logger().info(f"✅ Aruco Mapper đang chạy. Hướng camera vào các marker.")
        self.get_logger().info(f"Bản đồ sẽ được lưu vào: {self.map_save_path}")

    def capture_loop(self):
        while rclpy.ok() and self.running:
            ret, frame = self.cap.read()
            if ret:
                with self.frame_lock:
                    self.frame_buffer = frame
                self.new_frame_event.set()
            else:
                time.sleep(0.01)

    def process_loop(self):
        while rclpy.ok() and self.running:
            if self.new_frame_event.wait(timeout=0.05):
                frame_to_process = None
                with self.frame_lock:
                    frame_to_process = self.frame_buffer
                    self.new_frame_event.clear()
                if frame_to_process is not None:
                    self.process_frame(frame_to_process)

    def process_frame(self, frame):
        orig_h, orig_w = frame.shape[:2]
        left_hd = frame[:, :orig_w // 2]

        target_w, target_h = 672, 376
        left_vga = cv2.resize(left_hd, (target_w, target_h), interpolation=cv2.INTER_AREA)

        scale_x = target_w / (orig_w / 2)
        scale_y = target_h / orig_h
        K_scaled = self.K_orig.copy()
        K_scaled[0, 0] *= scale_x
        K_scaled[1, 1] *= scale_y
        K_scaled[0, 2] *= scale_x
        K_scaled[1, 2] *= scale_y

        self.detect_and_update_map(left_vga, K_scaled)

        # Bỏ phần publish ảnh
        # if self.pub_img.get_subscription_count() > 0:
        #     img_msg = self.bridge.cv2_to_imgmsg(left_vga, encoding='bgr8')
        #     self.pub_img.publish(img_msg)

    def detect_and_update_map(self, cv_img, K):
        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        info_msg = Float32MultiArray()
        info_data = []

        if ids is not None and len(ids) > 0:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_length, K, self.D_eff)

            # Tìm một marker đã có trong bản đồ để cập nhật vị trí camera
            relocalized = False
            for i, marker_id in enumerate(ids.flatten()):
                marker_id_int = int(marker_id) # Ép kiểu sang int
                if marker_id_int in self.detected_markers:
                    # T_map_marker: Transform của marker trong map (từ bản đồ)
                    T_map_marker = self.pose_to_matrix(self.detected_markers[marker_id_int])
                    
                    # T_cam_marker: Transform của marker so với camera (từ ảnh)
                    T_cam_marker = self.tvec_rvec_to_matrix(tvecs[i], rvecs[i])

                    # Tính T_map_cam = T_map_marker * T_cam_marker^-1
                    self.T_map_cam = T_map_marker @ np.linalg.inv(T_cam_marker)
                    relocalized = True
                    break # Chỉ cần 1 marker để cập nhật vị trí

            # Thêm các marker mới vào bản đồ dựa trên vị trí camera đã cập nhật
            for i, marker_id in enumerate(ids.flatten()):
                marker_id_int = int(marker_id) # Ép kiểu sang int
                if marker_id_int not in self.detected_markers:
                    # Nếu bản đồ chưa có gì, marker đầu tiên sẽ định nghĩa gốc tọa độ
                    if not self.map_initialized:
                        # T_map_cam là identity, T_map_marker = T_cam_marker
                        T_map_marker = self.tvec_rvec_to_matrix(tvecs[i], rvecs[i])
                        self.map_initialized = True
                        self.get_logger().info(f"Bản đồ được khởi tạo với marker ID {marker_id_int}.")
                    else:
                        # Tính T_map_marker = T_map_cam * T_cam_marker
                        T_cam_marker = self.tvec_rvec_to_matrix(tvecs[i], rvecs[i])
                        T_map_marker = self.T_map_cam @ T_cam_marker

                    self.detected_markers[marker_id_int] = self.matrix_to_pose(T_map_marker)
                    self.get_logger().info(f"Đã thêm marker ID {marker_id_int} vào bản đồ.")

        # Publish thông tin: ID và khoảng cách tới gốc map cho tất cả marker trong bản đồ
        for marker_id, pose_in_map in self.detected_markers.items():
            position = pose_in_map.position
            distance = np.sqrt(position.x**2 + position.y**2 + position.z**2)
            info_data.extend([float(marker_id), distance]) # marker_id ở đây đã là int

        # Cấu hình layout cho message
        num_markers = len(info_data) // 2
        info_msg.layout.dim.append(MultiArrayDimension(label="markers", size=num_markers, stride=2*num_markers))
        info_msg.layout.dim.append(MultiArrayDimension(label="id_dist", size=2, stride=2))

        # --- Tối ưu: Chỉ publish khi có thay đổi ---
        # Sắp xếp để đảm bảo thứ tự nhất quán khi so sánh
        sorted_info = sorted(zip(info_data[0::2], info_data[1::2]))
        sorted_flat_info = [item for sublist in sorted_info for item in sublist]

        if sorted_flat_info != self.last_info_data:
            info_msg.data = info_data
            self.pub_info.publish(info_msg)
            self.last_info_data = sorted_flat_info

    def tvec_rvec_to_pose_msg(self, tvec, rvec):
        pose = Pose()
        tvec_flat = tvec.flatten()
        rvec_flat = rvec.flatten()

        # Gán vị trí và ép kiểu float() ngay lập tức
        pose.position.x = float(tvec_flat[0])
        pose.position.y = float(tvec_flat[1])
        pose.position.z = float(tvec_flat[2])
        
        rotation = Rotation.from_rotvec(rvec_flat)
        quat = rotation.as_quat() # Trả về [x, y, z, w]
        
        # Gán hướng và ép kiểu float() ngay lập tức
        pose.orientation.x = float(quat[0])
        pose.orientation.y = float(quat[1])
        pose.orientation.z = float(quat[2])
        pose.orientation.w = float(quat[3])
            
        return pose

    def save_map_to_yaml(self):
        self.get_logger().info(f"Đang lưu {len(self.detected_markers)} markers vào {self.map_save_path}...")
        
        # Tạo thư mục nếu chưa có
        os.makedirs(os.path.dirname(self.map_save_path), exist_ok=True)

        map_data = {}
        for marker_id, pose in self.detected_markers.items():
            map_data[marker_id] = {
                'position': {
                    'x': float(pose.position.x), 
                    'y': float(pose.position.y), 
                    'z': float(pose.position.z)},
                'orientation': {
                    'x': float(pose.orientation.x), 
                    'y': float(pose.orientation.y), 
                    'z': float(pose.orientation.z), 
                    'w': float(pose.orientation.w)}
            }

        with open(self.map_save_path, 'w') as f:
            yaml.dump({'markers': map_data}, f, default_flow_style=False, sort_keys=False)
        
        self.get_logger().info("💾 Lưu bản đồ thành công!")

    # --- Helper functions for matrix transformations ---
    def tvec_rvec_to_matrix(self, tvec, rvec):
        T = np.eye(4)
        T[:3, :3] = Rotation.from_rotvec(rvec.flatten()).as_matrix()
        T[:3, 3] = tvec.flatten()
        return T

    def pose_to_matrix(self, pose_msg):
        T = np.eye(4)
        q = [pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w]
        T[:3, :3] = Rotation.from_quat(q).as_matrix()
        T[:3, 3] = [pose_msg.position.x, pose_msg.position.y, pose_msg.position.z]
        return T

    def matrix_to_pose(self, T):
        pose = Pose()
        q = Rotation.from_matrix(T[:3, :3]).as_quat()
        pose.position.x = float(T[0, 3])
        pose.position.y = float(T[1, 3])
        pose.position.z = float(T[2, 3])
        pose.orientation.x = float(q[0])
        pose.orientation.y = float(q[1])
        pose.orientation.z = float(q[2])
        pose.orientation.w = float(q[3])
        return pose


    def destroy_node(self):
        self.running = False
        self.process_thread.join()
        self.capture_thread.join()
        if hasattr(self, "cap") and self.cap.isOpened():
            self.cap.release()
        
        if self.detected_markers:
            self.save_map_to_yaml()

        self.get_logger().info("Aruco Mapper đã dừng.")
        super().destroy_node()

def main():
    rclpy.init()
    node = ArucoMapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()