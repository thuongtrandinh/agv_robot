#!/usr/bin/env python3
"""
ROS2 ArUco Localizer
Package: agv_zed2

Chức năng:
✅ Tải bản đồ ArUco từ file YAML.
✅ Mở camera ZED2.
✅ Dò tìm các ArUco marker.
✅ Nếu marker có trong bản đồ, tính toán pose của camera (robot) trong hệ tọa độ map.
✅ Publish pose của robot và broadcast TF transform (map -> camera_link).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, TransformStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import time
import subprocess
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose # Import Pose message type
import os
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation

def load_zed_yaml(package_name: str, yaml_name: str, side: str = 'left'):
    pkg_share_path = get_package_share_directory(package_name)
    yaml_path = os.path.join(pkg_share_path, 'config', yaml_name)
    
    # --- Cấu hình PyYAML để xử lý các kiểu dữ liệu của Numpy ---
    # Điều này đảm bảo file YAML được đọc với định dạng số chuẩn.
    # (Chỉ cần thiết nếu file YAML được tạo ra bởi một công cụ khác không ép kiểu)
    def numpy_float_representer(dumper, data):
        return dumper.represent_scalar(u'tag:yaml.org,2002:float', str(data))
    yaml.add_representer(np.float64, numpy_float_representer)
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

class ArucoLocalizerNode(Node):
    def __init__(self):
        super().__init__('aruco_localizer')

        # --- Parameters ---
        self.declare_parameter('device_id', -1)
        self.declare_parameter('calib_pkg', 'agv_zed2')
        self.declare_parameter('calib_file', 'zed2_calibration_hd.yaml')
        self.declare_parameter('marker_size', 0.173)
        self.declare_parameter('aruco_dict', 'DICT_6X6_250')
        self.declare_parameter('map_file', 'aruco_map.yaml')
        self.declare_parameter('camera_frame', 'camera_link')

        device_id_param = self.get_parameter('device_id').value
        calib_pkg = self.get_parameter('calib_pkg').value
        calib_file = self.get_parameter('calib_file').value
        self.marker_length = self.get_parameter('marker_size').value
        dict_name = self.get_parameter('aruco_dict').value
        map_file = self.get_parameter('map_file').value
        self.camera_frame = self.get_parameter('camera_frame').value

        # --- Load Map ---
        self.aruco_map = self.load_map_from_yaml(map_file)

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
        # self.bridge = CvBridge() # Không cần nếu không publish ảnh
        # self.pub_img = self.create_publisher(Image, '/aruco_localizer/image', qos) # Xóa publisher ảnh
        self.pub_pose = self.create_publisher(PoseStamped, '/robot_pose', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- Multi-threading setup ---
        self.frame_buffer = None
        self.frame_lock = threading.Lock()
        self.new_frame_event = threading.Event()
        self.running = True

        self.capture_thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.capture_thread.start()

        self.process_thread = threading.Thread(target=self.process_loop, daemon=True)
        self.process_thread.start()

        self.get_logger().info(f"✅ Aruco Localizer đang chạy. Định vị bằng {len(self.aruco_map)} markers.")

    def load_map_from_yaml(self, map_file):
        map_path = os.path.join(get_package_share_directory('agv_zed2'), 'map', map_file)
        if not os.path.exists(map_path):
            self.get_logger().error(f"Không tìm thấy file bản đồ: {map_path}. Hãy chạy node mapper trước.")
            return {}
        
        with open(map_path, 'r') as f:
            data = yaml.safe_load(f)
        
        self.get_logger().info(f"Tải bản đồ thành công từ {map_path}")
        return data.get('markers', {})

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

        self.localize_from_markers(left_vga, K_scaled)

        # Xóa phần publish ảnh gỡ lỗi
        # if self.pub_img.get_subscription_count() > 0:
        #     img_msg = self.bridge.cv2_to_imgmsg(left_vga, encoding='bgr8')
        #     self.pub_img.publish(img_msg)

    def localize_from_markers(self, cv_img, K):
        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None and len(ids) > 0:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_length, K, self.D_eff)

            for i, marker_id in enumerate(ids):
                marker_id = int(marker_id)
                if marker_id in self.aruco_map:
                    # --- Tính toán pose của camera trong hệ tọa độ map ---
                    # T_map_marker: Transform từ map -> marker (lấy từ bản đồ)
                    map_marker_data = self.aruco_map[marker_id]
                    
                    # Chuyển dữ liệu từ YAML sang đối tượng Pose để dùng hàm pose_to_matrix
                    pose_map_marker = Pose()
                    pose_map_marker.position.x = map_marker_data['position']['x']
                    pose_map_marker.position.y = map_marker_data['position']['y']
                    pose_map_marker.position.z = map_marker_data['position']['z']
                    pose_map_marker.orientation.x = map_marker_data['orientation']['x']
                    pose_map_marker.orientation.y = map_marker_data['orientation']['y']
                    pose_map_marker.orientation.z = map_marker_data['orientation']['z']
                    pose_map_marker.orientation.w = map_marker_data['orientation']['w']
                    T_map_marker = self.pose_to_matrix(pose_map_marker)

                    T_cam_marker = self.tvec_rvec_to_matrix(tvecs[i], rvecs[i])

                    # T_map_cam = T_map_marker * T_marker_cam = T_map_marker * T_cam_marker^-1
                    T_map_cam = T_map_marker @ np.linalg.linalg.inv(T_cam_marker)
                    
                    # Chuyển đổi ma trận T_map_cam sang Pose message
                    robot_pose = self.matrix_to_pose(T_map_cam)

                    # Publish PoseStamped
                    pose_msg = PoseStamped()
                    pose_msg.header.stamp = self.get_clock().now().to_msg()
                    pose_msg.header.frame_id = "map"
                    pose_msg.pose = robot_pose
                    self.pub_pose.publish(pose_msg)

                    # Broadcast TF
                    t = TransformStamped()
                    t.header.stamp = pose_msg.header.stamp
                    t.header.frame_id = "map"
                    t.child_frame_id = self.camera_frame
                    t.transform.translation.x = robot_pose.position.x
                    t.transform.translation.y = robot_pose.position.y
                    t.transform.translation.z = robot_pose.position.z
                    t.transform.rotation.x = robot_pose.orientation.x
                    t.transform.rotation.y = robot_pose.orientation.y
                    t.transform.rotation.z = robot_pose.orientation.z
                    t.transform.rotation.w = robot_pose.orientation.w
                    self.tf_broadcaster.sendTransform(t)

                    # Chỉ cần định vị từ 1 marker là đủ, thoát vòng lặp
                    break

    # --- Helper functions for matrix transformations (copied from aruco_mapper.py) ---
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
        self.get_logger().info("Aruco Localizer đã dừng.")
        super().destroy_node()

def main():
    rclpy.init()
    node = ArucoLocalizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()