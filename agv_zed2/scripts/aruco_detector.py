#!/usr/bin/env python3
"""
ROS2 ZED2 ArUco Detector (CPU-only)
Package: agv_zed2

✅ Mở camera ZED2 qua /dev/video0 (2560×720)
✅ Cắt ảnh trái, resize về VGA (672×376)
✅ Dò ArUco (6x6_250, 17.3 cm)
✅ Publish /zed2/left/image_raw & /aruco/distances
✅ Không cần NVIDIA hay ZED SDK
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import time
import subprocess
import yaml
from ament_index_python.packages import get_package_share_directory


def load_zed_yaml(package_name: str, yaml_name: str, side: str = 'left'):
    """Đọc thông số nội tại camera từ file calib ZED2 VGA YAML."""
    try:
        pkg_share_path = get_package_share_directory(package_name)
        yaml_path = f"{pkg_share_path}/config/{yaml_name}"
    except Exception:
        raise FileNotFoundError(f"Không tìm thấy file calib: {yaml_path}")

    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    key = 'left_camera' if side.lower().startswith('l') else 'right_camera'
    cam = data[key]
    K = np.array(cam['camera_matrix']['data'], dtype=np.float64).reshape(3, 3)
    D = np.array(cam['distortion_coefficients']['data'], dtype=np.float64)
    return K, D

def find_zed_camera_id():
    """Tự động tìm device_id của ZED camera bằng cách quét v4l2-ctl."""
    try:
        result = subprocess.run(['v4l2-ctl', '--list-devices'], capture_output=True, text=True, check=True)
        lines = result.stdout.splitlines()
        for i, line in enumerate(lines):
            # ZED camera thường có tên "ZED" hoặc "ZED 2"
            if 'ZED' in line:
                # Dòng tiếp theo sẽ chứa đường dẫn /dev/videoX
                if i + 1 < len(lines):
                    dev_path = lines[i+1].strip()
                    if dev_path.startswith('/dev/video'):
                        # Trích xuất số ID từ chuỗi '/dev/videoX'
                        return int(dev_path.replace('/dev/video', ''))
    except (FileNotFoundError, subprocess.CalledProcessError, ValueError, IndexError) as e:
        # Nếu v4l2-ctl không tồn tại hoặc có lỗi, trả về -1
        print(f"Lỗi khi tìm ZED camera: {e}")
        return -1
    return -1 # Không tìm thấy


class ZED2ArucoNode(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        # --- Parameter ---
        # device_id = -1 nghĩa là tự động tìm kiếm
        self.declare_parameter('device_id', -1)
        self.declare_parameter('calib_pkg', 'agv_zed2')
        self.declare_parameter('calib_file', 'zed2_calibration_hd.yaml')
        self.declare_parameter('marker_size', 0.173)
        self.declare_parameter('aruco_dict', 'DICT_6X6_250')

        device_id_param = self.get_parameter('device_id').value
        calib_pkg = self.get_parameter('calib_pkg').value
        calib_file = self.get_parameter('calib_file').value
        self.marker_length = self.get_parameter('marker_size').value
        dict_name = self.get_parameter('aruco_dict').value

        # --- Load camera calibration ---
        # Luôn load calib gốc của camera, sau đó sẽ scale theo ảnh resize
        self.K_orig, self.D = load_zed_yaml(calib_pkg, calib_file, 'left')
        self.D_eff = np.zeros_like(self.D)

        # --- Tự động tìm camera ---
        device_id = device_id_param
        if device_id == -1:
            self.get_logger().info("Đang tự động tìm ZED camera...")
            device_id = find_zed_camera_id()
            if device_id != -1:
                self.get_logger().info(f"✅ Tìm thấy ZED camera tại /dev/video{device_id}")

        # --- Open camera ---
        self.cap = cv2.VideoCapture(device_id, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().fatal(f"❌ Không mở được /dev/video{device_id} (ZED2). Kiểm tra kết nối USB 3.0 và quyền truy cập.")
            raise RuntimeError(f"Camera open failed on device {device_id}")
        # Chuyển sang chế độ HD và tần số 60Hz
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.cap.set(cv2.CAP_PROP_FPS, 60)


        # --- ArUco setup ---
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
        # Handle OpenCV version differences for DetectorParameters
        if cv2.__version__.startswith('3'):
            self.aruco_params = cv2.aruco.DetectorParameters()
        else:
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        
        # Tinh chỉnh để dò marker ở xa tốt hơn
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        # --- Tối ưu 3: Bỏ qua frame tĩnh ---
        self.prev_gray = None
        self.last_info_data = None

        # --- ROS setup ---
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST
        self.bridge = CvBridge()
        self.pub_img = self.create_publisher(Image, '/zed2/left/image_raw', qos)
        self.pub_info = self.create_publisher(Float32MultiArray, '/aruco/info', 10)

        # --- Multi-threading setup ---
        # Tối ưu 1: Dùng Event flag thay vì sleep, giảm copy
        self.frame_buffer = None
        self.frame_lock = threading.Lock()
        self.new_frame_event = threading.Event()
        self.running = True

        # Thread 1: Đọc ảnh từ camera
        self.capture_thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.capture_thread.start()

        # Thread 2: Xử lý ảnh
        self.process_thread = threading.Thread(target=self.process_loop, daemon=True)
        self.process_thread.start()

        self.get_logger().info("✅ ZED2 ArUco Detector running (HD Mode @ 60Hz, 3-Thread Arch)")

    def capture_loop(self):
        """Liên tục đọc frame mới nhất từ camera."""
        while rclpy.ok() and self.running:
            ret, frame = self.cap.read()
            if ret:
                # Ghi vào buffer và báo hiệu cho thread xử lý
                with self.frame_lock:
                    self.frame_buffer = frame
                self.new_frame_event.set() # Báo có frame mới
            else:
                time.sleep(0.01) # Nếu đọc lỗi, đợi 1 chút

    def process_loop(self):
        """Lấy frame mới nhất để xử lý."""
        while rclpy.ok() and self.running:
            # Đợi tín hiệu có frame mới, timeout 50ms
            if self.new_frame_event.wait(timeout=0.05):
                frame_to_process = None
                with self.frame_lock:
                    # Lấy frame và xóa cờ hiệu
                    frame_to_process = self.frame_buffer
                    self.new_frame_event.clear()
                
                if frame_to_process is not None:
                    self.process_frame(frame_to_process)

    def process_frame(self, frame):
        # Cắt nửa trái
        # Frame là 2560x720, ảnh trái là 1280x720 (HD)
        orig_h, orig_w = frame.shape[:2]
        left_hd = frame[:, :orig_w // 2]

        # --- Tối ưu 2: Resize sớm xuống VGA ---
        target_w, target_h = 672, 376
        left_vga = cv2.resize(left_hd, (target_w, target_h), interpolation=cv2.INTER_AREA)

        # Scale ma trận nội tại cho phù hợp ảnh VGA
        scale_x = target_w / (orig_w / 2)
        scale_y = target_h / orig_h
        K_scaled = self.K_orig.copy()
        K_scaled[0, 0] *= scale_x
        K_scaled[1, 1] *= scale_y
        K_scaled[0, 2] *= scale_x
        K_scaled[1, 2] *= scale_y

        # Dò ArUco và publish
        self.detect_and_publish(left_vga, K_scaled)

        # Publish ảnh để debug
        if self.pub_img.get_subscription_count() > 0:
            img_msg = self.bridge.cv2_to_imgmsg(left_vga, encoding='bgr8')
            self.pub_img.publish(img_msg)

    # ---------------------------------------------------------------------
    def detect_and_publish(self, cv_img, K):
        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)

        # --- Tối ưu 3: Bỏ qua frame tĩnh --- (Đã vô hiệu hóa)
        # Logic này gây ra việc không detect được ArUco khi marker đứng yên.
        # Để luôn detect, chúng ta sẽ bỏ qua phần này.
        # if self.prev_gray is not None:
        #     diff = cv2.absdiff(gray, self.prev_gray)
        #     if np.mean(diff) < 3.0: # Ngưỡng thay đổi nhỏ
        #         return # Bỏ qua, không detect

        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        info_msg = Float32MultiArray()
        info_data = []

        if ids is not None and len(ids) > 0:
            # Set layout for MultiArray
            # dim[0] = number of markers
            # dim[1] = number of fields per marker (id, dist)
            info_msg.layout.dim.append(MultiArrayDimension())
            info_msg.layout.dim.append(MultiArrayDimension())
            info_msg.layout.dim[0].label = "markers"
            info_msg.layout.dim[0].size = len(ids)
            info_msg.layout.dim[0].stride = 2 * len(ids)
            info_msg.layout.dim[1].label = "id_dist"
            info_msg.layout.dim[1].size = 2
            info_msg.layout.dim[1].stride = 2

            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_length, K, self.D_eff)
            for i in range(len(ids)):
                tvec = tvecs[i].reshape(3)
                dist = float(np.linalg.norm(tvec))
                info_data.append(float(ids[i][0]))
                info_data.append(dist)

        # --- Tối ưu 4: Chỉ publish khi có thay đổi ---
        if info_data != self.last_info_data:
            info_msg.data = info_data
            self.pub_info.publish(info_msg)
            self.last_info_data = info_data
        self.prev_gray = gray # Cập nhật prev_gray sau khi xử lý xong

    # ---------------------------------------------------------------------
    def destroy_node(self):
        self.running = False
        if hasattr(self, "cap") and self.cap.isOpened():
            self.cap.release()
        self.get_logger().info("Camera released, node stopped.")
        super().destroy_node()


def main():
    rclpy.init()
    node = ZED2ArucoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
