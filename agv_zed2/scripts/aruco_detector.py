#!/usr/bin/env python3
"""
ROS2 ZED2 ArUco FAST Detector (CPU-only, optimized)
FPS: 30+ on IPC
Author: Thien Huynh
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge

import cv2
import numpy as np
import threading
import time
import subprocess
import yaml
from ament_index_python.packages import get_package_share_directory


# --------------------------------------------------------
#  Load ZED calibration YAML
# --------------------------------------------------------
def load_zed_yaml(package_name: str, yaml_name: str):
    pkg_share_path = get_package_share_directory(package_name)
    yaml_path = f"{pkg_share_path}/config/{yaml_name}"

    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    cam = data["left_camera"]

    K = np.array(cam['camera_matrix']['data'], dtype=np.float64).reshape(3, 3)
    D = np.array(cam['distortion_coefficients']['data'], dtype=np.float64)

    return K, D


# --------------------------------------------------------
#  Auto-detect ZED device ID
# --------------------------------------------------------
def find_zed_camera_id():
    try:
        result = subprocess.run(
            ['v4l2-ctl', '--list-devices'],
            capture_output=True, text=True, check=True
        )
        lines = result.stdout.splitlines()

        for i, line in enumerate(lines):
            if 'ZED' in line:
                if i + 1 < len(lines):
                    dev_path = lines[i+1].strip()
                    if dev_path.startswith('/dev/video'):
                        return int(dev_path.replace('/dev/video', ''))

    except Exception as e:
        print(f"[WARN] Could not auto-detect ZED: {e}")
        return -1

    return -1


# --------------------------------------------------------
#             MAIN DETECTOR NODE
# --------------------------------------------------------
class ZED2ArucoNode(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        # ---------------------
        # ROS Params
        # ---------------------
        self.declare_parameter('device_id', -1)
        self.declare_parameter('calib_pkg', 'agv_zed2')
        self.declare_parameter('calib_file', 'zed2_calibration_hd.yaml')
        self.declare_parameter('marker_size', 0.173)
        self.declare_parameter('aruco_dict', 'DICT_6X6_250')

        device_id = self.get_parameter('device_id').value
        calib_pkg = self.get_parameter('calib_pkg').value
        calib_file = self.get_parameter('calib_file').value
        self.marker_length = float(self.get_parameter('marker_size').value)
        dict_name = self.get_parameter('aruco_dict').value

        # ---------------------
        # Load calibration
        # ---------------------
        self.K_hd, self.D = load_zed_yaml(calib_pkg, calib_file)

        # ---------------------
        # Auto-detect ZED
        # ---------------------
        if device_id == -1:
            self.get_logger().info("Auto scanning ZED camera...")
            device_id = find_zed_camera_id()
            if device_id != -1:
                self.get_logger().info(f"Found ZED at /dev/video{device_id}")

        # ---------------------
        # Setup camera
        # ---------------------
        self.cap = cv2.VideoCapture(device_id, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f"❌ Cannot open /dev/video{device_id}")

        # HD mode
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        # ---------------------
        # ArUco setup
        # ---------------------
        self.dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        # ---------------------
        # Resize target (VGA)
        # ---------------------
        self.vga_w = 672
        self.vga_h = 376

        scale_x = self.vga_w / 1280
        scale_y = self.vga_h / 720

        self.K_vga = self.K_hd.copy()
        self.K_vga[0, 0] *= scale_x
        self.K_vga[0, 2] *= scale_x
        self.K_vga[1, 1] *= scale_y
        self.K_vga[1, 2] *= scale_y

        # ---------------------
        # ROS Publishers
        # ---------------------
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.bridge = CvBridge()
        self.pub_img = self.create_publisher(Image, "/zed2/left/image_raw", qos)
        self.pub_det = self.create_publisher(Float32MultiArray, "/aruco/detections", 10)

        # ---------------------
        # Internal buffer
        # ---------------------
        self.frame_lock = threading.Lock()
        self.frame = None
        self.new_frame_event = threading.Event()
        self.running = True

        # ---------------------
        # Start worker threads
        # ---------------------
        threading.Thread(target=self.capture_loop, daemon=True).start()
        threading.Thread(target=self.process_loop, daemon=True).start()

        self.get_logger().info("🚀 ArUco FAST Detector running at ~30 FPS")

    # --------------------------------------------------------
    # CAPTURE THREAD
    # --------------------------------------------------------
    def capture_loop(self):
        while rclpy.ok() and self.running:
            ret, f = self.cap.read()
            if ret:
                with self.frame_lock:
                    self.frame = f
                self.new_frame_event.set()
            else:
                time.sleep(0.005)

    # --------------------------------------------------------
    # PROCESS THREAD
    # --------------------------------------------------------
    def process_loop(self):
        while rclpy.ok() and self.running:
            if not self.new_frame_event.wait(timeout=0.03):
                continue

            self.new_frame_event.clear()
            with self.frame_lock:
                frame = self.frame.copy()

            if frame is not None:
                self.process_frame(frame)

    # --------------------------------------------------------
    # ARUCO PROCESSING
    # --------------------------------------------------------
    def process_frame(self, frame):
        h, w = frame.shape[:2]
        left = frame[:, :w // 2]

        # Resize to VGA
        img_vga = cv2.resize(left, (self.vga_w, self.vga_h), cv2.INTER_AREA)

        gray = cv2.cvtColor(img_vga, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.dictionary, parameters=self.aruco_params
        )

        msg = Float32MultiArray()
        data = []

        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.K_vga, self.D
            )

            for i in range(len(ids)):
                marker_id = int(ids[i][0])
                tvec = tvecs[i].reshape(3)
                rvec = rvecs[i].reshape(3)

                # Convert rvec → roll pitch yaw
                R, _ = cv2.Rodrigues(rvec)
                sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
                roll = np.arctan2(R[2, 1], R[2, 2])
                pitch = np.arctan2(-R[2, 0], sy)
                yaw = np.arctan2(R[1, 0], R[0, 0])

                data.extend([marker_id, tvec[0], tvec[1], tvec[2], roll, pitch, yaw])

        msg.data = data
        self.pub_det.publish(msg)

        # Debug image
        if self.pub_img.get_subscription_count() > 0:
            self.pub_img.publish(self.bridge.cv2_to_imgmsg(img_vga, "bgr8"))

    # --------------------------------------------------------
    # CLEANUP
    # --------------------------------------------------------
    def destroy_node(self):
        self.running = False
        if hasattr(self, "cap") and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


# --------------------------------------------------------
#  MAIN
# --------------------------------------------------------
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


if __name__ == "__main__":
    main()
