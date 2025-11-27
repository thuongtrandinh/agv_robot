#!/usr/bin/env python3
"""
Trajectory Publisher with Adaptive Speed Profiling (Safe Shutdown Version)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math
import time

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        
        self.declare_parameter('trajectory_type', 2)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('path_points', 200)
        self.declare_parameter('preview_time', 10.0)
        self.declare_parameter('center_x', 0.7)
        self.declare_parameter('center_y', 0.25)
        self.declare_parameter('radius', 1.0)
        self.declare_parameter('enable_publish', True)
        self.declare_parameter('trajectory_speed', 0.3) 
        self.declare_parameter('corner_speed_scale', 0.3) 
        self.declare_parameter('ramp_time', 3.0) 
        
        self.trajectory_type = self.get_parameter('trajectory_type').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.path_points = self.get_parameter('path_points').value
        self.preview_time = self.get_parameter('preview_time').value
        self.center_x = self.get_parameter('center_x').value
        self.center_y = self.get_parameter('center_y').value
        self.radius = self.get_parameter('radius').value
        self.enable_publish = self.get_parameter('enable_publish').value
        self.base_speed = self.get_parameter('trajectory_speed').value
        self.corner_scale = self.get_parameter('corner_speed_scale').value
        self.ramp_time = self.get_parameter('ramp_time').value
        
        self.path_pub = self.create_publisher(Path, '/trajectory', 10)
        
        self.is_ready = False
        self.startup_delay = 3.0
        self.current_time = 0.0
        self.real_time_elapsed = 0.0
        self.dt = 1.0 / self.publish_rate
        
        self.startup_timer = self.create_timer(self.startup_delay, self.on_startup)
        self.timer = self.create_timer(self.dt, self.publish_loop)
        
        self.get_logger().info(f'✅ Adaptive Trajectory Publisher Started')

    def on_startup(self):
        self.is_ready = True
        self.startup_timer.cancel()
        self.get_logger().info('🚀 Starting Trajectory Generation...')

    def calculate_speed_factor(self, t):
        ramp_factor = 1.0
        if self.real_time_elapsed < self.ramp_time:
             progress = self.real_time_elapsed / self.ramp_time
             ramp_factor = progress * progress * (3.0 - 2.0 * progress)

        shape_factor = 1.0
        if self.trajectory_type == 2: # SQUARE
            side = self.radius * 2.0
            perimeter_pos = (t * self.base_speed) % (4 * side)
            dist_on_side = perimeter_pos % side
            threshold = side * 0.15 
            dist_to_corner = min(dist_on_side, side - dist_on_side)
            if dist_to_corner < threshold:
                ratio = dist_to_corner / threshold
                shape_factor = self.corner_scale + (1.0 - self.corner_scale) * ratio
            else: shape_factor = 1.0

        elif self.trajectory_type == 3: # FIGURE-8
            A = self.radius / 2.0
            omega_nominal = self.base_speed / (2.0 * A)
            phase = (omega_nominal * t) % (2 * math.pi)
            dist_to_peak_1 = abs(phase - math.pi/2)
            dist_to_peak_2 = abs(phase - 3*math.pi/2)
            min_dist_peak = min(dist_to_peak_1, dist_to_peak_2)
            if min_dist_peak < (math.pi / 3.0):
                ratio = min_dist_peak / (math.pi / 3.0)
                shape_factor = self.corner_scale + (1.0 - self.corner_scale) * (ratio**0.5)
            else: shape_factor = 1.0
        return ramp_factor * shape_factor

    def trajectory_reference_square(self, t):
        side = self.radius * 2.0
        total_len = 4 * side
        s = (t * self.base_speed) % total_len
        if s < side: x = self.center_x + side/2; y = self.center_y - side/2 + s; th = math.pi/2
        elif s < 2*side: x = self.center_x + side/2 - (s - side); y = self.center_y + side/2; th = math.pi
        elif s < 3*side: x = self.center_x - side/2; y = self.center_y + side/2 - (s - 2*side); th = -math.pi/2
        else: x = self.center_x - side/2 + (s - 3*side); y = self.center_y - side/2; th = 0.0
        return x, y, th

    def trajectory_reference_figure8(self, t):
        A = self.radius / 2.0
        omega = self.base_speed / (2.0 * A)
        phi = omega * t
        x_rel = A * math.cos(phi); y_rel = A * math.sin(2*phi) / 2.0
        x = self.center_x + x_rel; y = self.center_y + y_rel
        dx = -A * omega * math.sin(phi); dy = A * omega * math.cos(2*phi)
        th = math.atan2(dy, dx)
        return x, y, th

    def trajectory_reference_circle(self, t):
        R = self.radius
        omega = self.base_speed / R
        phi = omega * t
        x = self.center_x + R * math.cos(phi); y = self.center_y + R * math.sin(phi); th = phi + math.pi/2
        return x, y, th

    def get_ref_pose(self, t):
        if self.trajectory_type == 1: return self.trajectory_reference_circle(t)
        elif self.trajectory_type == 2: return self.trajectory_reference_square(t)
        elif self.trajectory_type == 3: return self.trajectory_reference_figure8(t)
        return 0,0,0

    def publish_loop(self):
        if not self.is_ready: return
        
        if self.enable_publish:
            path_msg = Path()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = 'map'
            
            temp_time = self.current_time
            preview_dt = self.preview_time / self.path_points
            
            for _ in range(self.path_points):
                factor = self.calculate_speed_factor(temp_time)
                x, y, th = self.get_ref_pose(temp_time)
                temp_time += preview_dt 
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = x; pose.pose.position.y = y
                cy = math.cos(th * 0.5); sy = math.sin(th * 0.5)
                pose.pose.orientation.z = sy; pose.pose.orientation.w = cy
                path_msg.poses.append(pose)
            
            self.path_pub.publish(path_msg)

        speed_factor = self.calculate_speed_factor(self.current_time)
        effective_dt = self.dt * speed_factor
        self.current_time += effective_dt
        self.real_time_elapsed += self.dt 

        if self.current_time % 1.0 < self.dt: 
            self.get_logger().info(f'Speed Factor: {speed_factor:.2f} | T_ref: {self.current_time:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally: 
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()