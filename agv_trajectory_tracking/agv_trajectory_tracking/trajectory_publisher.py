#!/usr/bin/env python3
"""
Trajectory Publisher with Adaptive Speed Profiling
Publishes trajectory points for the AGV to follow.

Author: Thuong Tran Dinh
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math


class TrajectoryPublisher(Node):
    """Publishes trajectory for AGV to follow."""
    
    # Trajectory types
    CIRCLE = 1
    SQUARE = 2
    FIGURE_8 = 3
    
    def __init__(self):
        super().__init__('trajectory_publisher')
        
        # Parameters
        self.declare_parameter('trajectory_type', self.SQUARE)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('path_points', 200)
        self.declare_parameter('preview_time', 10.0)
        self.declare_parameter('center_x', 0.7)
        self.declare_parameter('center_y', 0.25)
        self.declare_parameter('radius', 1.0)
        self.declare_parameter('trajectory_speed', 0.3)
        self.declare_parameter('corner_speed_scale', 0.3)
        
        self.trajectory_type = self.get_parameter('trajectory_type').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.path_points = self.get_parameter('path_points').value
        self.preview_time = self.get_parameter('preview_time').value
        self.center_x = self.get_parameter('center_x').value
        self.center_y = self.get_parameter('center_y').value
        self.radius = self.get_parameter('radius').value
        self.base_speed = self.get_parameter('trajectory_speed').value
        self.corner_scale = self.get_parameter('corner_speed_scale').value
        
        # Publisher
        self.path_pub = self.create_publisher(Path, '/trajectory', 10)
        
        # State
        self.is_ready = False
        self.startup_delay = 3.0
        self.current_time = 0.0
        self.dt = 1.0 / self.publish_rate
        
        # Timers
        self.startup_timer = self.create_timer(self.startup_delay, self.on_startup)
        self.timer = self.create_timer(self.dt, self.publish_loop)
        
        self.get_logger().info(f'Trajectory Publisher started (type={self.trajectory_type})')

    def on_startup(self):
        """Called after startup delay."""
        self.is_ready = True
        self.startup_timer.cancel()
        self.get_logger().info('Starting trajectory generation...')

    def calculate_speed_factor(self, t):
        """Calculate speed factor based on trajectory shape."""
        if self.trajectory_type == self.SQUARE:
            side = self.radius * 2.0
            perimeter_pos = (t * self.base_speed) % (4 * side)
            dist_on_side = perimeter_pos % side
            threshold = side * 0.15
            dist_to_corner = min(dist_on_side, side - dist_on_side)
            if dist_to_corner < threshold:
                ratio = dist_to_corner / threshold
                return self.corner_scale + (1.0 - self.corner_scale) * ratio
            return 1.0

        elif self.trajectory_type == self.FIGURE_8:
            A = self.radius / 2.0
            omega = self.base_speed / (2.0 * A)
            phase = (omega * t) % (2 * math.pi)
            dist_to_peak_1 = abs(phase - math.pi/2)
            dist_to_peak_2 = abs(phase - 3*math.pi/2)
            min_dist_peak = min(dist_to_peak_1, dist_to_peak_2)
            if min_dist_peak < (math.pi / 3.0):
                ratio = min_dist_peak / (math.pi / 3.0)
                return self.corner_scale + (1.0 - self.corner_scale) * (ratio**0.5)
            return 1.0
        
        return 1.0

    def get_reference_pose(self, t):
        """Get reference pose at time t."""
        if self.trajectory_type == self.CIRCLE:
            return self._circle_pose(t)
        elif self.trajectory_type == self.SQUARE:
            return self._square_pose(t)
        elif self.trajectory_type == self.FIGURE_8:
            return self._figure8_pose(t)
        return 0.0, 0.0, 0.0

    def _circle_pose(self, t):
        """Circle trajectory."""
        R = self.radius
        omega = self.base_speed / R
        phi = omega * t
        x = self.center_x + R * math.cos(phi)
        y = self.center_y + R * math.sin(phi)
        theta = phi + math.pi/2
        return x, y, theta

    def _square_pose(self, t):
        """Square trajectory."""
        side = self.radius * 2.0
        total_len = 4 * side
        s = (t * self.base_speed) % total_len
        
        if s < side:
            x = self.center_x + side/2
            y = self.center_y - side/2 + s
            theta = math.pi/2
        elif s < 2*side:
            x = self.center_x + side/2 - (s - side)
            y = self.center_y + side/2
            theta = math.pi
        elif s < 3*side:
            x = self.center_x - side/2
            y = self.center_y + side/2 - (s - 2*side)
            theta = -math.pi/2
        else:
            x = self.center_x - side/2 + (s - 3*side)
            y = self.center_y - side/2
            theta = 0.0
        
        return x, y, theta

    def _figure8_pose(self, t):
        """Figure-8 trajectory."""
        A = self.radius / 2.0
        omega = self.base_speed / (2.0 * A)
        phi = omega * t
        
        x = self.center_x + A * math.cos(phi)
        y = self.center_y + A * math.sin(2*phi) / 2.0
        
        dx = -A * omega * math.sin(phi)
        dy = A * omega * math.cos(2*phi)
        theta = math.atan2(dy, dx)
        
        return x, y, theta

    def publish_loop(self):
        """Main publish loop."""
        if not self.is_ready:
            return
        
        # Create path message
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        
        temp_time = self.current_time
        preview_dt = self.preview_time / self.path_points
        
        for _ in range(self.path_points):
            x, y, theta = self.get_reference_pose(temp_time)
            temp_time += preview_dt
            
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            
            # Convert theta to quaternion (only yaw)
            cy = math.cos(theta * 0.5)
            sy = math.sin(theta * 0.5)
            pose.pose.orientation.z = sy
            pose.pose.orientation.w = cy
            
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)
        
        # Update time with speed factor
        speed_factor = self.calculate_speed_factor(self.current_time)
        self.current_time += self.dt * speed_factor


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
