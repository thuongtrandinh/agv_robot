#!/usr/bin/env python3
"""
Circle Trajectory Publisher for Fuzzy Trajectory Controller
Publishes a circular path for the robot to follow
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math


class CircleTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('circle_trajectory_publisher')
        
        # Declare parameters
        self.declare_parameter('radius', 2.0)  # meters
        self.declare_parameter('center_x', 2.0)  # meters
        self.declare_parameter('center_y', 2.0)  # meters
        self.declare_parameter('num_points', 50)  # number of waypoints
        self.declare_parameter('publish_rate', 1.0)  # Hz
        
        # Get parameters
        self.radius = self.get_parameter('radius').value
        self.center_x = self.get_parameter('center_x').value
        self.center_y = self.get_parameter('center_y').value
        self.num_points = self.get_parameter('num_points').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # Publisher
        self.trajectory_pub = self.create_publisher(Path, '/trajectory', 10)
        
        # Timer to publish trajectory
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_trajectory)
        
        self.get_logger().info('Circle Trajectory Publisher initialized')
        self.get_logger().info(f'Circle: radius={self.radius}m, center=({self.center_x}, {self.center_y}), points={self.num_points}')
        
    def publish_trajectory(self):
        """Publish a circular trajectory"""
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'odom'
        
        # Generate circle waypoints
        for i in range(self.num_points + 1):  # +1 to close the circle
            angle = 2.0 * math.pi * i / self.num_points
            
            # Calculate position on circle
            x = self.center_x + self.radius * math.cos(angle)
            y = self.center_y + self.radius * math.sin(angle)
            
            # Calculate tangent angle (direction of movement)
            # Tangent is perpendicular to radius, so add π/2 to angle
            yaw = angle + math.pi / 2.0
            
            # Create pose
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            
            # Convert yaw to quaternion (simplified for 2D)
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            
            path.poses.append(pose)
        
        self.trajectory_pub.publish(path)
        self.get_logger().info(f'Published circular trajectory with {len(path.poses)} waypoints', 
                               throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = CircleTrajectoryPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
