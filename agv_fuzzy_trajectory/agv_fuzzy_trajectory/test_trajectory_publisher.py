#!/usr/bin/env python3
"""
Test trajectory publisher for fuzzy trajectory controller
Publishes a simple square path trajectory
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math


class TestTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('test_trajectory_publisher')
        
        # Publisher
        self.trajectory_pub = self.create_publisher(Path, '/trajectory', 10)
        
        # Timer to publish trajectory
        self.timer = self.create_timer(2.0, self.publish_trajectory)
        
        self.get_logger().info('Test Trajectory Publisher initialized')
        
    def publish_trajectory(self):
        """Publish a simple square trajectory"""
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'odom'
        
        # Create a square path: 2m x 2m
        # Starting at origin, going counter-clockwise
        waypoints = [
            (0.0, 0.0, 0.0),      # Start
            (2.0, 0.0, 0.0),      # Right
            (2.0, 2.0, math.pi/2),  # Up
            (0.0, 2.0, math.pi),    # Left
            (0.0, 0.0, -math.pi/2), # Down back to start
        ]
        
        for x, y, yaw in waypoints:
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
        self.get_logger().info(f'Published trajectory with {len(path.poses)} waypoints')


def main(args=None):
    rclpy.init(args=args)
    node = TestTrajectoryPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
