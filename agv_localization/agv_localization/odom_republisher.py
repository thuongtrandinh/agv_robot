#!/usr/bin/env python3
"""
Odometry Republisher with Enhanced Covariance
Adds/modifies covariance matrix to wheel odometry for use with robot_localization EKF

Author: Auto-generated for AGV localization
Date: November 7, 2025
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdomRepublisher(Node):
    """Republishes odometry with enhanced covariance for EKF fusion"""
    
    def __init__(self):
        super().__init__('odom_republisher')
        
        # Parameters
        self.declare_parameter('input_topic', '/diff_cont/odom')
        self.declare_parameter('output_topic', '/diff_cont/odom_with_covariance')
        
        # Odometry covariance values (tuned for differential drive)
        # Pose covariance: [x, y, z, roll, pitch, yaw]
        # Twist covariance: [vx, vy, vz, vroll, vpitch, vyaw]
        self.declare_parameter('pose_x_covariance', 0.001)
        self.declare_parameter('pose_y_covariance', 0.001)
        self.declare_parameter('pose_yaw_covariance', 0.01)
        self.declare_parameter('twist_vx_covariance', 0.002)
        self.declare_parameter('twist_vy_covariance', 0.002)
        self.declare_parameter('twist_vyaw_covariance', 0.02)
        
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        
        # Get covariance values
        pose_x_cov = self.get_parameter('pose_x_covariance').value
        pose_y_cov = self.get_parameter('pose_y_covariance').value
        pose_yaw_cov = self.get_parameter('pose_yaw_covariance').value
        twist_vx_cov = self.get_parameter('twist_vx_covariance').value
        twist_vy_cov = self.get_parameter('twist_vy_covariance').value
        twist_vyaw_cov = self.get_parameter('twist_vyaw_covariance').value
        
        # Build 6x6 covariance matrices
        # Pose: [x, y, z, roll, pitch, yaw]
        self.pose_covariance = [
            pose_x_cov, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, pose_y_cov, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e6, 0.0, 0.0, 0.0,  # z (unused)
            0.0, 0.0, 0.0, 1e6, 0.0, 0.0,  # roll (unused)
            0.0, 0.0, 0.0, 0.0, 1e6, 0.0,  # pitch (unused)
            0.0, 0.0, 0.0, 0.0, 0.0, pose_yaw_cov
        ]
        
        # Twist: [vx, vy, vz, vroll, vpitch, vyaw]
        self.twist_covariance = [
            twist_vx_cov, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, twist_vy_cov, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e6, 0.0, 0.0, 0.0,  # vz (unused)
            0.0, 0.0, 0.0, 1e6, 0.0, 0.0,  # vroll (unused)
            0.0, 0.0, 0.0, 0.0, 1e6, 0.0,  # vpitch (unused)
            0.0, 0.0, 0.0, 0.0, 0.0, twist_vyaw_cov
        ]
        
        # Publisher and Subscriber
        self.odom_pub = self.create_publisher(Odometry, output_topic, 10)
        self.odom_sub = self.create_subscription(
            Odometry,
            input_topic,
            self.odom_callback,
            10
        )
        
        self.get_logger().info('Odometry Republisher started')
        self.get_logger().info(f'  Input: {input_topic}')
        self.get_logger().info(f'  Output: {output_topic}')
        self.get_logger().info(f'  Pose covariance: x={pose_x_cov}, y={pose_y_cov}, yaw={pose_yaw_cov}')
        self.get_logger().info(f'  Twist covariance: vx={twist_vx_cov}, vy={twist_vy_cov}, vyaw={twist_vyaw_cov}')
    
    def odom_callback(self, msg):
        """Add/modify covariance in odometry message and republish"""
        
        # Create new message
        odom_out = Odometry()
        
        # Copy all data
        odom_out.header = msg.header
        odom_out.child_frame_id = msg.child_frame_id
        odom_out.pose.pose = msg.pose.pose
        odom_out.twist.twist = msg.twist.twist
        
        # Replace covariance matrices
        odom_out.pose.covariance = self.pose_covariance
        odom_out.twist.covariance = self.twist_covariance
        
        # Publish
        self.odom_pub.publish(odom_out)


def main(args=None):
    rclpy.init(args=args)
    node = OdomRepublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
