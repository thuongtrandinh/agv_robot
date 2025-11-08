#!/usr/bin/env python3
"""
IMU Republisher with Covariance
Adds covariance matrix to Gazebo IMU data for use with robot_localization EKF

Author: Auto-generated for AGV localization
Date: November 7, 2025
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np


class ImuRepublisher(Node):
    """Republishes IMU data with proper covariance for EKF fusion"""
    
    def __init__(self):
        super().__init__('imu_republisher')
        
        # Parameters
        self.declare_parameter('input_topic', '/imu')
        self.declare_parameter('output_topic', '/imu_with_covariance')
        
        # IMU covariance values (tuned for Gazebo IMU)
        # Diagonal: [orientation_x, orientation_y, orientation_z,
        #            angular_velocity_x, angular_velocity_y, angular_velocity_z,
        #            linear_acceleration_x, linear_acceleration_y, linear_acceleration_z]
        self.declare_parameter('orientation_covariance', 0.01)
        self.declare_parameter('angular_velocity_covariance', 0.02)
        self.declare_parameter('linear_acceleration_covariance', 0.04)
        
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        
        # Covariance matrices (3x3 diagonal)
        orient_cov = self.get_parameter('orientation_covariance').value
        ang_vel_cov = self.get_parameter('angular_velocity_covariance').value
        lin_acc_cov = self.get_parameter('linear_acceleration_covariance').value
        
        self.orientation_covariance = [
            orient_cov, 0.0, 0.0,
            0.0, orient_cov, 0.0,
            0.0, 0.0, orient_cov
        ]
        
        self.angular_velocity_covariance = [
            ang_vel_cov, 0.0, 0.0,
            0.0, ang_vel_cov, 0.0,
            0.0, 0.0, ang_vel_cov
        ]
        
        self.linear_acceleration_covariance = [
            lin_acc_cov, 0.0, 0.0,
            0.0, lin_acc_cov, 0.0,
            0.0, 0.0, lin_acc_cov
        ]
        
        # Publisher and Subscriber
        self.imu_pub = self.create_publisher(Imu, output_topic, 10)
        self.imu_sub = self.create_subscription(
            Imu, 
            input_topic, 
            self.imu_callback, 
            10
        )
        
        self.get_logger().info('IMU Republisher started')
        self.get_logger().info(f'  Input: {input_topic}')
        self.get_logger().info(f'  Output: {output_topic}')
        self.get_logger().info(f'  Orientation cov: {orient_cov}')
        self.get_logger().info(f'  Angular velocity cov: {ang_vel_cov}')
        self.get_logger().info(f'  Linear acceleration cov: {lin_acc_cov}')
    
    def imu_callback(self, msg):
        """Add covariance to IMU message and republish"""
        
        # Create new message with covariance
        imu_out = Imu()
        
        # Copy header and data
        imu_out.header = msg.header
        imu_out.orientation = msg.orientation
        imu_out.angular_velocity = msg.angular_velocity
        imu_out.linear_acceleration = msg.linear_acceleration
        
        # Add covariance matrices
        imu_out.orientation_covariance = self.orientation_covariance
        imu_out.angular_velocity_covariance = self.angular_velocity_covariance
        imu_out.linear_acceleration_covariance = self.linear_acceleration_covariance
        
        # Publish
        self.imu_pub.publish(imu_out)


def main(args=None):
    rclpy.init(args=args)
    node = ImuRepublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
