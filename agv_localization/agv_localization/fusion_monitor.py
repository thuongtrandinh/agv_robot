#!/usr/bin/env python3
"""
Fusion Monitor - Compare AMCL, ArUco, and Fused poses
Author: Thuong Tran Dinh
Date: October 27, 2025

This node subscribes to:
1. /amcl_pose - AMCL localization
2. /aruco/pose_with_covariance - ArUco absolute position
3. /odometry/map_fused - EKF fused result

It publishes:
- Statistics and comparison metrics
- Visualization markers showing the difference
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA
import math
import time


class FusionMonitor(Node):
    def __init__(self):
        super().__init__('fusion_monitor')
        
        # Store latest poses
        self.amcl_pose = None
        self.aruco_pose = None
        self.fused_pose = None
        
        self.amcl_time = None
        self.aruco_time = None
        self.fused_time = None
        
        self.aruco_detected = False
        self.last_aruco_time = time.time()
        
        # Statistics
        self.amcl_aruco_diff_history = []
        self.amcl_fused_diff_history = []
        self.aruco_fused_diff_history = []
        
        # Subscribers
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            10
        )
        
        self.aruco_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/aruco/pose_with_covariance',
            self.aruco_callback,
            10
        )
        
        self.fused_sub = self.create_subscription(
            Odometry,
            '/odometry/map_fused',
            self.fused_callback,
            10
        )
        
        # Publishers
        self.marker_pub = self.create_publisher(MarkerArray, '/fusion_markers', 10)
        
        # Timer for periodic comparison
        self.timer = self.create_timer(1.0, self.compare_and_publish)
        
        self.get_logger().info('🔍 Fusion Monitor started!')
        self.get_logger().info('   Monitoring:')
        self.get_logger().info('     - /amcl_pose (red)')
        self.get_logger().info('     - /aruco/pose_with_covariance (green)')
        self.get_logger().info('     - /odometry/map_fused (blue)')
    
    def amcl_callback(self, msg):
        self.amcl_pose = msg.pose.pose
        self.amcl_time = self.get_clock().now()
    
    def aruco_callback(self, msg):
        self.aruco_pose = msg.pose.pose
        self.aruco_time = self.get_clock().now()
        self.aruco_detected = True
        self.last_aruco_time = time.time()
    
    def fused_callback(self, msg):
        self.fused_pose = msg.pose.pose
        self.fused_time = self.get_clock().now()
    
    def calculate_distance(self, pose1, pose2):
        """Calculate Euclidean distance between two poses"""
        if pose1 is None or pose2 is None:
            return None
        
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        dz = pose1.position.z - pose2.position.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)
    
    def calculate_angle_diff(self, pose1, pose2):
        """Calculate yaw angle difference between two poses"""
        if pose1 is None or pose2 is None:
            return None
        
        yaw1 = self.quaternion_to_yaw(pose1.orientation)
        yaw2 = self.quaternion_to_yaw(pose2.orientation)
        
        diff = yaw1 - yaw2
        # Normalize to [-pi, pi]
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        
        return math.degrees(diff)
    
    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw angle"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def get_covariance_trace(self, cov_matrix):
        """Get trace of covariance matrix (sum of diagonal)"""
        # Covariance is 6x6: [x, y, z, roll, pitch, yaw]
        # We only care about x, y, yaw
        return cov_matrix[0] + cov_matrix[7] + cov_matrix[35]
    
    def compare_and_publish(self):
        """Compare poses and publish visualization + statistics"""
        
        # Check if ArUco recently detected
        time_since_aruco = time.time() - self.last_aruco_time
        if time_since_aruco < 2.0:
            self.aruco_detected = True
        else:
            self.aruco_detected = False
        
        # Print header
        self.get_logger().info('═' * 80)
        self.get_logger().info('📊 FUSION MONITOR REPORT')
        self.get_logger().info('═' * 80)
        
        # Status
        status = []
        if self.amcl_pose is not None:
            status.append('✅ AMCL')
        else:
            status.append('❌ AMCL')
        
        if self.aruco_detected:
            status.append('✅ ArUco (DETECTED)')
        else:
            status.append('⚠️  ArUco (NOT DETECTED)')
        
        if self.fused_pose is not None:
            status.append('✅ Fused')
        else:
            status.append('❌ Fused')
        
        self.get_logger().info('Status: ' + ' | '.join(status))
        
        # Calculate differences
        if self.amcl_pose and self.aruco_pose and self.aruco_detected:
            amcl_aruco_dist = self.calculate_distance(self.amcl_pose, self.aruco_pose)
            amcl_aruco_angle = self.calculate_angle_diff(self.amcl_pose, self.aruco_pose)
            
            self.get_logger().info('─' * 80)
            self.get_logger().info('📏 AMCL vs ArUco:')
            self.get_logger().info(f'   Position difference: {amcl_aruco_dist:.4f} m')
            self.get_logger().info(f'   Angle difference:    {amcl_aruco_angle:.2f} deg')
            
            if amcl_aruco_dist > 0.1:
                self.get_logger().warn(f'   ⚠️  Large difference detected! AMCL might be drifting.')
            
            self.amcl_aruco_diff_history.append(amcl_aruco_dist)
            if len(self.amcl_aruco_diff_history) > 10:
                self.amcl_aruco_diff_history.pop(0)
        
        if self.amcl_pose and self.fused_pose:
            amcl_fused_dist = self.calculate_distance(self.amcl_pose, self.fused_pose)
            amcl_fused_angle = self.calculate_angle_diff(self.amcl_pose, self.fused_pose)
            
            self.get_logger().info('─' * 80)
            self.get_logger().info('📏 AMCL vs Fused:')
            self.get_logger().info(f'   Position difference: {amcl_fused_dist:.4f} m')
            self.get_logger().info(f'   Angle difference:    {amcl_fused_angle:.2f} deg')
            
            self.amcl_fused_diff_history.append(amcl_fused_dist)
            if len(self.amcl_fused_diff_history) > 10:
                self.amcl_fused_diff_history.pop(0)
        
        if self.aruco_pose and self.fused_pose and self.aruco_detected:
            aruco_fused_dist = self.calculate_distance(self.aruco_pose, self.fused_pose)
            aruco_fused_angle = self.calculate_angle_diff(self.aruco_pose, self.fused_pose)
            
            self.get_logger().info('─' * 80)
            self.get_logger().info('📏 ArUco vs Fused:')
            self.get_logger().info(f'   Position difference: {aruco_fused_dist:.4f} m')
            self.get_logger().info(f'   Angle difference:    {aruco_fused_angle:.2f} deg')
            
            if aruco_fused_dist < 0.05:
                self.get_logger().info('   ✅ Fusion is following ArUco closely (good!)')
            elif aruco_fused_dist < 0.15:
                self.get_logger().info('   ⚠️  Fusion is moderately following ArUco')
            else:
                self.get_logger().warn('   ❌ Fusion is NOT following ArUco well')
            
            self.aruco_fused_diff_history.append(aruco_fused_dist)
            if len(self.aruco_fused_diff_history) > 10:
                self.aruco_fused_diff_history.pop(0)
        
        # Print average differences
        if len(self.amcl_aruco_diff_history) > 0:
            avg_amcl_aruco = sum(self.amcl_aruco_diff_history) / len(self.amcl_aruco_diff_history)
            self.get_logger().info('─' * 80)
            self.get_logger().info(f'📈 Average AMCL-ArUco diff (last 10): {avg_amcl_aruco:.4f} m')
        
        if len(self.amcl_fused_diff_history) > 0:
            avg_amcl_fused = sum(self.amcl_fused_diff_history) / len(self.amcl_fused_diff_history)
            self.get_logger().info(f'📈 Average AMCL-Fused diff (last 10): {avg_amcl_fused:.4f} m')
        
        if len(self.aruco_fused_diff_history) > 0:
            avg_aruco_fused = sum(self.aruco_fused_diff_history) / len(self.aruco_fused_diff_history)
            self.get_logger().info(f'📈 Average ArUco-Fused diff (last 10): {avg_aruco_fused:.4f} m')
        
        # Publish visualization markers
        self.publish_markers()
        
        self.get_logger().info('═' * 80)
    
    def publish_markers(self):
        """Publish visualization markers for RViz"""
        marker_array = MarkerArray()
        
        # AMCL marker (Red)
        if self.amcl_pose:
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'amcl'
            marker.id = 0
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose = self.amcl_pose
            marker.scale.x = 0.5
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)  # Red
            marker_array.markers.append(marker)
        
        # ArUco marker (Green)
        if self.aruco_pose and self.aruco_detected:
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'aruco'
            marker.id = 1
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose = self.aruco_pose
            marker.scale.x = 0.5
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)  # Green
            marker_array.markers.append(marker)
        
        # Fused marker (Blue)
        if self.fused_pose:
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'fused'
            marker.id = 2
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose = self.fused_pose
            marker.scale.x = 0.6
            marker.scale.y = 0.12
            marker.scale.z = 0.12
            marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.9)  # Blue
            marker_array.markers.append(marker)
        
        # Line between AMCL and ArUco (if both exist)
        if self.amcl_pose and self.aruco_pose and self.aruco_detected:
            line_marker = Marker()
            line_marker.header.frame_id = 'map'
            line_marker.header.stamp = self.get_clock().now().to_msg()
            line_marker.ns = 'diff_line'
            line_marker.id = 3
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            line_marker.scale.x = 0.02
            line_marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.6)  # Yellow
            line_marker.points.append(self.amcl_pose.position)
            line_marker.points.append(self.aruco_pose.position)
            marker_array.markers.append(line_marker)
        
        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = FusionMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
