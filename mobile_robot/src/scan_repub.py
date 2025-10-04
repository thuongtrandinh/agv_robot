#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanRepublisher(Node):
    def __init__(self):
        super().__init__('scan_republisher')
        
        # Subscribe to the original scan topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Publisher for the fixed scan topic
        self.publisher = self.create_publisher(
            LaserScan,
            '/scan_fixed',
            10
        )
        
        self.get_logger().info('Scan republisher node started')
        self.get_logger().info('Subscribing to: /scan')
        self.get_logger().info('Publishing to: /scan_fixed')

    def scan_callback(self, msg):
        # Create a new message with the corrected frame_id
        fixed_msg = LaserScan()
        
        # Copy all data from the original message
        fixed_msg.header = msg.header
        fixed_msg.angle_min = msg.angle_min
        fixed_msg.angle_max = msg.angle_max
        fixed_msg.angle_increment = msg.angle_increment
        fixed_msg.time_increment = msg.time_increment
        fixed_msg.scan_time = msg.scan_time
        fixed_msg.range_min = msg.range_min
        fixed_msg.range_max = msg.range_max
        fixed_msg.ranges = msg.ranges
        fixed_msg.intensities = msg.intensities
        
        # Fix the frame_id
        fixed_msg.header.frame_id = "laser"
        
        # Publish the fixed message
        self.publisher.publish(fixed_msg)

def main(args=None):
    rclpy.init(args=args)
    
    scan_republisher = ScanRepublisher()
    
    try:
        rclpy.spin(scan_republisher)
    except KeyboardInterrupt:
        pass
    
    scan_republisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
