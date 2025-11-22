#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan

class ScanThrottle(Node):
    def __init__(self):
        super().__init__('scan_throttle')
        
        # QoS profile for sensor data (BEST_EFFORT) - compatible with AMCL
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.pub = self.create_publisher(LaserScan, '/scan', qos_profile=sensor_qos)
        self.sub = self.create_subscription(LaserScan, '/scan_raw', self.cb, qos_profile=sensor_qos)
        self.last_pub = self.get_clock().now()
        self.period = 1.0 / 5.0  # 5Hz

    def cb(self, msg):
        now = self.get_clock().now()
        if (now - self.last_pub).nanoseconds / 1e9 >= self.period:
            self.pub.publish(msg)
            self.last_pub = now

def main(args=None):
    rclpy.init(args=args)
    node = ScanThrottle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
