#!/usr/bin/env python3
"""Simple node to throttle and republish LaserScan messages with refreshed timestamps.

Usage:
  python3 scan_throttle.py --input /scan_raw --output /scan --frequency 5.0

This is intentionally minimal and runs directly from the source tree when launched
from the launch file using python3.
"""
import argparse
import copy
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanThrottle(Node):
    def __init__(self, input_topic: str, output_topic: str, frequency: float):
        super().__init__('scan_throttle')
        self._last_msg = None
        self._pub = self.create_publisher(LaserScan, output_topic, 10)
        self._sub = self.create_subscription(LaserScan, input_topic, self._cb, 10)
        self._timer = self.create_timer(1.0 / max(0.1, frequency), self._on_timer)
        self.get_logger().info(f"scan_throttle: republishing {input_topic} -> {output_topic} @ {frequency} Hz")

    def _cb(self, msg: LaserScan):
        # store the last received scan
        self._last_msg = msg

    def _on_timer(self):
        if self._last_msg is None:
            return
        out = copy.deepcopy(self._last_msg)
        now = self.get_clock().now().to_msg()
        stamp = out.header.stamp
        # Convert to float seconds
        scan_time = stamp.sec + stamp.nanosec * 1e-9
        now_time = now.sec + now.nanosec * 1e-9
        # If scan is in the future, clamp to now
        if scan_time > now_time:
            out.header.stamp = now
        # If scan is too old (>0.5s before now), skip publish
        elif now_time - scan_time > 0.5:
            return
        # Otherwise publish as is
        self._pub.publish(out)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', default='/scan_raw', help='input LaserScan topic')
    parser.add_argument('--output', default='/scan', help='output LaserScan topic')
    parser.add_argument('--frequency', type=float, default=5.0, help='publish frequency (Hz)')
    # Accept and ignore unknown args (ros2 launch passes --ros-args and remapping args)
    args, _unknown = parser.parse_known_args()

    rclpy.init()
    node = ScanThrottle(args.input, args.output, args.frequency)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
