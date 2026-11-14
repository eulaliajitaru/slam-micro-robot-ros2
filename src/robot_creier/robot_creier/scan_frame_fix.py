#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from copy import deepcopy

class ScanFrameFix(Node):
    def __init__(self):
        super().__init__('scan_frame_fix')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pub = self.create_publisher(LaserScan, '/scan_fixed', 10)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

    def scan_callback(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'laser_horizontal'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.16
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)
        fixed = deepcopy(msg)
        fixed.header.frame_id = 'laser_horizontal'
        self.pub.publish(fixed)

def main():
    rclpy.init()
    rclpy.spin(ScanFrameFix())

if __name__ == '__main__':
    main()
