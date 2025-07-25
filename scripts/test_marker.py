#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker

class TestMarker(Node):
    def __init__(self):
        super().__init__('test_marker')
        self.pub = self.create_publisher(Marker, '/test_marker', 1)
        marker = Marker()
        marker.header.frame_id = 'livox_frame'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.scale.x = 2.0
        marker.scale.y = 2.0
        marker.scale.z = 2.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.id = 0
        marker.lifetime.sec = 0
        self.pub.publish(marker)
        self.get_logger().info('Published test marker!')

rclpy.init()
node = TestMarker()
rclpy.spin_once(node, timeout_sec=1.0) 