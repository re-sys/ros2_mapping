#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA
import math

class ClusterVisualizationNode(Node):
    def __init__(self):
        super().__init__('cluster_visualization_node')
        
        # 订阅最近点
        self.closest_point_sub = self.create_subscription(
            PointStamped,
            '/closest_point',
            self.closest_point_callback,
            10
        )
        
        # 发布可视化标记
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/cluster_visualization_markers',
            10
        )
        
        self.get_logger().info('Cluster Visualization Node initialized')
        
    def closest_point_callback(self, msg):
        """处理最近点消息并创建可视化标记"""
        marker_array = MarkerArray()
        
        # 设置lifetime
        marker_lifetime = rclpy.duration.Duration(seconds=5.0).to_msg()
        frame_id = 'livox_frame'
        
        # 创建最近点的标记 - 使用更大的红色球体
        closest_marker = Marker()
        closest_marker.header.frame_id = frame_id
        closest_marker.header.stamp = msg.header.stamp
        closest_marker.ns = "closest_point"
        closest_marker.id = 0
        closest_marker.type = Marker.SPHERE
        closest_marker.action = Marker.ADD
        
        closest_marker.pose.position.x = msg.point.x
        closest_marker.pose.position.y = msg.point.y
        closest_marker.pose.position.z = msg.point.z
        closest_marker.pose.orientation.w = 1.0
        
        # 设置更大的大小
        closest_marker.scale.x = 1.0
        closest_marker.scale.y = 1.0
        closest_marker.scale.z = 1.0
        
        # 鲜艳的红色
        closest_marker.color.r = 1.0
        closest_marker.color.g = 0.0
        closest_marker.color.b = 0.0
        closest_marker.color.a = 1.0
        
        closest_marker.lifetime = marker_lifetime
        marker_array.markers.append(closest_marker)
        
        # 创建距离文本标记
        distance = math.sqrt(msg.point.x**2 + msg.point.y**2 + msg.point.z**2)
        text_marker = Marker()
        text_marker.header.frame_id = frame_id
        text_marker.header.stamp = msg.header.stamp
        text_marker.ns = "distance_text"
        text_marker.id = 1
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        
        text_marker.pose.position.x = msg.point.x
        text_marker.pose.position.y = msg.point.y
        text_marker.pose.position.z = msg.point.z + 1.0
        text_marker.pose.orientation.w = 1.0
        
        text_marker.scale.z = 0.5  # 更大的文本
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 0.0  # 黄色文本
        text_marker.color.a = 1.0
        
        text_marker.text = f"CLOSEST: {distance:.2f}m"
        text_marker.lifetime = marker_lifetime
        marker_array.markers.append(text_marker)
        
        # 创建警告框标记
        warning_marker = Marker()
        warning_marker.header.frame_id = frame_id
        warning_marker.header.stamp = msg.header.stamp
        warning_marker.ns = "warning_box"
        warning_marker.id = 2
        warning_marker.type = Marker.CUBE
        warning_marker.action = Marker.ADD
        
        warning_marker.pose.position.x = msg.point.x
        warning_marker.pose.position.y = msg.point.y
        warning_marker.pose.position.z = msg.point.z
        warning_marker.pose.orientation.w = 1.0
        
        # 设置立方体大小
        warning_marker.scale.x = 1.5
        warning_marker.scale.y = 1.5
        warning_marker.scale.z = 1.5
        
        # 闪烁的橙色
        warning_marker.color.r = 1.0
        warning_marker.color.g = 0.5
        warning_marker.color.b = 0.0
        warning_marker.color.a = 0.7
        
        warning_marker.lifetime = marker_lifetime
        marker_array.markers.append(warning_marker)
        
        # 发布标记
        self.marker_pub.publish(marker_array)
        
        self.get_logger().info(f'Closest point: ({msg.point.x:.2f}, {msg.point.y:.2f}, {msg.point.z:.2f}), Distance: {distance:.2f}m')

def main(args=None):
    rclpy.init(args=args)
    node = ClusterVisualizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 