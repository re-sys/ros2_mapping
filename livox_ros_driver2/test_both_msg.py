#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from livox_ros_driver2.msg import CustomMsg
import time

class LivoxTestSubscriber(Node):
    def __init__(self):
        super().__init__('livox_test_subscriber')
        
        # 订阅PointCloud2消息
        self.pointcloud2_sub = self.create_subscription(
            PointCloud2,
            'livox/lidar',
            self.pointcloud2_callback,
            10
        )
        
        # 订阅自定义消息
        self.custom_sub = self.create_subscription(
            CustomMsg,
            'livox/lidar_custom',
            self.custom_callback,
            10
        )
        
        self.pointcloud2_count = 0
        self.custom_count = 0
        self.start_time = time.time()
        
        # 创建定时器，每秒打印统计信息
        self.timer = self.create_timer(1.0, self.print_stats)
        
        self.get_logger().info('Livox test subscriber started. Waiting for messages...')
    
    def pointcloud2_callback(self, msg):
        self.pointcloud2_count += 1
        self.get_logger().info(f'Received PointCloud2 #{self.pointcloud2_count}: {len(msg.data)} bytes, {msg.width} points')
    
    def custom_callback(self, msg):
        self.custom_count += 1
        self.get_logger().info(f'Received CustomMsg #{self.custom_count}: {msg.point_num} points, lidar_id: {msg.lidar_id}')
    
    def print_stats(self):
        elapsed = time.time() - self.start_time
        self.get_logger().info(f'Stats after {elapsed:.1f}s: PointCloud2={self.pointcloud2_count}, CustomMsg={self.custom_count}')

def main():
    rclpy.init()
    node = LivoxTestSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Test interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 