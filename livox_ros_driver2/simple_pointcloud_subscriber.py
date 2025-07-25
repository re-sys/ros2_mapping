#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


class SimplePointCloudSubscriber(Node):
    def __init__(self):
        super().__init__('simple_pointcloud_subscriber')
        
        # 订阅livox/lidar话题
        self.subscription = self.create_subscription(
            PointCloud2,
            'livox/lidar',
            self.callback,
            10
        )
        
        print("开始监听 livox/lidar 话题...")

    def callback(self, msg):
        # 计算点数
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        num_points = len(points)
        
        # 简单打印点数
        print(f"点数: {num_points}")


def main():
    rclpy.init()
    node = SimplePointCloudSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 