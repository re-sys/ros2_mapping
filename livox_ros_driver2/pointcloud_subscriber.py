#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np


class PointCloudSubscriber(Node):
    def __init__(self):
        super().__init__('pointcloud_subscriber')
        
        # 创建订阅者，订阅livox/lidar话题
        self.subscription = self.create_subscription(
            PointCloud2,
            'livox/lidar',
            self.pointcloud_callback,
            10
        )
        
        self.get_logger().info('PointCloud订阅者已启动，正在监听 livox/lidar 话题...')

    def pointcloud_callback(self, msg):
        """
        处理接收到的点云数据
        """
        try:
            # 获取点云中的点数
            num_points = len(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))
            
            # 打印信息
            self.get_logger().info(f'接收到点云帧 - 时间戳: {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}, 点数: {num_points}')
            
            # 可选：打印更详细的信息
            # self.get_logger().info(f'帧ID: {msg.header.frame_id}, 序列号: {msg.header.stamp}')
            
        except Exception as e:
            self.get_logger().error(f'处理点云数据时出错: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    # 创建订阅者节点
    pointcloud_subscriber = PointCloudSubscriber()
    
    try:
        # 运行节点
        rclpy.spin(pointcloud_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        # 清理资源
        pointcloud_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 