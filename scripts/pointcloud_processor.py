#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import numpy as np
import open3d as o3d
from std_msgs.msg import Header

class PointCloudProcessor(Node):
    def __init__(self):
        super().__init__('pointcloud_processor')
        
        # 订阅FAST_LIO的点云话题
        self.sub = self.create_subscription(
            PointCloud2,
            '/Laser_map',  # FAST_LIO的默认点云话题
            self.pointcloud_callback,
            10
        )
        
        # 发布处理后的点云
        self.pub = self.create_publisher(
            PointCloud2,
            '/filtered_pointcloud',
            10
        )
        
        # 定义空间范围 (单位：米)
        self.x_range = (-5.0, 5.0)    # X轴范围
        self.y_range = (-5.0, 5.0)    # Y轴范围
        self.z_range = (-1.0, 2.0)    # Z轴范围
        
        # 体素下采样大小
        self.voxel_size = 0.05
        
        self.get_logger().info('PointCloud Processor initialized')
        self.get_logger().info(f'Filtering range: X={self.x_range}, Y={self.y_range}, Z={self.z_range}')
        self.get_logger().info(f'Voxel size: {self.voxel_size} m')
    
    def pointcloud_callback(self, msg):
        """处理点云数据"""
        # 将PointCloud2转换为NumPy数组
        points = point_cloud2.read_points_numpy(msg)
        
        # 1. 体素下采样
        downsampled_points = self.voxel_downsample(points)
        
        # 2. 空间范围过滤
        filtered_points = self.spatial_filter(downsampled_points)
        
        # 发布处理后的点云
        self.publish_pointcloud(filtered_points, msg.header)
        
        # 打印点云信息
        self.get_logger().info(f'Original: {len(points)} points | '
                               f'Downsampled: {len(downsampled_points)} points | '
                               f'Filtered: {len(filtered_points)} points',
                               throttle_duration_sec=1.0)
    
    def voxel_downsample(self, points):
        """体素下采样"""
        # 创建Open3D点云对象
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points[['x', 'y', 'z']])
        
        # 执行体素下采样
        downpcd = pcd.voxel_down_sample(voxel_size=self.voxel_size)
        
        # 返回下采样后的点
        return np.asarray(downpcd.points)
    
    def spatial_filter(self, points):
        """空间范围过滤"""
        # 提取坐标
        x = points[:, 0]
        y = points[:, 1]
        z = points[:, 2]
        
        # 创建空间范围掩码
        mask = (
            (x >= self.x_range[0]) & (x <= self.x_range[1]) &
            (y >= self.y_range[0]) & (y <= self.y_range[1]) &
            (z >= self.z_range[0]) & (z <= self.z_range[1])
        )
        
        # 返回过滤后的点
        return points[mask]
    
    def publish_pointcloud(self, points, original_header):
        """发布处理后的点云"""
        # 创建PointCloud2消息头
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = original_header.frame_id  # 使用原始坐标系
        
        # 定义点云字段
        fields = [
            point_cloud2.PointField(name='x', offset=0, datatype=point_cloud2.PointField.FLOAT32, count=1),
            point_cloud2.PointField(name='y', offset=4, datatype=point_cloud2.PointField.FLOAT32, count=1),
            point_cloud2.PointField(name='z', offset=8, datatype=point_cloud2.PointField.FLOAT32, count=1)
        ]
        
        # 创建并发布点云
        pc2_msg = point_cloud2.create_cloud(header, fields, points)
        self.pub.publish(pc2_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
