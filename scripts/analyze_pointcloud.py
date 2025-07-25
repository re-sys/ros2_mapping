#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import struct

class PointCloudAnalyzer(Node):
    def __init__(self):
        super().__init__('point_cloud_analyzer')
        
        # 订阅点云话题
        self.point_cloud_sub = self.create_subscription(
            PointCloud2,
            '/livox/lidar',
            self.point_cloud_callback,
            10
        )
        
        self.get_logger().info('Point Cloud Analyzer initialized')
        
    def point_cloud_callback(self, msg):
        """分析点云数据"""
        # 解析点云数据
        points = self.parse_pointcloud2(msg)
        
        if len(points) == 0:
            self.get_logger().warn('Empty point cloud received')
            return
            
        # 提取坐标
        x_coords = points[:, 0]
        y_coords = points[:, 1]
        z_coords = points[:, 2]
        
        # 分析Z轴分布
        z_min = np.min(z_coords)
        z_max = np.max(z_coords)
        z_mean = np.mean(z_coords)
        z_std = np.std(z_coords)
        
        # 分析XY平面分布
        xy_distances = np.sqrt(x_coords**2 + y_coords**2)
        max_distance = np.max(xy_distances)
        
        # 统计不同高度范围的点数
        z_ranges = [
            (-np.inf, -0.5, "Below -0.5m"),
            (-0.5, 0.0, "-0.5m to 0m"),
            (0.0, 0.5, "0m to 0.5m"),
            (0.5, 1.0, "0.5m to 1.0m"),
            (1.0, np.inf, "Above 1.0m")
        ]
        
        self.get_logger().info("=== Point Cloud Analysis ===")
        self.get_logger().info(f"Total points: {len(points)}")
        self.get_logger().info(f"Z-axis range: {z_min:.3f}m to {z_max:.3f}m")
        self.get_logger().info(f"Z-axis mean: {z_mean:.3f}m ± {z_std:.3f}m")
        self.get_logger().info(f"Max XY distance: {max_distance:.3f}m")
        
        self.get_logger().info("Z-axis distribution:")
        for z_min_range, z_max_range, label in z_ranges:
            count = np.sum((z_coords >= z_min_range) & (z_coords < z_max_range))
            percentage = (count / len(points)) * 100
            self.get_logger().info(f"  {label}: {count} points ({percentage:.1f}%)")
        
        # 检查是否有强度信息
        if msg.fields:
            intensity_field = None
            for field in msg.fields:
                if field.name == 'intensity':
                    intensity_field = field
                    break
            
            if intensity_field:
                self.get_logger().info("Point cloud contains intensity information")
            else:
                self.get_logger().info("Point cloud does NOT contain intensity information")
        
        self.get_logger().info("==========================")
        
    def parse_pointcloud2(self, msg):
        """解析PointCloud2消息"""
        # 获取点云格式信息
        point_step = msg.point_step
        row_step = msg.row_step
        
        # 解析字段信息
        fields = {}
        for field in msg.fields:
            fields[field.name] = {
                'offset': field.offset,
                'datatype': field.datatype,
                'count': field.count
            }
        
        # 检查是否有必要的字段
        if 'x' not in fields or 'y' not in fields or 'z' not in fields:
            self.get_logger().error('Point cloud missing x, y, or z fields')
            return np.array([])
        
        # 解析点云数据
        points = []
        for i in range(0, len(msg.data), point_step):
            point_data = msg.data[i:i+point_step]
            
            # 提取x, y, z坐标
            x = struct.unpack('f', point_data[fields['x']['offset']:fields['x']['offset']+4])[0]
            y = struct.unpack('f', point_data[fields['y']['offset']:fields['y']['offset']+4])[0]
            z = struct.unpack('f', point_data[fields['z']['offset']:fields['z']['offset']+4])[0]
            
            # 过滤无效点（NaN或无穷大）
            if not (np.isnan(x) or np.isnan(y) or np.isnan(z) or 
                   np.isinf(x) or np.isinf(y) or np.isinf(z)):
                points.append([x, y, z])
        
        return np.array(points)

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudAnalyzer()
    
    # 只运行一次分析
    rclpy.spin_once(node, timeout_sec=5.0)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 