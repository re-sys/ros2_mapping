#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import struct
from sensor_msgs_py import point_cloud2

class PointCloudFilter(Node):
    """
    点云滤波器节点
    订阅FAST-LIO发布的cloud_registered点云
    进行z轴空间滤波（1.4m到1.6m）
    发布过滤后的点云
    """
    
    def __init__(self):
        super().__init__('pointcloud_filter')
        
        # 滤波参数
        self.z_min = 1.4  # z轴最小值 (米)
        self.z_max = 1.2  # z轴最大值 (米)
        
        # 订阅FAST-LIO的点云话题
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/cloud_registered',  # FAST-LIO发布的点云话题
            self.pointcloud_callback,
            10
        )
        
        # 发布过滤后的点云话题
        self.filtered_pub = self.create_publisher(
            PointCloud2,
            '/cloud_filtered',  # 过滤后的点云话题
            10
        )
        
        self.get_logger().info(f'PointCloud Filter initialized')
        self.get_logger().info(f'Z-axis filter range: {self.z_min}m to {self.z_max}m')
        self.get_logger().info(f'Subscribing to: /cloud_registered')
        self.get_logger().info(f'Publishing to: /cloud_filtered')
        
        # 统计信息
        self.total_points = 0
        self.filtered_points = 0
        self.callback_count = 0
        
    def pointcloud_callback(self, msg):
        """
        处理接收到的点云消息
        进行z轴空间滤波
        """
        try:
            # 将PointCloud2消息转换为numpy数组
            points = point_cloud2.read_points(
                msg, 
                field_names=("x", "y", "z"), 
                skip_nans=True
            )
            
            # 转换为numpy数组
            points_array = np.array(list(points))
            
            if len(points_array) == 0:
                self.get_logger().warn('Received empty point cloud')
                return
            
            # 进行z轴空间滤波
            z_mask = (points_array[:, 2] >= self.z_min) & (points_array[:, 2] <= self.z_max)
            filtered_points = points_array[z_mask]
            
            # 更新统计信息
            self.total_points += len(points_array)
            self.filtered_points += len(filtered_points)
            self.callback_count += 1
            
            # 创建过滤后的PointCloud2消息
            filtered_msg = PointCloud2()
            filtered_msg.header = msg.header
            filtered_msg.header.frame_id = msg.header.frame_id
            
            # 设置点云字段
            filtered_msg.fields = msg.fields
            filtered_msg.point_step = msg.point_step
            filtered_msg.row_step = len(filtered_points) * msg.point_step
            filtered_msg.is_bigendian = msg.is_bigendian
            filtered_msg.is_dense = msg.is_dense
            
            # 将过滤后的点云数据打包
            filtered_data = []
            for point in filtered_points:
                # 根据原始消息的字段格式打包数据
                point_data = struct.pack('<fff', point[0], point[1], point[2])
                filtered_data.extend(point_data)
            
            filtered_msg.data = bytes(filtered_data)
            
            # 发布过滤后的点云
            self.filtered_pub.publish(filtered_msg)
            
            # 每10次回调打印一次统计信息
            if self.callback_count % 10 == 0:
                filter_ratio = len(filtered_points) / len(points_array) * 100 if len(points_array) > 0 else 0
                self.get_logger().info(
                    f'Callback {self.callback_count}: '
                    f'Original points: {len(points_array)}, '
                    f'Filtered points: {len(filtered_points)}, '
                    f'Filter ratio: {filter_ratio:.1f}%'
                )
                
        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {str(e)}')
    
    def set_filter_range(self, z_min, z_max):
        """
        设置z轴滤波范围
        
        Args:
            z_min: z轴最小值 (米)
            z_max: z轴最大值 (米)
        """
        if z_min >= z_max:
            self.get_logger().error('z_min must be less than z_max')
            return
            
        self.z_min = z_min
        self.z_max = z_max
        self.get_logger().info(f'Filter range updated: {self.z_min}m to {self.z_max}m')
    
    def get_statistics(self):
        """
        获取统计信息
        
        Returns:
            dict: 包含统计信息的字典
        """
        avg_filter_ratio = (self.filtered_points / self.total_points * 100) if self.total_points > 0 else 0
        return {
            'total_points': self.total_points,
            'filtered_points': self.filtered_points,
            'callback_count': self.callback_count,
            'average_filter_ratio': avg_filter_ratio
        }


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudFilter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 打印最终统计信息
        stats = node.get_statistics()
        node.get_logger().info('Final statistics:')
        node.get_logger().info(f'  Total points processed: {stats["total_points"]}')
        node.get_logger().info(f'  Total points filtered: {stats["filtered_points"]}')
        node.get_logger().info(f'  Total callbacks: {stats["callback_count"]}')
        node.get_logger().info(f'  Average filter ratio: {stats["average_filter_ratio"]:.1f}%')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
