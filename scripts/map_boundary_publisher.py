#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA
import math

class MapBoundaryPublisher(Node):
    """
    Map边界发布器
    基于initialpose的位置，绘制长方形边界
    """
    
    def __init__(self):
        super().__init__('map_boundary_publisher')
        
        # 订阅initialpose话题
        self.initialpose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initialpose_callback,
            10
        )
        
        # 发布map边界消息
        self.boundary_marker_pub = self.create_publisher(
            MarkerArray,
            '/map_boundary_markers',
            10
        )
        
        # 发布边界点数组
        self.boundary_poses_pub = self.create_publisher(
            PoseArray,
            '/map_boundary_poses',
            10
        )
        
        # 边界参数
        self.y_length = 15.0  # Y轴方向长边：-7.5到7.5m
        self.x_length = 8.0   # X轴方向短边：0到8m
        self.arc_radius = 2.0  # 半圆弧半径：2m
        
        # 初始化标志
        
        self.get_logger().info('Map边界发布器已启动')
        self.get_logger().info(f'边界尺寸: Y轴长边 {self.y_length}m, X轴短边 {self.x_length}m')
    
    def initialpose_callback(self, msg):
        """处理initialpose消息，创建边界"""
        # 提取initialpose的位置和姿态
        initial_x = msg.pose.pose.position.x
        initial_y = msg.pose.pose.position.y
        initial_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
        
        self.get_logger().info(f'收到initialpose: x={initial_x:.3f}, y={initial_y:.3f}, yaw={math.degrees(initial_yaw):.2f}°')
        
        # 创建边界
        self.create_boundary(initial_x, initial_y, initial_yaw)
        
        # 标记已创建边界
        self.boundary_created = True
        
        self.get_logger().info('Map边界已更新并发布')
    
    def create_boundary(self, center_x, center_y, center_yaw):
        """创建长方形边界"""
        # 计算边界四个角点的坐标（相对于initialpose）
        # 边界以initialpose为中心，Y轴方向为长边，X轴正方向为短边
        
        # 边界角点（相对于initialpose的局部坐标）
        local_corners = [
            (0, -self.y_length/2),      # 左下角
            (self.x_length, -self.y_length/2),  # 右下角
            (self.x_length, self.y_length/2),   # 右上角
            (0, self.y_length/2)        # 左上角
        ]
        
        # 将局部坐标转换为世界坐标
        world_corners = []
        for local_x, local_y in local_corners:
            world_x, world_y = self.transform_local_to_world(local_x, local_y, center_x, center_y, center_yaw)
            world_corners.append((world_x, world_y))
        
        # 计算半圆弧点
        arc_points = self.calculate_arc_points(center_x, center_y, center_yaw)
        
        # 发布边界标记
        self.publish_boundary_markers(world_corners, arc_points)
        
        # 发布边界点数组
        self.publish_boundary_poses(world_corners)
        
        # 打印边界信息
        self.get_logger().info('边界角点坐标:')
        for i, (x, y) in enumerate(world_corners):
            self.get_logger().info(f'  角点{i+1}: ({x:.3f}, {y:.3f})')
        
        self.get_logger().info(f'半圆弧半径: {self.arc_radius}m')
    
    def transform_local_to_world(self, local_x, local_y, center_x, center_y, center_yaw):
        """将局部坐标转换为世界坐标"""
        # 旋转
        cos_yaw = math.cos(center_yaw)
        sin_yaw = math.sin(center_yaw)
        
        rotated_x = local_x * cos_yaw - local_y * sin_yaw
        rotated_y = local_x * sin_yaw + local_y * cos_yaw
        
        # 平移
        world_x = rotated_x + center_x
        world_y = rotated_y + center_y
        
        return world_x, world_y
    
    def calculate_arc_points(self, center_x, center_y, center_yaw):
        """计算半圆弧的点"""
        arc_points = []
        
        # 半圆弧的角度范围：从-y到y，即从-90度到90度
        start_angle = -math.pi/2  # -90度
        end_angle = math.pi/2     # 90度
        num_points = 50  # 圆弧上的点数
        
        for i in range(num_points + 1):
            # 计算当前角度
            angle = start_angle + (end_angle - start_angle) * i / num_points
            
            # 计算圆弧上的局部坐标
            local_x = self.arc_radius * math.cos(angle)
            local_y = self.arc_radius * math.sin(angle)
            
            # 转换为世界坐标
            world_x, world_y = self.transform_local_to_world(local_x, local_y, center_x, center_y, center_yaw)
            arc_points.append((world_x, world_y))
        
        return arc_points
    
    def publish_boundary_markers(self, corners, arc_points):
        """发布边界标记"""
        marker_array = MarkerArray()
        
        # 首先删除旧的边界标记
        delete_marker = Marker()
        delete_marker.header.frame_id = "camera_init"
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        delete_marker.ns = "map_boundary"
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        delete_corners_marker = Marker()
        delete_corners_marker.header.frame_id = "camera_init"
        delete_corners_marker.header.stamp = self.get_clock().now().to_msg()
        delete_corners_marker.ns = "map_boundary_corners"
        delete_corners_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_corners_marker)
        
        delete_arc_marker = Marker()
        delete_arc_marker.header.frame_id = "camera_init"
        delete_arc_marker.header.stamp = self.get_clock().now().to_msg()
        delete_arc_marker.ns = "map_boundary_arc"
        delete_arc_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_arc_marker)
        
        # 创建边界线标记
        line_marker = Marker()
        line_marker.header.frame_id = "camera_init"
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.ns = "map_boundary"
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        
        # 设置边界线的几何形状
        line_marker.points = []
        for x, y in corners:
            from geometry_msgs.msg import Point
            point = Point()
            point.x = x
            point.y = y
            point.z = 0.0
            line_marker.points.append(point)
        
        # 闭合边界线
        if len(corners) > 0:
            point = Point()
            point.x = corners[0][0]
            point.y = corners[0][1]
            point.z = 0.0
            line_marker.points.append(point)
        
        # 设置边界线的样式
        line_marker.scale.x = 0.1  # 线宽
        line_marker.color = ColorRGBA()
        line_marker.color.r = 1.0
        line_marker.color.g = 0.0
        line_marker.color.b = 0.0
        line_marker.color.a = 1.0
        
        marker_array.markers.append(line_marker)
        
        # 创建角点标记
        for i, (x, y) in enumerate(corners):
            corner_marker = Marker()
            corner_marker.header.frame_id = "camera_init"
            corner_marker.header.stamp = self.get_clock().now().to_msg()
            corner_marker.ns = "map_boundary_corners"
            corner_marker.id = i
            corner_marker.type = Marker.SPHERE
            corner_marker.action = Marker.ADD
            
            corner_marker.pose.position.x = x
            corner_marker.pose.position.y = y
            corner_marker.pose.position.z = 0.0
            corner_marker.pose.orientation.w = 1.0
            
            corner_marker.scale.x = 0.2
            corner_marker.scale.y = 0.2
            corner_marker.scale.z = 0.2
            
            corner_marker.color = ColorRGBA()
            corner_marker.color.r = 0.0
            corner_marker.color.g = 1.0
            corner_marker.color.b = 0.0
            corner_marker.color.a = 1.0
            
            marker_array.markers.append(corner_marker)
        
        # 创建半圆弧标记
        arc_marker = Marker()
        arc_marker.header.frame_id = "camera_init"
        arc_marker.header.stamp = self.get_clock().now().to_msg()
        arc_marker.ns = "map_boundary_arc"
        arc_marker.id = 0
        arc_marker.type = Marker.LINE_STRIP
        arc_marker.action = Marker.ADD
        
        # 设置半圆弧的几何形状
        arc_marker.points = []
        for x, y in arc_points:
            from geometry_msgs.msg import Point
            point = Point()
            point.x = x
            point.y = y
            point.z = 0.0
            arc_marker.points.append(point)
        
        # 设置半圆弧的样式
        arc_marker.scale.x = 0.1  # 线宽
        arc_marker.color = ColorRGBA()
        arc_marker.color.r = 0.0
        arc_marker.color.g = 0.0
        arc_marker.color.b = 1.0  # 蓝色
        arc_marker.color.a = 1.0
        
        marker_array.markers.append(arc_marker)
        
        # 发布标记
        self.boundary_marker_pub.publish(marker_array)
    
    def publish_boundary_poses(self, corners):
        """发布边界点数组"""
        pose_array = PoseArray()
        pose_array.header.frame_id = "camera_init"
        pose_array.header.stamp = self.get_clock().now().to_msg()
        
        for x, y in corners:
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)
        
        self.boundary_poses_pub.publish(pose_array)
    
    def quaternion_to_yaw(self, quaternion):
        """从四元数提取yaw角"""
        siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

def main(args=None):
    rclpy.init(args=args)
    node = MapBoundaryPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 