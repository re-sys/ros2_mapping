#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Int32
import numpy as np
import cv2
import struct
import math

class PointCloudProcessor(Node):
    """
    点云处理器节点
    订阅FAST-LIO发布的registered点云，进行空间滤波、投影和圆形检测
    将检测结果发布到ROS话题
    """
    
    def __init__(self):
        super().__init__('pointcloud_processor')
        
        # 订阅FAST-LIO发布的点云话题
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/cloud_registered',  # FAST-LIO通常发布到这个话题
            self.pointcloud_callback,
            10
        )
        
        # 发布检测结果
        self.circle_detected_pub = self.create_publisher(
            Bool,
            '/circle_detected',
            10
        )
        
        self.circle_center_pub = self.create_publisher(
            Point,
            '/circle_center',
            10
        )
        
        self.circle_radius_pub = self.create_publisher(
            Float32,
            '/circle_radius',
            10
        )
        
        # 发布投影图像（用于调试）
        self.projection_image_pub = self.create_publisher(
            Image,
            '/projection_image',
            10
        )
        
        # 发布积累进度
        self.accumulation_progress_pub = self.create_publisher(
            Int32,
            '/accumulation_progress',
            10
        )
        
        # 空间滤波参数
        self.x_range = (2.5, 3.5)      # x轴范围
        self.y_range = (-0.3, 0.9)     # y轴范围  
        self.z_range = (0.0, 2.0)       # z轴范围
        
        # 圆形检测参数
        self.min_radius = 0.1           # 最小半径
        self.max_radius = 0.4           # 最大半径
        self.inlier_threshold = 0.05    # 内点阈值
        self.min_inliers = 10           # 最少内点数量
        self.max_iterations = 1000      # RANSAC最大迭代次数
        
        # 投影参数
        self.projection_resolution = 0.01  # 投影分辨率 (m/pixel)
        self.projection_size = (600, 600)  # 投影图像大小
        
        # 点云积累参数
        self.accumulation_frames = 10   # 积累帧数
        self.accumulated_points = []    # 积累的点云
        self.frame_count = 0            # 当前帧数
        self.circle_detected = False    # 圆形检测标志
        
        self.get_logger().info('点云处理器节点已启动')
        self.get_logger().info(f'点云积累帧数: {self.accumulation_frames}')
        
    def pointcloud_callback(self, msg):
        """
        点云回调函数
        """
        # 如果已经检测到圆形，直接返回
        if self.circle_detected:
            return
            
        try:
            # 解析点云数据
            points = self.parse_pointcloud2(msg)
            if len(points) == 0:
                return
                
            # 空间滤波
            filtered_points = self.spatial_filter(points)
            if len(filtered_points) == 0:
                self.get_logger().debug('空间滤波后无点云数据')
                return
            
            # 积累点云
            self.accumulated_points.extend(filtered_points)
            self.frame_count += 1
            
            # 发布积累进度
            progress_msg = Int32()
            progress_msg.data = self.frame_count
            self.accumulation_progress_pub.publish(progress_msg)
            
            # 显示积累进度
            self.get_logger().info(f'点云积累进度: {self.frame_count}/{self.accumulation_frames}, 当前积累点数: {len(self.accumulated_points)}')
            
            # 检查是否达到积累帧数
            if self.frame_count < self.accumulation_frames:
                return
            
            # 转换为numpy数组
            accumulated_points_array = np.array(self.accumulated_points)
            
            # 投影到XY平面
            projection_image = self.project_to_xy_plane(accumulated_points_array)
            if projection_image is None:
                return
                
            # 发布投影图像
            self.publish_projection_image(projection_image)
                
            # 提取轮廓
            contours = self.extract_contours(projection_image)
            if not contours:
                self.get_logger().debug('未检测到轮廓')
                self.get_logger().info('未检测到圆形')
                # 发布未检测到圆形的消息
                detected_msg = Bool()
                detected_msg.data = False
                self.circle_detected_pub.publish(detected_msg)
                return
                
            # 圆形拟合
            best_circle = self.fit_circle_ransac(accumulated_points_array, contours)
            if best_circle is not None:
                center_x, center_y, center_z, radius = best_circle
                self.get_logger().info(f'检测到圆形: 中心({center_x:.3f}, {center_y:.3f}, {center_z:.3f}), 半径: {radius:.3f}m')
                
                # 发布检测结果
                self.publish_circle_detection(best_circle)
                
                # 标记已检测到圆形
                self.circle_detected = True
                
            else:
                self.get_logger().info('未检测到圆形')
                # 发布未检测到圆形的消息
                detected_msg = Bool()
                detected_msg.data = False
                self.circle_detected_pub.publish(detected_msg)
                
        except Exception as e:
            self.get_logger().error(f'处理点云时出错: {str(e)}')
    
    def parse_pointcloud2(self, msg):
        """
        解析PointCloud2消息
        """
        points = []
        
        # 获取点云字段信息
        fields = {field.name: field.offset for field in msg.fields}
        
        # 检查必要字段
        required_fields = ['x', 'y', 'z']
        for field in required_fields:
            if field not in fields:
                self.get_logger().error(f'点云缺少必要字段: {field}')
                return points
        
        # 解析点云数据
        point_step = msg.point_step
        data = msg.data
        
        for i in range(msg.width * msg.height):
            offset = i * point_step
            
            # 提取x, y, z坐标
            x = struct.unpack('f', data[offset + fields['x']:offset + fields['x'] + 4])[0]
            y = struct.unpack('f', data[offset + fields['y']:offset + fields['y'] + 4])[0]
            z = struct.unpack('f', data[offset + fields['z']:offset + fields['z'] + 4])[0]
            
            # 检查是否为有效点（非NaN和无穷大）
            if not (np.isnan(x) or np.isnan(y) or np.isnan(z) or 
                   np.isinf(x) or np.isinf(y) or np.isinf(z)):
                points.append([x, y, z])
        
        return np.array(points)
    
    def spatial_filter(self, points):
        """
        空间滤波：提取指定范围内的点云
        """
        if len(points) == 0:
            return np.array([])
        
        # 应用空间滤波
        mask = (
            (points[:, 0] >= self.x_range[0]) & (points[:, 0] <= self.x_range[1]) &
            (points[:, 1] >= self.y_range[0]) & (points[:, 1] <= self.y_range[1]) &
            (points[:, 2] >= self.z_range[0]) & (points[:, 2] <= self.z_range[1])
        )
        
        filtered_points = points[mask]
        self.get_logger().debug(f'空间滤波: {len(points)} -> {len(filtered_points)} 点')
        
        return filtered_points
    
    def project_to_xy_plane(self, points):
        """
        将点云投影到XY平面
        """
        if len(points) == 0:
            return None
        
        # 计算投影范围
        x_min, x_max = points[:, 0].min(), points[:, 0].max()
        y_min, y_max = points[:, 1].min(), points[:, 1].max()
        
        # 创建投影图像
        width = int((x_max - x_min) / self.projection_resolution) + 1
        height = int((y_max - y_min) / self.projection_resolution) + 1
        
        # 限制图像大小
        width = min(width, self.projection_size[0])
        height = min(height, self.projection_size[1])
        
        projection_image = np.zeros((height, width), dtype=np.uint8)
        
        # 投影点云
        for point in points:
            x, y = point[0], point[1]
            
            # 计算像素坐标
            pixel_x = int((x - x_min) / self.projection_resolution)
            pixel_y = int((y - y_min) / self.projection_resolution)
            
            # 检查边界
            if 0 <= pixel_x < width and 0 <= pixel_y < height:
                projection_image[pixel_y, pixel_x] = 255
        
        return projection_image
    
    def extract_contours(self, image):
        """
        提取轮廓
        """
        # 形态学操作，连接断开的点
        kernel = np.ones((3, 3), np.uint8)
        image = cv2.morphologyEx(image, cv2.MORPH_CLOSE, kernel)
        image = cv2.morphologyEx(image, cv2.MORPH_OPEN, kernel)
        
        # 查找轮廓
        contours, _ = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # 过滤小轮廓
        min_contour_area = 50  # 最小轮廓面积
        filtered_contours = []
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > min_contour_area:
                filtered_contours.append(contour)
        
        return filtered_contours
    
    def fit_circle_ransac(self, points, contours):
        """
        使用RANSAC拟合圆形
        """
        if not contours:
            return None
        
        best_circle = None
        best_inliers = 0
        
        # 将轮廓点转换为原始坐标
        contour_points = []
        for contour in contours:
            for point in contour:
                # 将像素坐标转换回世界坐标
                x_pixel, y_pixel = point[0]
                # 这里需要根据投影参数进行逆变换
                # 简化处理：直接使用原始点云中的点
                contour_points.append([x_pixel, y_pixel])
        
        # 如果没有足够的轮廓点，使用原始点云
        if len(contour_points) < 3:
            contour_points = points[:, :2].tolist()
        
        contour_points = np.array(contour_points)
        
        if len(contour_points) < 3:
            return None
        
        # RANSAC圆形拟合
        for iteration in range(self.max_iterations):
            # 随机选择3个点
            if len(contour_points) < 3:
                break
                
            indices = np.random.choice(len(contour_points), 3, replace=False)
            p1, p2, p3 = contour_points[indices]
            
            # 计算圆心和半径
            circle = self.calculate_circle_from_three_points(p1, p2, p3)
            if circle is None:
                continue
                
            center_x, center_y, radius = circle
            
            # 检查半径是否在合理范围内
            if radius < self.min_radius or radius > self.max_radius:
                continue
            
            # 计算内点数量
            inliers = 0
            for point in contour_points:
                distance = np.sqrt((point[0] - center_x)**2 + (point[1] - center_y)**2)
                if abs(distance - radius) <= self.inlier_threshold:
                    inliers += 1
            
            # 更新最佳圆形
            if inliers > best_inliers and inliers >= self.min_inliers:
                best_inliers = inliers
                # 计算圆心的z坐标（使用原始点云中最近点的z坐标）
                center_z = self.get_center_z_coordinate(points, center_x, center_y)
                best_circle = (center_x, center_y, center_z, radius)
        
        return best_circle
    
    def calculate_circle_from_three_points(self, p1, p2, p3):
        """
        根据三个点计算圆心和半径
        """
        try:
            # 计算三个点的坐标
            x1, y1 = p1[0], p1[1]
            x2, y2 = p2[0], p2[1]
            x3, y3 = p3[0], p3[1]
            
            # 检查三点是否共线
            if abs((y2 - y1) * (x3 - x2) - (y3 - y2) * (x2 - x1)) < 1e-10:
                return None
            
            # 计算圆心
            # 使用垂直平分线的交点
            # 中点1
            mx1 = (x1 + x2) / 2
            my1 = (y1 + y2) / 2
            # 中点2
            mx2 = (x2 + x3) / 2
            my2 = (y2 + y3) / 2
            
            # 方向向量1
            dx1 = x2 - x1
            dy1 = y2 - y1
            # 方向向量2
            dx2 = x3 - x2
            dy2 = y3 - y2
            
            # 垂直向量1
            vx1 = -dy1
            vy1 = dx1
            # 垂直向量2
            vx2 = -dy2
            vy2 = dx2
            
            # 求解线性方程组
            # (mx1 + t1 * vx1) = (mx2 + t2 * vx2)
            # (my1 + t1 * vy1) = (my2 + t2 * vy2)
            
            # 使用克莱默法则求解
            det = vx1 * vy2 - vx2 * vy1
            if abs(det) < 1e-10:
                return None
                
            t1 = ((mx2 - mx1) * vy2 - (my2 - my1) * vx2) / det
            
            # 圆心坐标
            center_x = mx1 + t1 * vx1
            center_y = my1 + t1 * vy1
            
            # 计算半径
            radius = np.sqrt((x1 - center_x)**2 + (y1 - center_y)**2)
            
            return center_x, center_y, radius
            
        except Exception as e:
            self.get_logger().debug(f'计算圆形时出错: {str(e)}')
            return None
    
    def get_center_z_coordinate(self, points, center_x, center_y):
        """
        获取圆心对应的z坐标
        """
        if len(points) == 0:
            return 0.0
        
        # 找到距离圆心最近的点
        distances = np.sqrt((points[:, 0] - center_x)**2 + (points[:, 1] - center_y)**2)
        nearest_idx = np.argmin(distances)
        
        return points[nearest_idx, 2]
    
    def publish_circle_detection(self, circle_info):
        """
        发布圆形检测结果
        """
        center_x, center_y, center_z, radius = circle_info
        
        # 发布检测标志
        detected_msg = Bool()
        detected_msg.data = True
        self.circle_detected_pub.publish(detected_msg)
        
        # 发布圆心坐标
        center_msg = Point()
        center_msg.x = center_x
        center_msg.y = center_y
        center_msg.z = center_z
        self.circle_center_pub.publish(center_msg)
        
        # 发布半径
        radius_msg = Float32()
        radius_msg.data = radius
        self.circle_radius_pub.publish(radius_msg)
    
    def publish_projection_image(self, projection_image):
        """
        发布投影图像
        """
        # 转换为ROS Image消息
        height, width = projection_image.shape
        image_msg = Image()
        image_msg.height = height
        image_msg.width = width
        image_msg.encoding = 'mono8'
        image_msg.step = width
        image_msg.data = projection_image.tobytes()
        
        self.projection_image_pub.publish(image_msg)

def main(args=None):
    rclpy.init(args=args)
    
    node = PointCloudProcessor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 