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
    基于initialpose的位置，绘制类似篮球场的边界
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
        
        # 边界参数 - 类似篮球场布局
        self.x_offset = 0.975  # 整体向x轴负方向偏移距离
        self.y_length = 8.0  # Y轴方向长边：-4到4m
        self.x_length = 15.0   # X轴方向短边：0到15m
        
        # 内嵌小矩形框参数
        self.inner_rect_width = 2.5  # 小矩形短边（与y轴平行）
        self.inner_rect_length = 2.5  # 小矩形长边
        self.inner_rect_center_x = -self.x_offset+self.inner_rect_length/2  # 小矩形中心x坐标
        
        # 外围边界参数
        self.straight_line_length = 0.975  # 直线段长度
        self.arc_radius = 3.1  # 半圆弧半径
        
        # 篮筐和篮板参数
        self.basket_radius = 0.225  # 篮筐半径
        self.backboard_length = 1.8  # 篮板长度
        self.backboard_offset = 0.375  # 篮板中心距离原点的偏移（负x方向）
        
        
        self.get_logger().info('Map边界发布器已启动')
        self.get_logger().info(f'边界尺寸: Y轴长边 {self.y_length}m, X轴短边 {self.x_length}m')
        self.get_logger().info(f'整体偏移: {self.x_offset}m')
        self.get_logger().info(f'内嵌矩形: {self.inner_rect_width}m x {self.inner_rect_length}m')
        self.get_logger().info(f'篮筐半径: {self.basket_radius}m')
        self.get_logger().info(f'篮板长度: {self.backboard_length}m')
    
    def initialpose_callback(self, msg):
        """处理initialpose消息，创建边界"""
        # 提取initialpose的位置和姿态
        initial_x = msg.pose.pose.position.x
        initial_y = msg.pose.pose.position.y
        initial_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
        
        self.get_logger().info(f'收到initialpose: x={initial_x:.3f}, y={initial_y:.3f}, yaw={math.degrees(initial_yaw):.2f}°')
        
        # 创建边界
        self.create_boundary(initial_x, initial_y, initial_yaw)
        
        
        self.get_logger().info('Map边界已更新并发布')
    
    def create_boundary(self, center_x, center_y, center_yaw):
        """创建类似篮球场的边界"""
        # 计算主边界四个角点的坐标（考虑x轴偏移）
        # 边界以initialpose为中心，Y轴方向为长边，X轴正方向为短边
        
        # 主边界角点（相对于initialpose的局部坐标，考虑偏移）
        local_corners = [
            (-self.x_offset, -self.y_length/2),      # 左下角
            (self.x_length - self.x_offset, -self.y_length/2),  # 右下角
            (self.x_length - self.x_offset, self.y_length/2),   # 右上角
            (-self.x_offset, self.y_length/2)        # 左上角
        ]
        
        # 将局部坐标转换为世界坐标
        world_corners = []
        for local_x, local_y in local_corners:
            world_x, world_y = self.transform_local_to_world(local_x, local_y, center_x, center_y, center_yaw)
            world_corners.append((world_x, world_y))
        
        # 计算内嵌小矩形框的角点
        inner_corners = self.calculate_inner_rectangle_corners(center_x, center_y, center_yaw)
        
        # 计算外围边界（直线+圆弧）
        outer_boundary_points = self.calculate_outer_boundary_points(center_x, center_y, center_yaw)
        
        # 计算篮筐和篮板
        basket_center = (center_x, center_y)  # 篮筐圆心在map原点
        backboard_points = self.calculate_backboard_points(center_x, center_y, center_yaw)
        
        # 发布边界标记
        self.publish_boundary_markers(world_corners, inner_corners, outer_boundary_points, basket_center, backboard_points)
        
        # 发布边界点数组
        self.publish_boundary_poses(world_corners)
        
        # 打印边界信息
        self.get_logger().info('主边界角点坐标:')
        for i, (x, y) in enumerate(world_corners):
            self.get_logger().info(f'  角点{i+1}: ({x:.3f}, {y:.3f})')
        
        self.get_logger().info('内嵌矩形角点坐标:')
        for i, (x, y) in enumerate(inner_corners):
            self.get_logger().info(f'  内角点{i+1}: ({x:.3f}, {y:.3f})')
        
        # 打印篮筐和篮板信息
        self.get_logger().info(f'篮筐中心坐标: ({basket_center[0]:.3f}, {basket_center[1]:.3f})')
        self.get_logger().info('篮板端点坐标:')
        for i, (x, y) in enumerate(backboard_points):
            self.get_logger().info(f'  篮板端点{i+1}: ({x:.3f}, {y:.3f})')
    
    def calculate_inner_rectangle_corners(self, center_x, center_y, center_yaw):
        """计算内嵌小矩形框的角点"""
        # 内嵌矩形的局部坐标（相对于initialpose）
        inner_local_corners = [
            (self.inner_rect_center_x - self.inner_rect_length/2, -self.inner_rect_width/2),  # 左下角
            (self.inner_rect_center_x + self.inner_rect_length/2, -self.inner_rect_width/2),  # 右下角
            (self.inner_rect_center_x + self.inner_rect_length/2, self.inner_rect_width/2),   # 右上角
            (self.inner_rect_center_x - self.inner_rect_length/2, self.inner_rect_width/2)    # 左上角
        ]
        
        # 转换为世界坐标
        inner_world_corners = []
        for local_x, local_y in inner_local_corners:
            world_x, world_y = self.transform_local_to_world(local_x, local_y, center_x, center_y, center_yaw)
            inner_world_corners.append((world_x, world_y))
        
        return inner_world_corners
    
    def calculate_outer_boundary_points(self, center_x, center_y, center_yaw):
        """计算外围边界点（直线+圆弧）"""
        outer_points = []
        
        # 计算圆弧中心点（在x轴正方向，距离为arc_radius）
        arc_center_local_x = - self.x_offset + self.straight_line_length
        arc_center_local_y = 0.0
        arc_center_world_x, arc_center_world_y = self.transform_local_to_world(
            arc_center_local_x, arc_center_local_y, center_x, center_y, center_yaw
        )
        
        # 计算直线段的起点和终点
        # 上直线段
        upper_line_start_local = (- self.x_offset, -self.arc_radius)
        upper_line_end_local = (- self.x_offset + self.straight_line_length, -self.arc_radius)
        
        # 下直线段
        lower_line_start_local = (- self.x_offset, self.arc_radius)
        lower_line_end_local = (- self.x_offset + self.straight_line_length, self.arc_radius)
        
        # 转换为世界坐标
        upper_start_world = self.transform_local_to_world(*upper_line_start_local, center_x, center_y, center_yaw)
        upper_end_world = self.transform_local_to_world(*upper_line_end_local, center_x, center_y, center_yaw)
        lower_start_world = self.transform_local_to_world(*lower_line_start_local, center_x, center_y, center_yaw)
        lower_end_world = self.transform_local_to_world(*lower_line_end_local, center_x, center_y, center_yaw)
        
        # 计算圆弧点
        arc_points = self.calculate_arc_points(arc_center_world_x, arc_center_world_y, center_yaw)
        
        # 组装外围边界点
        outer_points.extend([upper_start_world, upper_end_world])
        outer_points.extend(arc_points)
        outer_points.extend([lower_end_world, lower_start_world])
        
        return outer_points
    
    def calculate_arc_points(self, arc_center_x, arc_center_y, center_yaw):
        """计算半圆弧的点"""
        arc_points = []
        
        # 半圆弧的角度范围：从-y到y，即从-90度到90度
        start_angle = -math.pi/2  # -90度
        end_angle = math.pi/2     # 90度
        num_points = 30  # 圆弧上的点数
        
        for i in range(num_points + 1):
            # 计算当前角度
            angle = start_angle + (end_angle - start_angle) * i / num_points
            
            # 计算圆弧上的局部坐标
            local_x = self.arc_radius * math.cos(angle)
            local_y = self.arc_radius * math.sin(angle)
            
            # 转换为世界坐标
            world_x, world_y = self.transform_local_to_world(local_x, local_y, arc_center_x, arc_center_y, center_yaw)
            arc_points.append((world_x, world_y))
        
        return arc_points
    
    def calculate_backboard_points(self, center_x, center_y, center_yaw):
        """计算篮板的两个端点"""
        # 篮板的局部坐标（相对于initialpose）
        # 篮板中心在负x方向偏移backboard_offset距离
        backboard_center_local_x = -self.backboard_offset
        backboard_center_local_y = 0.0
        
        # 篮板的两个端点（垂直于y轴方向）
        backboard_start_local = (backboard_center_local_x, -self.backboard_length/2)
        backboard_end_local = (backboard_center_local_x, self.backboard_length/2)
        
        # 转换为世界坐标
        backboard_start_world = self.transform_local_to_world(*backboard_start_local, center_x, center_y, center_yaw)
        backboard_end_world = self.transform_local_to_world(*backboard_end_local, center_x, center_y, center_yaw)
        
        return [backboard_start_world, backboard_end_world]
    
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
    
    def publish_boundary_markers(self, main_corners, inner_corners, outer_points, basket_center, backboard_points):
        """发布边界标记"""
        marker_array = MarkerArray()
        
        # 首先删除旧的边界标记
        delete_marker = Marker()
        delete_marker.header.frame_id = "camera_init"
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        delete_marker.ns = "map_boundary"
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        # 创建主边界线标记（白色）
        main_line_marker = Marker()
        main_line_marker.header.frame_id = "camera_init"
        main_line_marker.header.stamp = self.get_clock().now().to_msg()
        main_line_marker.ns = "map_boundary"
        main_line_marker.id = 0
        main_line_marker.type = Marker.LINE_STRIP
        main_line_marker.action = Marker.ADD
        
        # 设置主边界线的几何形状
        main_line_marker.points = []
        for x, y in main_corners:
            from geometry_msgs.msg import Point
            point = Point()
            point.x = x
            point.y = y
            point.z = 0.0
            main_line_marker.points.append(point)
        
        # 闭合主边界线
        if len(main_corners) > 0:
            point = Point()
            point.x = main_corners[0][0]
            point.y = main_corners[0][1]
            point.z = 0.0
            main_line_marker.points.append(point)
        
        # 设置主边界线的样式（白色）
        main_line_marker.scale.x = 0.05  # 线宽
        main_line_marker.color = ColorRGBA()
        main_line_marker.color.r = 1.0
        main_line_marker.color.g = 1.0
        main_line_marker.color.b = 1.0
        main_line_marker.color.a = 1.0
        
        marker_array.markers.append(main_line_marker)
        
        # 创建内嵌矩形标记（白色）
        inner_line_marker = Marker()
        inner_line_marker.header.frame_id = "camera_init"
        inner_line_marker.header.stamp = self.get_clock().now().to_msg()
        inner_line_marker.ns = "map_boundary"
        inner_line_marker.id = 1
        inner_line_marker.type = Marker.LINE_STRIP
        inner_line_marker.action = Marker.ADD
        
        # 设置内嵌矩形的几何形状
        inner_line_marker.points = []
        for x, y in inner_corners:
            from geometry_msgs.msg import Point
            point = Point()
            point.x = x
            point.y = y
            point.z = 0.0
            inner_line_marker.points.append(point)
        
        # 闭合内嵌矩形
        if len(inner_corners) > 0:
            point = Point()
            point.x = inner_corners[0][0]
            point.y = inner_corners[0][1]
            point.z = 0.0
            inner_line_marker.points.append(point)
        
        # 设置内嵌矩形的样式（白色）
        inner_line_marker.scale.x = 0.05  # 线宽
        inner_line_marker.color = ColorRGBA()
        inner_line_marker.color.r = 1.0
        inner_line_marker.color.g = 1.0
        inner_line_marker.color.b = 1.0
        inner_line_marker.color.a = 1.0
        
        marker_array.markers.append(inner_line_marker)
        
        # 创建外围边界标记（白色）
        outer_line_marker = Marker()
        outer_line_marker.header.frame_id = "camera_init"
        outer_line_marker.header.stamp = self.get_clock().now().to_msg()
        outer_line_marker.ns = "map_boundary"
        outer_line_marker.id = 2
        outer_line_marker.type = Marker.LINE_STRIP
        outer_line_marker.action = Marker.ADD
        
        # 设置外围边界的几何形状
        outer_line_marker.points = []
        for x, y in outer_points:
            from geometry_msgs.msg import Point
            point = Point()
            point.x = x
            point.y = y
            point.z = 0.0
            outer_line_marker.points.append(point)
        
        # 设置外围边界的样式（白色）
        outer_line_marker.scale.x = 0.05  # 线宽
        outer_line_marker.color = ColorRGBA()
        outer_line_marker.color.r = 1.0
        outer_line_marker.color.g = 1.0
        outer_line_marker.color.b = 1.0
        outer_line_marker.color.a = 1.0
        
        marker_array.markers.append(outer_line_marker)
        
        # 创建篮筐标记（红色圆形）
        basket_marker = Marker()
        basket_marker.header.frame_id = "camera_init"
        basket_marker.header.stamp = self.get_clock().now().to_msg()
        basket_marker.ns = "map_boundary"
        basket_marker.id = 3
        basket_marker.type = Marker.CYLINDER
        basket_marker.action = Marker.ADD
        
        # 设置篮筐的位置和大小
        basket_marker.pose.position.x = basket_center[0]
        basket_marker.pose.position.y = basket_center[1]
        basket_marker.pose.position.z = 0.0
        basket_marker.pose.orientation.w = 1.0
        
        basket_marker.scale.x = self.basket_radius * 2  # 直径
        basket_marker.scale.y = self.basket_radius * 2  # 直径
        basket_marker.scale.z = 0.05  # 高度（很薄）
        
        # 设置篮筐的颜色（红色）
        basket_marker.color = ColorRGBA()
        basket_marker.color.r = 1.0
        basket_marker.color.g = 0.0
        basket_marker.color.b = 0.0
        basket_marker.color.a = 0.8
        
        marker_array.markers.append(basket_marker)
        
        # 创建篮板标记（蓝色直线）
        backboard_marker = Marker()
        backboard_marker.header.frame_id = "camera_init"
        backboard_marker.header.stamp = self.get_clock().now().to_msg()
        backboard_marker.ns = "map_boundary"
        backboard_marker.id = 4
        backboard_marker.type = Marker.LINE_STRIP
        backboard_marker.action = Marker.ADD
        
        # 设置篮板的几何形状
        backboard_marker.points = []
        for x, y in backboard_points:
            from geometry_msgs.msg import Point
            point = Point()
            point.x = x
            point.y = y
            point.z = 0.0
            backboard_marker.points.append(point)
        
        # 设置篮板的样式（蓝色，较粗）
        backboard_marker.scale.x = 0.05  # 线宽
        backboard_marker.color = ColorRGBA()
        backboard_marker.color.r = 1.0
        backboard_marker.color.g = 1.0
        backboard_marker.color.b = 1.0
        backboard_marker.color.a = 1.0
        
        marker_array.markers.append(backboard_marker)
        
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