#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, PoseStamped, Twist, PoseArray
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import yaml
import math
import os
import time

class MultiPointPositionController(Node):
    def __init__(self):
        super().__init__('multi_point_position_controller')
        
        # 初始化参数
        self.current_position = Vector3()  # 当前位置 (x, y, yaw)
        self.target_positions = []  # 目标位置列表
        self.current_target_index = 0  # 当前目标索引
        self.position_tolerance = 0.1  # 位置到达容差
        self.yaw_tolerance = 0.1  # 角度到达容差
        
        # 坐标变换参数
        self.transform_params = [0.0, 0.0, 0.0]  # [x_offset, y_offset, yaw_rotation] - camera_init到map
        self.inverse_transform_params = [0.0, 0.0, 0.0]  # [x_offset, y_offset, yaw_rotation] - map到camera_init
        self.map_origin_set = False  # map原点是否已设置
        
        # 移动控制参数
        self.moving = False  # 是否正在移动
        self.move_start_time = 0.0  # 移动开始时间
        self.move_delay = 1.0  # shoot_btn后等待时间
        self.sequence_completed = False  # 序列是否完成
        
        # 控制参数
        self.kp_linear = 2.0  # 线速度比例系数
        self.max_linear_vel = 1.5  # 最大线速度
        
        # 角度控制参数
        self.kp_angular = 2.0  # 角速度比例系数
        self.max_angular_vel = 2.2  # 最大角速度
        
        # 预定义的目标点位（在initialpose坐标系下）
        self.waypoints = [
            (2.0, 0.0, 0.0),    # 前方2m
            (4.0, 2.0, 0.0),    # 右前方
            (4.0, -2.0, 0.0),   # 左前方
            (6.0, 0.0, 0.0),    # 前方6m
            (2.0, 0.0, 0.0),    # 回到前方2m
        ]
        
        # 订阅者
        self.odom_sub = self.create_subscription(
            Odometry,
            '/Odometry',
            self.odom_callback,
            10
        )
        
        self.initialpose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initialpose_callback,
            10
        )
        self.get_logger().info('Initialpose subscriber created')
        
        self.shoot_btn_sub = self.create_subscription(
            Bool,
            '/position_btn',
            self.shoot_btn_callback,
            10
        )
        
        # 订阅goalpose话题
        self.goalpose_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goalpose_callback,
            10
        )
        
        # 发布者
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Marker发布者
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/waypoint_markers',
            10
        )
        
        # 控制定时器
        self.control_timer = self.create_timer(0.05, self.control_callback)
        
        # Marker更新定时器
        self.marker_timer = self.create_timer(1.0, self.update_markers)
        
        self.get_logger().info('Multi-Point Position Controller initialized')
        self.get_logger().info(f'Waypoints: {len(self.waypoints)} points')
        self.get_logger().info('Waiting for initialpose to start...')
        
    def initialpose_callback(self, msg):
        """处理initialpose消息，设置map坐标系原点"""
        self.get_logger().info('Received initialpose message!')
        
        # 提取initialpose的位置和姿态
        map_origin_x = msg.pose.pose.position.x
        map_origin_y = msg.pose.pose.position.y
        map_origin_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
        
        self.get_logger().info(f'Initialpose data: x={map_origin_x:.3f}, y={map_origin_y:.3f}, yaw={math.degrees(map_origin_yaw):.2f}°')
        
        # 计算从map原点到当前odom坐标系的变换参数
        rotated_x, rotated_y, yaw = self.inverse_transform(map_origin_x, map_origin_y, map_origin_yaw)
        self.transform_params[0] = rotated_x
        self.transform_params[1] = rotated_y
        self.transform_params[2] = yaw
        
        # 计算map到camera_init的变换参数（逆变换）
        # 这里需要计算从map坐标系到camera_init坐标系的变换
        # 由于initialpose给出的是map原点在camera_init坐标系下的位置
        # 所以map到camera_init的变换就是减去这个偏移，然后旋转-角度
        self.inverse_transform_params[0] = map_origin_x
        self.inverse_transform_params[1] = map_origin_y
        self.inverse_transform_params[2] = map_origin_yaw
        
        self.map_origin_set = True
        
        # 初始化目标位置列表
        self.target_positions = self.waypoints.copy()
        
        # 立即更新marker显示
        self.update_markers()
        
        self.get_logger().info('Map origin set from initialpose')
        self.get_logger().info(f'Transform params: x={self.transform_params[0]:.3f}, y={self.transform_params[1]:.3f}, yaw={math.degrees(self.transform_params[2]):.2f}°')
        self.get_logger().info('Ready to start waypoint navigation')
    
    def odom_callback(self, msg):
        """处理Odometry消息并进行坐标转换"""
        if not self.map_origin_set:
            return
            
        # 提取原始位姿（在camera_init坐标系下）
        self.current_position.x = msg.pose.pose.position.x
        self.current_position.y = msg.pose.pose.position.y
        self.current_position.z = self.quaternion_to_yaw(msg.pose.pose.orientation)
    
    def transform_map_to_camera_init(self, map_x, map_y, map_yaw):
        """坐标变换：从map坐标系到camera_init坐标系"""
        # 1. 先旋转（绕原点旋转）
        cos_yaw = math.cos(self.inverse_transform_params[2])
        sin_yaw = math.sin(self.inverse_transform_params[2])
        
        rotated_x = map_x * cos_yaw - map_y * sin_yaw
        rotated_y = map_x * sin_yaw + map_y * cos_yaw
        
        # 2. 再平移
        translated_x = rotated_x + self.inverse_transform_params[0]
        translated_y = rotated_y + self.inverse_transform_params[1]
        
        # 3. 更新yaw角
        translated_yaw = map_yaw + self.inverse_transform_params[2]
        
        return translated_x, translated_y, translated_yaw
    
    def transform_camera_init_to_map(self, camera_x, camera_y, camera_yaw):
        """坐标变换：从camera_init坐标系到map坐标系"""
        # 1. 先旋转
        cos_yaw = math.cos(self.transform_params[2])
        sin_yaw = math.sin(self.transform_params[2])
        

        rotated_x = camera_x * cos_yaw - camera_y * sin_yaw
        rotated_y = camera_x * sin_yaw + camera_y * cos_yaw
        # 2. 再平移
        translated_x = rotated_x + self.transform_params[0]
        translated_y = rotated_y + self.transform_params[1]
        
        # 3. 更新yaw角
        rotated_yaw = camera_yaw + self.transform_params[2]
        
        
        return translated_x, translated_y, rotated_yaw
    
    def inverse_transform(self, x, y, yaw):
        """逆变换"""
        neg_yaw = -yaw
        cos_yaw = math.cos(neg_yaw)
        sin_yaw = math.sin(neg_yaw)
        rotated_x = -(x * cos_yaw - y * sin_yaw)
        rotated_y = -(x * sin_yaw + y * cos_yaw)
        return rotated_x, rotated_y, neg_yaw
    
    def quaternion_to_yaw(self, quaternion):
        """从四元数提取yaw角"""
        siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw
    
    def shoot_btn_callback(self, msg):
        """处理shoot_btn消息，移动到下一个目标点"""
        if not self.map_origin_set:
            self.get_logger().warn('Map origin not set yet, cannot start navigation')
            return
            
        if msg.data:
            if self.sequence_completed:
                # 重置序列
                self.current_target_index = 0
                self.sequence_completed = False
                self.moving = False
                self.get_logger().info('Navigation sequence reset')
            else:
                # 移动到下一个目标点
                if self.current_target_index < len(self.target_positions):
                    self.moving = True
                    self.move_start_time = time.time()
                    current_target = self.target_positions[self.current_target_index]
                    self.get_logger().info(f'Moving to waypoint {self.current_target_index + 1}: ({current_target[0]:.2f}, {current_target[1]:.2f})')
                else:
                    self.get_logger().info('No more waypoints to visit')
    
    def goalpose_callback(self, msg):
        """处理goalpose消息，添加新的目标点"""
        if not self.map_origin_set:
            self.get_logger().warn('Map origin not set yet, cannot add goal pose')
            return
            
        # 提取goalpose的位置和姿态（在camera_init坐标系下）
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        goal_yaw = self.quaternion_to_yaw(msg.pose.orientation)
        
        # 将camera_init坐标系下的目标点转换到map坐标系
        map_x, map_y, map_yaw = self.transform_camera_init_to_map(goal_x, goal_y, goal_yaw)
        
        # 添加到目标位置列表
        new_waypoint = (map_x, map_y, map_yaw)
        self.target_positions.append(new_waypoint)
        
        # 打印坐标信息
        self.get_logger().info(f'Added new waypoint:')
        self.get_logger().info(f'  Camera_init coordinates: x={goal_x:.3f}, y={goal_y:.3f}, yaw={math.degrees(goal_yaw):.2f}°')
        self.get_logger().info(f'  Map coordinates: x={map_x:.3f}, y={map_y:.3f}, yaw={math.degrees(map_yaw):.2f}°')
        self.get_logger().info(f'  transformed_map_x={self.transform_params[0]:.3f}, y={self.transform_params[1]:.3f}, yaw={math.degrees(self.transform_params[2]):.2f}°')
        self.get_logger().info(f'Total waypoints: {len(self.target_positions)}')
        
        # 更新marker显示
        self.update_markers()
    
    def calculate_error_yaw(self, target_yaw):
        """计算角度误差"""
        current_yaw = self.current_position.z
        target_yaw = target_yaw
        
        # 处理角度跨越±π的情况
        while current_yaw - target_yaw > math.pi:
            current_yaw -= 2 * math.pi
        while current_yaw - target_yaw < -math.pi:
            current_yaw += 2 * math.pi
            
        error_yaw = target_yaw - current_yaw
        return error_yaw
    
    def is_position_reached(self, target):
        """检查是否到达目标位置"""
        # 将map坐标系下的目标点转换到camera_init坐标系
        target_camera_x, target_camera_y, target_camera_yaw = self.transform_map_to_camera_init(target[0], target[1], target[2])
        
        dx = target_camera_x - self.current_position.x
        dy = target_camera_y - self.current_position.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # 计算角度误差
        error_yaw = self.calculate_error_yaw(target_camera_yaw)
        
        return distance < self.position_tolerance and abs(error_yaw) < self.yaw_tolerance
    
    def control_callback(self):
        """控制回调函数"""
        if not self.map_origin_set or not self.moving or self.sequence_completed:
            return
            
        # 检查移动延迟
        if time.time() - self.move_start_time < self.move_delay:
            return
            
        # 检查是否还有目标点
        if self.current_target_index >= len(self.target_positions):
            self.sequence_completed = True
            self.moving = False
            self.get_logger().info('Navigation sequence completed!')
            return
            
        # 获取当前目标
        current_target = self.target_positions[self.current_target_index]
        
        # 检查是否到达当前目标
        if self.is_position_reached(current_target):
            self.get_logger().info(f'Reached waypoint {self.current_target_index + 1}: ({current_target[0]:.2f}, {current_target[1]:.2f})')
            
            # 删除已到达的marker
            self.delete_waypoint_marker(self.current_target_index)
            
            self.current_target_index += 1
            
            # 停止移动，等待下一次shoot_btn信号
            self.moving = False
            
            # 发布停止命令
            cmd_vel = Twist()
            self.cmd_vel_pub.publish(cmd_vel)
            
            # 更新marker显示下一个目标点
            self.update_markers()
            
            if self.current_target_index >= len(self.target_positions):
                self.sequence_completed = True
                self.get_logger().info('Navigation sequence completed!')
            else:
                self.get_logger().info('Waiting for next shoot_btn to continue...')
            return
        
        # 计算控制命令
        self.compute_control_command(current_target)
    
    def compute_control_command(self, target):
        """计算控制命令"""
        # 将map坐标系下的目标点转换到camera_init坐标系
        target_camera_x, target_camera_y, target_camera_yaw = self.transform_map_to_camera_init(target[0], target[1], target[2])
        
        # 计算位置误差（在camera_init坐标系下）
        dx = target_camera_x - self.current_position.x
        dy = target_camera_y - self.current_position.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # 计算角度误差
        error_yaw = self.calculate_error_yaw(target_camera_yaw)
        
        # 创建cmd_vel消息
        cmd_vel = Twist()
        
        # 线速度控制（基于距离，沿着camera_init的xy轴）
        linear_vel_x = self.kp_linear * dx
        linear_vel_y = self.kp_linear * dy
        
        # 限制线速度
        linear_vel_x = max(-self.max_linear_vel, min(self.max_linear_vel, linear_vel_x))
        linear_vel_y = max(-self.max_linear_vel, min(self.max_linear_vel, linear_vel_y))
        
        # 角速度控制（基于角度误差，只使用kp）
        angular_vel = self.kp_angular * error_yaw
        angular_vel = max(-self.max_angular_vel, min(self.max_angular_vel, angular_vel))
        
        # 设置速度
        cmd_vel.linear.x = linear_vel_x
        cmd_vel.linear.y = linear_vel_y
        cmd_vel.angular.z = angular_vel
        
        # 发布控制命令
        self.cmd_vel_pub.publish(cmd_vel)
        
        # 输出调试信息
        if self.current_target_index % 10 == 0:  # 每10次输出一次
            self.get_logger().info(f'Target {self.current_target_index + 1}: distance={distance:.2f}m, angle_error={math.degrees(error_yaw):.1f}°')
    
    def create_waypoint_marker(self, position, marker_id, is_current_target=False):
        """创建路径点标记"""
        marker = Marker()
        marker.header.frame_id = "camera_init"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "waypoints"
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = 0.0
        
        # 设置箭头方向（指向yaw角方向）
        yaw = position[2]
        marker.pose.orientation.z = math.sin(yaw / 2.0)
        marker.pose.orientation.w = math.cos(yaw / 2.0)
        
        # 设置标记大小
        if is_current_target:
            marker.scale.x = 0.8  # 箭头长度
            marker.scale.y = 0.3  # 箭头宽度
            marker.scale.z = 0.3  # 箭头高度
            # 当前目标为红色
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        else:
            marker.scale.x = 0.6  # 箭头长度
            marker.scale.y = 0.2  # 箭头宽度
            marker.scale.z = 0.2  # 箭头高度
            # 其他目标为蓝色
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
        
        marker.color.a = 1.0
        return marker
    
    def delete_waypoint_marker(self, waypoint_index):
        """删除指定的路径点marker"""
        marker_array = MarkerArray()
        
        # 创建删除marker
        delete_marker = Marker()
        delete_marker.header.frame_id = "camera_init"
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        delete_marker.ns = "waypoints"
        delete_marker.id = waypoint_index
        delete_marker.action = Marker.DELETE
        
        marker_array.markers.append(delete_marker)
        
        # 发布删除命令
        self.marker_pub.publish(marker_array)
        
        self.get_logger().info(f'Deleted waypoint marker {waypoint_index + 1}')
    
    def update_markers(self):
        """更新路径点标记"""
        if not self.map_origin_set:
            self.get_logger().debug('Cannot update markers: map_origin not set')
            return
            
        self.get_logger().debug('Updating markers...')
        marker_array = MarkerArray()
        
        # 删除旧的标记
        delete_marker = Marker()
        delete_marker.header.frame_id = "camera_init"
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        delete_marker.ns = "waypoints"
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        # 创建路径点标记
        for i, waypoint in enumerate(self.target_positions):
            # 下一个目标点标红（不管是否在移动）
            is_current = (i == self.current_target_index and not self.sequence_completed)
            # 将map坐标系下的路径点转换到camera_init坐标系
            waypoint_camera_x, waypoint_camera_y, _ = self.transform_map_to_camera_init(waypoint[0], waypoint[1], waypoint[2])
            waypoint_camera = (waypoint_camera_x, waypoint_camera_y, waypoint[2])
            marker = self.create_waypoint_marker(waypoint_camera, i, is_current)
            marker_array.markers.append(marker)
        
        # 发布标记
        self.marker_pub.publish(marker_array)
        
        # 输出调试信息
        # self.get_logger().info(f'Updated {len(self.target_positions)} waypoint markers')

def main(args=None):
    rclpy.init(args=args)
    node = MultiPointPositionController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 