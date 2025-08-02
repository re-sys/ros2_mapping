#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, PoseStamped, Twist
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray
import yaml
import math
import os
import time

class PositionController(Node):
    def __init__(self):
        super().__init__('position_controller')
        
        # 初始化参数
        self.current_position = Vector3()  # 当前位置 (x, y, yaw)
        self.target_positions = []  # 目标位置列表
        self.current_target_index = 0  # 当前目标索引
        self.position_tolerance = 0.2  # 位置到达容差
        
        # 坐标变换参数
        self.transform_params = [0.0, 0.0, 0.0]  # [x_offset, y_offset, yaw_rotation]
        
        # 移动控制参数
        self.moving = False  # 是否正在移动
        self.move_start_time = 0.0  # 移动开始时间
        self.move_delay = 1.0  # shoot_btn后等待时间
        
        # 控制参数
        self.kp_linear = 2.0  # 线速度比例系数
        self.max_linear_vel = 7.0  # 最大线速度
        
        # 角度控制参数
        self.use_angular_control = False  # 是否使用角度闭环控制
        self.kp_angular = 4.0  # 角速度比例系数
        self.kd_angular = 1.0  # 角速度微分系数
        self.max_angular_vel = 2.2  # 最大角速度
        self.last_error_yaw = 0.0  # 上次角度误差
        
        # 从yaml文件加载目标位置列表
        self.load_target_positions()
        
        # 订阅者
        self.transformed_sub = self.create_subscription(
            Vector3,
            '/transformedxyyaw',
            self.transformed_callback,
            10
        )
        
        self.transformed_params_sub = self.create_subscription(
            Vector3,
            '/transformed_params',
            self.transformed_params_callback,
            10
        )
        
        self.goalpose_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goalpose_callback,
            10
        )
        
        self.shoot_btn_sub = self.create_subscription(
            Bool,
            '/shoot_btn',
            self.shoot_btn_callback,
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
            '/target_markers',
            10
        )
        
        # 控制定时器
        self.control_timer = self.create_timer(0.05, self.control_callback)
        
        # Marker更新定时器
        self.marker_timer = self.create_timer(1.0, self.update_markers)
        
        self.get_logger().info('Position Controller initialized')
        self.get_logger().info(f'Loaded {len(self.target_positions)} target positions from config')
        self.get_logger().info(f'Angular control: {"ON" if self.use_angular_control else "OFF"}')
        
    def toggle_angular_control(self):
        """切换角度控制模式"""
        self.use_angular_control = not self.use_angular_control
        self.get_logger().info(f'Angular control {"ENABLED" if self.use_angular_control else "DISABLED"}')
        
    def set_angular_control(self, enabled):
        """设置角度控制模式"""
        self.use_angular_control = enabled
        self.get_logger().info(f'Angular control {"ENABLED" if self.use_angular_control else "DISABLED"}')
        
    def load_target_positions(self):
        """从yaml文件加载目标位置列表"""
        config_file = os.path.join(os.path.dirname(__file__), 'target_positions.yaml')
        
        if os.path.exists(config_file):
            try:
                with open(config_file, 'r') as file:
                    config = yaml.safe_load(file)
                    positions = config.get('target_positions', [])
                    
                    for pos in positions:
                        target = Vector3()
                        target.x = pos.get('x', 0.0)
                        target.y = pos.get('y', 0.0)
                        target.z = pos.get('yaw', 0.0)  # z存储yaw角度
                        self.target_positions.append(target)
                        
                self.get_logger().info(f'Successfully loaded {len(self.target_positions)} positions from {config_file}')
            except Exception as e:
                self.get_logger().warn(f'Failed to load config file: {e}')
        else:
            self.get_logger().info(f'Config file not found: {config_file}, starting with empty position list')
    
    def transformed_callback(self, msg):
        """处理当前位置信息"""
        self.current_position = msg
    def inverse_transform(self,x,y,yaw):

        yaw = -yaw
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        rotated_x = -(x * cos_yaw - y * sin_yaw)
        rotated_y = -(x * sin_yaw + y * cos_yaw)
        return rotated_x,rotated_y,yaw    

    def transformed_params_callback(self, msg):
        """处理坐标变换参数"""
        self.transform_params[0] = msg.x
        self.transform_params[1] = msg.y
        self.transform_params[2] = msg.z
        
        # 重新变换所有目标位置
        self.transform_all_targets()
        
        # 重新发布marker
        self.update_markers()
        
    def goalpose_callback(self, msg):
        """处理新的目标位姿，添加到位置列表"""
        target = Vector3()
        target.x = msg.pose.position.x
        target.y = msg.pose.position.y
        
        # 从四元数提取yaw角
        orientation = msg.pose.orientation
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        target.z = math.atan2(siny_cosp, cosy_cosp)

        self.inverse_transform_params = self.inverse_transform(self.transform_params[0],self.transform_params[1],self.transform_params[2])

        self.target_positions.append(target)
        
        # 保存原始目标位置
        if not hasattr(self, 'original_targets'):
            self.original_targets = []
        original = Vector3()
        transformed_x,transformed_y,transformed_yaw = self.inverse_transform_point(target.x,target.y,target.z)
        original.x = transformed_x  # 保存原始坐标
        original.y = transformed_y
        original.z = transformed_yaw
        # self.get_logger().info(f'original_target: {target},original_in_map: {original}')
        # self.get_logger().info(f'inverse_transform_params: {self.inverse_transform_params}')
        self.original_targets.append(original)
        
        # self.get_logger().info(f'Added new target position (transformed): x={target.x:.3f}, y={target.y:.3f}, yaw={math.degrees(target.z):.2f}°')
        # self.get_logger().info(f'Total target positions: {len(self.target_positions)}')
    def inverse_transform_point(self,x,y,yaw):
        cos_yaw = math.cos(self.inverse_transform_params[2])
        sin_yaw = math.sin(self.inverse_transform_params[2])
        rotated_x = x * cos_yaw - y * sin_yaw
        rotated_y = x * sin_yaw + y * cos_yaw
        transformed_x = rotated_x + self.inverse_transform_params[0]
        transformed_y = rotated_y + self.inverse_transform_params[1]
        transformed_yaw = yaw + self.inverse_transform_params[2]

        return transformed_x,transformed_y,transformed_yaw

    def transform_all_targets(self):
        """重新变换所有目标位置"""
        if not self.target_positions:
            return
            
        # 保存原始目标位置（如果还没有保存的话）
        if not hasattr(self, 'original_targets'):
            self.original_targets = []
            for target in self.target_positions:
                original = Vector3()
                original.x = target.x
                original.y = target.y
                original.z = target.z
                self.original_targets.append(original)
        
        # 重新变换所有目标位置
        for i, original_target in enumerate(self.original_targets):
            # 应用坐标变换
            # 1. 先旋转
            cos_yaw = math.cos(self.transform_params[2])
            sin_yaw = math.sin(self.transform_params[2])
            
            rotated_x = original_target.x * cos_yaw - original_target.y * sin_yaw
            rotated_y = original_target.x * sin_yaw + original_target.y * cos_yaw
            self.get_logger().info(f'rotated_x: {rotated_x}, rotated_y: {rotated_y}')
            self.get_logger().info(f'transform_params: {self.transform_params}')
            self.get_logger().info(f'original_target: {original_target}')
            # 2. 再平移
            transformed_x = rotated_x + self.transform_params[0]
            transformed_y = rotated_y + self.transform_params[1]
            
            # 3. 更新yaw角
            transformed_yaw = original_target.z + self.transform_params[2]
            
            # 更新目标位置
            self.target_positions[i].x = transformed_x
            self.target_positions[i].y = transformed_y
            self.target_positions[i].z = transformed_yaw
            
        self.get_logger().info(f'Transformed {len(self.target_positions)} target positions with new transform parameters')
        
    def shoot_btn_callback(self, msg):
        """处理shoot_btn状态"""
        # 只要收到shoot_btn消息就移动到下一个目标
        if self.current_target_index < len(self.target_positions) - 1:
            self.current_target_index += 1
            self.move_start_time = time.time()
            self.moving = True
            self.get_logger().info(f'Moving to next target: {self.current_target_index + 1}/{len(self.target_positions)}')
        else:
            self.get_logger().info('Reached last target position')
        
    def calculate_error_yaw(self, target_yaw):
        """计算角度误差（仿照rotate_shoot的逻辑）"""
        current_yaw = self.current_position.z
        
        # 处理角度环绕
        if (current_yaw - target_yaw) > math.pi:
            current_yaw -= 2 * math.pi
        elif (current_yaw - target_yaw) < -math.pi:
            current_yaw += 2 * math.pi
            
        error_yaw = target_yaw - current_yaw
        return error_yaw
    
    def is_position_reached(self, target):
        """检查是否到达目标位置"""
        if not self.current_position:
            return False
            
        # 计算位置距离
        dx = target.x - self.current_position.x
        dy = target.y - self.current_position.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # 根据是否使用角度控制来决定检查条件
        if self.use_angular_control:
            # 计算角度误差
            error_yaw = self.calculate_error_yaw(target.z)
            angle_diff = abs(error_yaw)
            return distance < self.position_tolerance and angle_diff < 0.1  # 角度容差0.1弧度
        else:
            # 只检查位置距离，不检查角度
            return distance < self.position_tolerance
        
    def control_callback(self):
        """主控制循环"""
        if not self.target_positions:
            return
            
        # 检查是否正在移动且等待时间已过
        if self.moving and (time.time() - self.move_start_time) >= self.move_delay:
            # 获取当前目标
            current_target = self.target_positions[self.current_target_index]
            
            # 检查是否到达目标
            if self.is_position_reached(current_target):
                self.moving = False
                self.get_logger().info(f'Reached target {self.current_target_index + 1}: x={current_target.x:.3f}, y={current_target.y:.3f}')
                self.cmd_vel_pub.publish(Twist())
            else:
                # 计算控制命令
                cmd_vel = self.compute_control_command(current_target)
                self.cmd_vel_pub.publish(cmd_vel)
                
                # 计算到目标的距离和角度
                dx = current_target.x - self.current_position.x
                dy = current_target.y - self.current_position.y
                distance = math.sqrt(dx*dx + dy*dy)
                
                if self.use_angular_control:
                    error_yaw = self.calculate_error_yaw(current_target.z)
                    self.get_logger().info(f'Moving to target {self.current_target_index + 1}: distance={distance:.3f}m, angle_diff={math.degrees(abs(error_yaw)):.2f}°', 
                                         throttle_duration_sec=2.0)
                else:
                    self.get_logger().info(f'Moving to target {self.current_target_index + 1}: distance={distance:.3f}m', 
                                         throttle_duration_sec=2.0)
    
    def compute_control_command(self, target):
        """计算控制命令"""
        cmd_vel = Twist()
        
        if not self.current_position:
            return cmd_vel
            
        # 计算位置误差
        dx = target.x - self.current_position.x
        dy = target.y - self.current_position.y
        
        # 计算线速度（在机器人坐标系下）
        # 将全局误差转换到机器人坐标系
        cos_yaw = math.cos(self.current_position.z)
        sin_yaw = math.sin(self.current_position.z)
        
        # 机器人坐标系下的误差
        robot_dx = dx * cos_yaw + dy * sin_yaw
        robot_dy = -dx * sin_yaw + dy * cos_yaw
        
        # 计算线速度
        linear_x = self.kp_linear * robot_dx
        linear_y = self.kp_linear * robot_dy
        
        # 限制线速度范围
        linear_x = max(-self.max_linear_vel, min(self.max_linear_vel, linear_x))
        linear_y = max(-self.max_linear_vel, min(self.max_linear_vel, linear_y))
        
        # 根据是否使用角度控制来计算角速度
        if self.use_angular_control:
            # 计算角度误差（PID控制）
            error_yaw = self.calculate_error_yaw(target.z)
            angular_z = self.kp_angular * error_yaw + self.kd_angular * (error_yaw - self.last_error_yaw)
            self.last_error_yaw = error_yaw
            
            # 限制角速度范围
            angular_z = max(-self.max_angular_vel, min(self.max_angular_vel, angular_z))
        else:
            # 不控制角速度
            angular_z = 0.0
        
        cmd_vel.linear.x = linear_x
        cmd_vel.linear.y = linear_y
        cmd_vel.linear.z = 0.0
        cmd_vel.angular.x = 0.0
        cmd_vel.angular.y = 0.0
        cmd_vel.angular.z = angular_z
        
        return cmd_vel

    def create_target_marker(self, position, marker_id, is_next_target=False):
        """创建目标位置marker"""
        marker = Marker()
        marker.header.frame_id = "camera_init"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "target_positions"
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # 设置位置
        marker.pose.position.x = position.x
        marker.pose.position.y = position.y
        marker.pose.position.z = 0.0
        
        # 设置方向（箭头指向yaw角度）
        yaw = position.z
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = math.sin(yaw / 2.0)
        marker.pose.orientation.w = math.cos(yaw / 2.0)
        
        # 设置大小
        marker.scale.x = 0.5  # 箭头长度
        marker.scale.y = 0.1  # 箭头宽度
        marker.scale.z = 0.1  # 箭头高度
        
        # 设置颜色
        if is_next_target:
            # 下一个目标：红色
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
        else:
            # 其他目标：蓝色
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 0.7
            
        return marker
    

    
    def update_markers(self):
        """更新marker可视化"""
        if not self.target_positions:
            return
            
        marker_array = MarkerArray()
        marker_id = 0
        
        # 首先删除所有旧的marker
        delete_marker = Marker()
        delete_marker.header.frame_id = "camera_init"
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        delete_marker.ns = "target_positions"
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        # 创建未来目标的marker
        for i in range(self.current_target_index, len(self.target_positions)):
            target = self.target_positions[i]
            is_next_target = (i == self.current_target_index)
            
            # 创建箭头marker
            arrow_marker = self.create_target_marker(target, marker_id, is_next_target)
            marker_array.markers.append(arrow_marker)
            marker_id += 1
        
        # 发布marker数组
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = PositionController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 