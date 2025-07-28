#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, Quaternion, PoseWithCovarianceStamped
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster
import math
import numpy as np

class CoordinateTransformerNode(Node):
    def __init__(self):
        super().__init__('coordinate_transformer_node')
        
        # 初始化坐标变换参数 (x, y, yaw)
        self.transform_params = [0.0, 0.0, 0.0]  # [x_offset, y_offset, yaw_rotation]
        self.offset = 0.1
        
        # 初始化变换后的坐标
        self.transformed_x = 0.0
        self.transformed_y = 0.0
        self.transformed_yaw = 0.0
        
        # 初始化目标位姿
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_yaw = 0.0
        self.has_goal = False
        
        # 初始化map原点
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_origin_yaw = 0.0
        self.map_origin_set = False
        
        # PID控制参数
        self.kp_linear_x = 1.0
        self.kp_linear_y = 1.0
        self.kp_angular = 2.0
        self.max_linear_vel = 0.5
        self.max_angular_vel = 1.0
        
        # 订阅FastLIO的Odometry话题
        self.odom_sub = self.create_subscription(
            Odometry,
            '/Odometry',  # FastLIO发布的Odometry话题
            self.odom_callback,
            10
        )
        
        # 订阅坐标变换参数
        self.transform_params_sub = self.create_subscription(
            Float32MultiArray,
            '/transform_params',  # 用于接收变换参数的话题
            self.transform_params_callback,
            10
        )
        
        # 订阅目标位姿
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        
        # 订阅initialpose
        self.initialpose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initialpose_callback,
            10
        )
        
        # 发布cmd_vel
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # TF广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 发布map到base_link的静态变换
        self.publish_static_transform()
        
        # 创建控制定时器，10Hz
        self.control_timer = self.create_timer(0.1, self.control_callback)
        
        self.get_logger().info('Coordinate Transformer Node initialized')
        self.get_logger().info('Transform parameters: x={:.3f}, y={:.3f}, yaw={:.3f}'.format(
            self.transform_params[0], self.transform_params[1], self.transform_params[2]))

    def initialpose_callback(self, msg):
        """处理initialpose消息，设置map坐标系原点"""
        # 提取initialpose的位置和姿态
        self.map_origin_x = msg.pose.pose.position.x
        self.map_origin_y = msg.pose.pose.position.y
        self.map_origin_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
        
        # 计算从map原点到当前odom坐标系的变换参数
        # 这个变换将map坐标系的原点映射到initialpose指定的位置
        cos_yaw = math.cos(-self.map_origin_yaw)
        sin_yaw = math.sin(-self.map_origin_yaw)

        rotated_x = -self.map_origin_x * cos_yaw + self.map_origin_y * sin_yaw
        rotated_y = -self.map_origin_x * sin_yaw - self.map_origin_y * cos_yaw

        self.transform_params[0] = rotated_x
        self.transform_params[1] = rotated_y
        self.transform_params[2] = -self.map_origin_yaw
        
        self.map_origin_set = True
        
        # 更新静态变换
        self.publish_static_transform()
        
        self.get_logger().info('Map origin set from initialpose: x={:.3f}, y={:.3f}, yaw={:.3f}'.format(
            self.map_origin_x, self.map_origin_y, self.map_origin_yaw))
        self.get_logger().info('Updated transform parameters: x={:.3f}, y={:.3f}, yaw={:.3f}'.format(
            self.transform_params[0], self.transform_params[1], self.transform_params[2]))


    def transform_params_callback(self, msg):
        """接收坐标变换参数"""
        if len(msg.data) >= 3:
            self.transform_params = [msg.data[0], msg.data[1], msg.data[2]]
            self.get_logger().info('Updated transform parameters: x={:.3f}, y={:.3f}, yaw={:.3f}'.format(
                self.transform_params[0], self.transform_params[1], self.transform_params[2]))
            
            # 更新静态变换
            self.publish_static_transform()
        else:
            self.get_logger().warn('Invalid transform parameters. Expected 3 values, got {}'.format(len(msg.data)))
    
    def goal_callback(self, msg):
        base_x = msg.pose.position.x
        base_y = msg.pose.position.y
        original_yaw = self.quaternion_to_yaw(msg.pose.orientation)
        if msg.header.frame_id == 'map':
            transformed_x = base_x
            transformed_y = base_y
        else:

            cos_yaw = math.cos(self.transform_params[2])
            sin_yaw = math.sin(self.transform_params[2])
            

            rotated_x = base_x * cos_yaw - base_y * sin_yaw
            rotated_y = base_x * sin_yaw + base_y * cos_yaw
            
            # 2. 再平移
            transformed_x = rotated_x + self.transform_params[0]
            transformed_y = rotated_y + self.transform_params[1]
            

        self.goal_x = transformed_x
        self.goal_y = transformed_y
        self.goal_yaw = math.atan2(-transformed_y,-transformed_x)
        self.has_goal = True
        self.get_logger().info('New goal received: x={:.3f}, y={:.3f}, yaw={:.3f}'.format(
            self.goal_x, self.goal_y, self.goal_yaw))
    
    def control_callback(self):
        """舵轮控制回调函数，基于当前Yaw角配置x,y速度方向"""
        if not self.has_goal:
            return
            
        # 计算位置误差
        error_x = self.goal_x - self.transformed_x
        error_y = self.goal_y - self.transformed_y
        error_yaw = self.goal_yaw - self.transformed_yaw
        
        # 角度误差归一化到[-pi, pi]
        while error_yaw > math.pi:
            error_yaw -= 2 * math.pi
        while error_yaw < -math.pi:
            error_yaw += 2 * math.pi
        
        # 计算距离误差
        distance_error = math.sqrt(error_x**2 + error_y**2)
        
        # 创建cmd_vel消息
        cmd_vel = Twist()
        
        # 计算目标方向角度（全局坐标系）
        target_angle = math.atan2(error_y, error_x)
        
        # 计算相对于当前Yaw的方向角度
        relative_angle = target_angle - self.transformed_yaw
        
        # 角度归一化
        while relative_angle > math.pi:
            relative_angle -= 2 * math.pi
        while relative_angle < -math.pi:
            relative_angle += 2 * math.pi
        
        # 计算速度分量（基于当前Yaw方向）
        if distance_error > 0.05:  # 有位置误差时
            # 计算速度大小（比例控制）
            speed = min(self.kp_linear_x * distance_error, self.max_linear_vel)
            
            # 分解为x,y分量（基于当前Yaw方向）
            cmd_vel.linear.x = speed * math.cos(relative_angle)
            cmd_vel.linear.y = speed * math.sin(relative_angle)
        else:  # 位置误差很小时
            cmd_vel.linear.x = 0.0
            cmd_vel.linear.y = 0.0
        
        # 角速度控制（基于yaw误差）
        cmd_vel.angular.z = self.kp_angular * error_yaw
        
        # 限制速度
        cmd_vel.linear.x = max(-self.max_linear_vel, min(self.max_linear_vel, cmd_vel.linear.x))
        cmd_vel.linear.y = max(-self.max_linear_vel, min(self.max_linear_vel, cmd_vel.linear.y))
        cmd_vel.angular.z = max(-self.max_angular_vel, min(self.max_angular_vel, cmd_vel.angular.z))
        
        # 发布cmd_vel
        self.cmd_vel_pub.publish(cmd_vel)
        
        # 检查是否到达目标
        if distance_error < 0.05 and abs(error_yaw) < 0.05:
            self.get_logger().info('Goal reached!')
            self.has_goal = False
    
    def odom_callback(self, msg):
        """处理Odometry消息并进行坐标转换"""
        # 提取原始位姿
        original_x = msg.pose.pose.position.x
        original_y = msg.pose.pose.position.y
        
        # 从四元数提取yaw角
        original_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
        base_x = math.cos(original_yaw)*self.offset + original_x
        base_y = math.sin(original_yaw)*self.offset + original_y
        # 应用坐标变换
        # 1. 先旋转
        cos_yaw = math.cos(self.transform_params[2])
        sin_yaw = math.sin(self.transform_params[2])
        
        rotated_x = base_x * cos_yaw - base_y * sin_yaw
        rotated_y = base_x * sin_yaw + base_y * cos_yaw
        
        # 2. 再平移
        self.transformed_x = rotated_x + self.transform_params[0]
        self.transformed_y = rotated_y + self.transform_params[1]
        
        # 3. 更新yaw角
        self.transformed_yaw = original_yaw + self.transform_params[2]
        
        # 发布TF变换
        self.publish_dynamic_transform(msg, base_x, base_y)
        self.publish_static_transform()

    
    def quaternion_to_yaw(self, quaternion):
        """从四元数提取yaw角"""
        # 使用四元数的z分量计算yaw角
        siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw
    
    def yaw_to_quaternion(self, yaw):
        """从yaw角创建四元数"""
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = math.sin(yaw / 2.0)
        quaternion.w = math.cos(yaw / 2.0)
        return quaternion
    
    def publish_static_transform(self):
        """发布map到camera_init的静态变换"""
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'camera_init'
        
        # 设置变换参数
        transform.transform.translation.x = self.transform_params[0]
        transform.transform.translation.y = self.transform_params[1]
        transform.transform.translation.z = 0.0
        
        # 设置旋转
        transform.transform.rotation = self.yaw_to_quaternion(self.transform_params[2])
        
        # 发布变换
        self.tf_broadcaster.sendTransform(transform)
    
    def publish_dynamic_transform(self, odom_msg, base_x, base_y):
        """发布camera_init到base_link的动态变换"""
        transform = TransformStamped()
        transform.header.stamp = odom_msg.header.stamp
        transform.header.frame_id = 'camera_init'
        transform.child_frame_id = 'base_link'
        
        # translation.xy使用base_x, base_y，translation.z照抄odom
        transform.transform.translation.x = base_x
        transform.transform.translation.y = base_y
        transform.transform.translation.z = odom_msg.pose.pose.position.z
        
        # rotation直接照抄odom
        transform.transform.rotation = odom_msg.pose.pose.orientation
        
        # 发布变换
        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = CoordinateTransformerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 