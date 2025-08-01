#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist, Vector3, PoseStamped
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster
import math

class OdomToMapConverter(Node):
    def __init__(self):
        super().__init__('odom_to_map_converter')
        
        # 初始化坐标变换参数 (x, y, yaw)
        self.transform_params = [0.0, 0.0, 0.0]  # [x_offset, y_offset, yaw_rotation]
        self.offset = 0.35
        self.last_rpm_value = 0.0
        self.error_yaw_integral = 0.0
        # 初始化变换后的坐标
        self.transformed_x = 0.0
        self.transformed_y = 0.0
        self.transformed_yaw = 0.0
        # 初始化visual_pose位置
        self.visual_pose_x = 0.0
        self.visual_pose_y = 0.0
        self.visual_pose_z = 0.0

        # 初始化map原点
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_origin_yaw = 0.0
        self.map_origin_set = False
        
        # 初始化目标位姿
        self.goal_yaw = 0.0
        self.has_goal = False
        
        # PID控制参数
        # self.kp_angular = 2.0
        # self.kp_integral = 0.1
        # self.max_angular_vel = 1.5
        self.kp_angular = 1.0
        self.kp_integral = 0.05
        self.max_angular_vel = 1.0
        
        # # 订阅initialpose话题
        # self.initialpose_sub = self.create_subscription(
        #     PoseWithCovarianceStamped,
        #     '/initialpose',
        #     self.initialpose_callback,
        #     10
        # )
        
        # 订阅visual_pose话题
        self.visual_pose_sub = self.create_subscription(
            PoseStamped,
            '/visual_pose',
            self.visual_pose_callback,
            10
        )
        # # 订阅Odometry话题
        # self.odom_sub = self.create_subscription(
        #     Odometry,
        #     '/Odometry',
        #     self.odom_callback,
        #     10
        # )
        
        # 订阅shoot_btn话题
        self.shoot_btn_sub = self.create_subscription(
            Bool,
            '/shoot_btn',
            self.shoot_btn_callback,
            10
        )
        
        # 发布cmd_vel
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # 发布shoot_rpm
        self.rpm_pub = self.create_publisher(
            Vector3,
            '/shoot_rpm',
            10
        )
        
        # TF广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 发布静态变换
        self.publish_static_transform()
        
        # 创建控制定时器，10Hz
        self.control_timer = self.create_timer(0.1, self.control_callback)
        
        self.get_logger().info('Odom to Map Converter initialized')
        self.get_logger().info('Transform parameters: x={:.3f}, y={:.3f}, yaw={:.3f}'.format(
            self.transform_params[0], self.transform_params[1], self.transform_params[2]))

    def shoot_btn_callback(self, msg):
        """处理shoot_btn消息，设置目标角度"""
        if msg.data:  # 当收到true时
            # 将当前goal_pose的角度设为目标
            self.goal_yaw = math.atan2(-self.transformed_y, -self.transformed_x)
            self.has_goal = True
            self.get_logger().info('Shoot button pressed! Target yaw set to: {:.3f}°'.format(
                math.degrees(self.goal_yaw)))

    def visual_pose_callback(self, msg):
        """处理visual_pose消息，设置目标角度"""
        self.visual_pose_x = msg.pose.position.x
        self.visual_pose_y = msg.pose.position.y
        self.visual_pose_z = msg.pose.position.z
        # self.get_logger().info('Visual pose set to: x={:.3f}, y={:.3f}, z={:.3f}'.format(
        #     self.visual_pose_x, self.visual_pose_y, self.visual_pose_z))

    def control_callback(self):
        """旋转控制回调函数，控制到指定角度"""
        if not self.has_goal:
            return
            
        # # 计算角度误差
        # if (self.transformed_yaw-self.goal_yaw)>math.pi:
        #     self.transformed_yaw -= 2*math.pi
        # elif (self.transformed_yaw-self.goal_yaw)<-math.pi:
        #     self.transformed_yaw += 2*math.pi
        # error_yaw = self.goal_yaw - self.transformed_yaw
        error_yaw = math.atan2(self.visual_pose_x, self.visual_pose_z)
        self.error_yaw_integral += error_yaw

        self.error_yaw_integral = max(-0.5, min(0.5, self.error_yaw_integral))
        # 创建cmd_vel消息
        cmd_vel = Twist()
        
        # 角速度控制（基于yaw误差）
        cmd_vel.angular.z = self.kp_angular * error_yaw + self.kp_integral * self.error_yaw_integral
        
        # 限制角速度
        cmd_vel.angular.z = max(-self.max_angular_vel, min(self.max_angular_vel, cmd_vel.angular.z))
        
        # 发布cmd_vel
        self.cmd_vel_pub.publish(cmd_vel)
        # self.get_logger().info(f'Published cmd_vel: {cmd_vel.angular.z} for error_yaw: {error_yaw:.3f}°')
        # 检查是否到达目标角度
        if abs(error_yaw) < 0.02:  # 约3度误差
            self.error_yaw_integral = 0.0
            self.get_logger().info('Target angle reached! Current yaw: {:.3f}°, Target yaw: {:.3f}°'.format(
                math.degrees(self.transformed_yaw), math.degrees(self.goal_yaw)))
            
            # 计算到map原点的距离并发布RPM值
            distance = math.sqrt(self.visual_pose_x**2 + self.visual_pose_z**2)
            rpm_value = self.calculate_rpm(distance + self.offset)
            self.get_logger().info(f'Published RPM: {rpm_value} for distance: {distance:.2f}m')
            self.last_rpm_value = rpm_value
            rpm_msg = Vector3()
            rpm_msg.x = rpm_value
            rpm_msg.y = rpm_value
            rpm_msg.z = rpm_value
            self.rpm_pub.publish(rpm_msg)
            self.get_logger().info(f'Published RPM: {rpm_value} for distance: {distance:.2f}m')
            
            self.has_goal = False
        # else:
        #     self.get_logger().info(f"error_integral: {self.error_yaw_integral}")
        #     self.get_logger().info(f'Current yaw: {math.degrees(self.transformed_yaw):.3f}°, Target yaw: {math.degrees(self.goal_yaw):.3f}°')
        #     self.get_logger().info(f'Published cmd_vel: {cmd_vel.angular.z} for error_yaw: {error_yaw:.3f}°')
    def calculate_rpm(self,x):
        # RPM = -26.231*x**2 + 596.720*x +646.734
        RPM = 436.262883 * x + 834.703062
        return RPM
    
    # def initialpose_callback(self, msg):
    #     """处理initialpose消息，设置map坐标系原点"""
    #     # 提取initialpose的位置和姿态
    #     self.map_origin_x = msg.pose.pose.position.x
    #     self.map_origin_y = msg.pose.pose.position.y
    #     self.map_origin_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
        
    #     # 计算从map原点到当前odom坐标系的变换参数
    #     # 这个变换将map坐标系的原点映射到initialpose指定的位置
    #     cos_yaw = math.cos(-self.map_origin_yaw)
    #     sin_yaw = math.sin(-self.map_origin_yaw)

    #     rotated_x = -self.map_origin_x * cos_yaw + self.map_origin_y * sin_yaw
    #     rotated_y = -self.map_origin_x * sin_yaw - self.map_origin_y * cos_yaw

    #     self.transform_params[0] = rotated_x
    #     self.transform_params[1] = rotated_y
    #     self.transform_params[2] = -self.map_origin_yaw
        
    #     self.map_origin_set = True
        
    #     # 更新静态变换
    #     self.publish_static_transform()
        
    #     self.get_logger().info('Map origin set from initialpose: x={:.3f}, y={:.3f}, yaw={:.3f}'.format(
    #         self.map_origin_x, self.map_origin_y, self.map_origin_yaw))
    #     self.get_logger().info('Updated transform parameters: x={:.3f}, y={:.3f}, yaw={:.3f}'.format(
    #         self.transform_params[0], self.transform_params[1], self.transform_params[2]))

    # def odom_callback(self, msg):
    #     """处理Odometry消息并进行坐标转换"""
    #     # 提取原始位姿
    #     original_x = msg.pose.pose.position.x
    #     original_y = msg.pose.pose.position.y
        
    #     # 从四元数提取yaw角
    #     original_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
    #     # 应用坐标变换
    #     # 1. 先旋转
    #     cos_yaw = math.cos(self.transform_params[2])
    #     sin_yaw = math.sin(self.transform_params[2])
        
    #     rotated_x = original_x * cos_yaw - original_y * sin_yaw
    #     rotated_y = original_x * sin_yaw + original_y * cos_yaw
        
    #     # 2. 再平移
    #     self.transformed_x = rotated_x + self.transform_params[0]
    #     self.transformed_y = rotated_y + self.transform_params[1]
        
    #     # 3. 更新yaw角
    #     self.transformed_yaw = original_yaw + self.transform_params[2]
        
    #     # 打印转换后的坐标信息
    #     self.print_transformed_coordinates(original_x, original_y, original_yaw)
        
    #     # 发布TF变换
    #     self.publish_static_transform()

    # def print_transformed_coordinates(self, original_x, original_y, original_yaw):
    #     """打印转换后的坐标信息"""
        # print("=" * 60)
        # # print("坐标转换结果:")
        # # print(f"原始Odometry坐标: x={original_x:.3f}, y={original_y:.3f}, yaw={math.degrees(original_yaw):.2f}°")
        # # print(f"转换后Map坐标:    x={self.transformed_x:.3f}, y={self.transformed_y:.3f}, yaw={math.degrees(self.transformed_yaw):.2f}°")
        # print(f"当前距离:         {math.sqrt(self.transformed_x**2 + self.transformed_y**2):.2f}m")
        # print(f"变换参数:         x_offset={self.transform_params[0]:.3f}, y_offset={self.transform_params[1]:.3f}, yaw_rotation={math.degrees(self.transform_params[2]):.2f}°")
        # print(f"当前计算的转速:   {self.calculate_rpm(math.sqrt(self.transformed_x**2 + self.transformed_y**2)):.2f}RPM")
        # print(f"上次计算的转速:   {self.last_rpm_value:.2f}RPM")
        # # if self.map_origin_set:
        # #     print(f"Map原点:          x={self.map_origin_x:.3f}, y={self.map_origin_y:.3f}, yaw={math.degrees(self.map_origin_yaw):.2f}°")
        # # else:
        # #     print("Map原点:          未设置")
        # print("=" * 60)

    def quaternion_to_yaw(self, quaternion):
        """从四元数提取yaw角"""
        # 使用四元数的z分量计算yaw角
        siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw
    
    def yaw_to_quaternion(self, yaw):
        """从yaw角创建四元数"""
        from geometry_msgs.msg import Quaternion
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
    

def main(args=None):
    rclpy.init(args=args)
    node = OdomToMapConverter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()                                                 