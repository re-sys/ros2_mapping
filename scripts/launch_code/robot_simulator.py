#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from std_srvs.srv import SetBool
import math
import numpy as np

class RobotSimulator(Node):
    def __init__(self):
        super().__init__('robot_simulator')
        
        # 机器人状态
        self.x = 0.0  # 当前位置x
        self.y = 0.0  # 当前位置y
        self.yaw = 0.0  # 当前角度
        self.vx = 0.0  # 当前线速度x
        self.vy = 0.0  # 当前线速度y
        self.vz = 0.0  # 当前角速度
        
        # 仿真参数
        self.dt = 0.05  # 仿真时间步长 (20Hz)
        self.max_linear_vel = 2.0  # 最大线速度 m/s
        self.max_angular_vel = 3.0  # 最大角速度 rad/s
        
        # 控制模式标志
        self.global_control_mode = False  # True: 全局控制模式, False: 局部控制模式
        
        # 订阅cmd_vel话题
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # 创建控制模式切换服务
        self.control_mode_service = self.create_service(
            SetBool,
            '/set_control_mode',
            self.set_control_mode_callback
        )
        
        # 发布Odometry话题
        self.odom_pub = self.create_publisher(
            Odometry,
            '/Odometry',
            10
        )
        
        # TF广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 创建定时器，20Hz更新
        self.timer = self.create_timer(self.dt, self.update_simulation)
        
        self.get_logger().info('Robot Simulator initialized')
        self.get_logger().info('Publishing Odometry at 20Hz')
        self.get_logger().info('Listening to /cmd_vel for movement commands')
        self.get_logger().info(f'Control mode: {"Global" if self.global_control_mode else "Local"}')
        
    def cmd_vel_callback(self, msg):
        """处理速度命令"""
        if self.global_control_mode:
            # 全局控制模式：速度方向始终与初始坐标系xy轴重合
            # 直接使用cmd_vel中的x,y速度，不进行坐标系转换
            self.vx = max(-self.max_linear_vel, min(self.max_linear_vel, msg.linear.x))
            self.vy = max(-self.max_linear_vel, min(self.max_linear_vel, msg.linear.y))
            self.vz = max(-self.max_angular_vel, min(self.max_angular_vel, msg.angular.z))
            
            self.get_logger().info(f'Global mode - cmd_vel: linear=({self.vx:.2f}, {self.vy:.2f}), angular={self.vz:.2f}', throttle_duration_sec=1.0)
        else:
            # 局部控制模式：速度方向相对于机器人当前朝向
            # 限制速度范围
            self.vx = max(-self.max_linear_vel, min(self.max_linear_vel, msg.linear.x))
            self.vy = max(-self.max_linear_vel, min(self.max_linear_vel, msg.linear.y))
            self.vz = max(-self.max_angular_vel, min(self.max_angular_vel, msg.angular.z))
            
            self.get_logger().info(f'Local mode - cmd_vel: linear=({self.vx:.2f}, {self.vy:.2f}), angular={self.vz:.2f}', throttle_duration_sec=1.0)
        
    def update_simulation(self):
        """更新仿真状态"""
        if self.global_control_mode:
            # 全局控制模式：直接使用全局坐标系的速度
            self.x += self.vx * self.dt
            self.y += self.vy * self.dt
            self.yaw += self.vz * self.dt
        else:
            # 局部控制模式：速度相对于机器人当前朝向
            self.x += (self.vx * math.cos(self.yaw) - self.vy * math.sin(self.yaw)) * self.dt
            self.y += (self.vx * math.sin(self.yaw) + self.vy * math.cos(self.yaw)) * self.dt
            self.yaw += self.vz * self.dt
        
        # 处理角度环绕
        while self.yaw > math.pi:
            self.yaw -= 2 * math.pi
        while self.yaw < -math.pi:
            self.yaw += 2 * math.pi
            
        # 发布Odometry
        self.publish_odometry()
        
        # 发布TF变换
        self.publish_tf()
        
    def publish_odometry(self):
        """发布里程计信息"""
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'camera_init'
        odom_msg.child_frame_id = 'body'
        
        # 设置位置
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        
        # 设置姿态（四元数）
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        
        # 设置速度
        odom_msg.twist.twist.linear.x = self.vx
        odom_msg.twist.twist.linear.y = self.vy
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = self.vz
        
        
        self.odom_pub.publish(odom_msg)
        
    def publish_tf(self):
        """发布TF变换"""
        # camera_init到body的变换
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'camera_init'
        transform.child_frame_id = 'body'
        
        # 设置平移
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        
        # 设置旋转（四元数）
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = math.sin(self.yaw / 2.0)
        transform.transform.rotation.w = math.cos(self.yaw / 2.0)
        
        # 发布变换
        self.tf_broadcaster.sendTransform(transform)
        
    def get_robot_state(self):
        """获取机器人当前状态"""
        return {
            'x': self.x,
            'y': self.y,
            'yaw': self.yaw,
            'vx': self.vx,
            'vy': self.vy,
            'vz': self.vz
        }
    
    def toggle_control_mode(self):
        """切换控制模式"""
        self.global_control_mode = not self.global_control_mode
        mode_name = "Global" if self.global_control_mode else "Local"
        self.get_logger().info(f'Switched to {mode_name} control mode')
        
    def set_control_mode(self, global_mode):
        """设置控制模式"""
        self.global_control_mode = global_mode
        mode_name = "Global" if self.global_control_mode else "Local"
        self.get_logger().info(f'Set to {mode_name} control mode')
    
    def set_control_mode_callback(self, request, response):
        """服务回调函数：设置控制模式"""
        self.set_control_mode(request.data)
        response.success = True
        response.message = f"Control mode set to {'Global' if self.global_control_mode else 'Local'}"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = RobotSimulator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 