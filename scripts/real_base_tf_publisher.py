#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist
from tf2_ros import TransformBroadcaster
import math
import numpy as np

class TestOdomPublisher(Node):
    def __init__(self):
        super().__init__('test_odom_publisher')
        
        # 初始位姿
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        
        # 创建TF广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 创建Odometry发布器
        self.odom_pub = self.create_publisher(
            Odometry,
            '/Odometry',
            10
        )
        
        # 订阅cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # 创建定时器更新位姿
        self.timer = self.create_timer(0.1, self.update_odom)  # 10Hz
        self.get_logger().info('Test Odom Publisher initialized')
    
    def cmd_vel_callback(self, msg):
        """处理速度指令，更新位姿"""
        # 计算时间间隔
        current_time = self.get_clock().now()
        if not hasattr(self, 'last_update_time'):
            self.last_update_time = current_time
            return
        
        dt = 0.1
        self.last_update_time = current_time
        
        # 更新位姿
        self.x += msg.linear.x * math.cos(self.yaw) * dt - msg.linear.y * math.sin(self.yaw)*dt
        self.y += msg.linear.x * math.sin(self.yaw) * dt + msg.linear.y * math.cos(self.yaw)*dt
        self.yaw += msg.angular.z * dt
        
        # 角度归一化
        while self.yaw > math.pi:
            self.yaw -= 2 * math.pi
        while self.yaw < -math.pi:
            self.yaw += 2 * math.pi
    
    def update_odom(self):
        """更新并发布Odometry和TF"""
        # 创建Odometry消息
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'camera_init'
        odom.child_frame_id = 'body'
        
        # 设置位姿
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # 设置方向（四元数）
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        
        # 发布Odometry
        self.odom_pub.publish(odom)
        
        # 发布TF变换
        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = 'camera_init'
        t.child_frame_id = 'body'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)
        
        # 打印当前位姿

def main(args=None):
    rclpy.init(args=args)
    node = TestOdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 