#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import math
import threading
import sys
import tty
import termios
import os

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initialpose_publisher')
        
        # 发布initialpose话题
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )
        
        # 初始化参数
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0  # 弧度制
        
        # 创建键盘输入线程
        self.input_thread = threading.Thread(target=self.keyboard_input_loop, daemon=True)
        self.input_thread.start()
        
        self.get_logger().info('Initial Pose Publisher initialized')
        self.get_logger().info('Controls:')
        self.get_logger().info('  X: Set X position')
        self.get_logger().info('  Y: Set Y position')
        self.get_logger().info('  A: Set Yaw angle (degrees)')
        self.get_logger().info('  P: Publish current initialpose')
        self.get_logger().info('  S: Show current values')
        self.get_logger().info('  R: Reset to (0,0,0)')
        self.get_logger().info('  q: Quit')
        self.print_status()
        
    def print_status(self):
        """打印当前状态"""
        print(f"\r当前设置: x={self.x:.3f}, y={self.y:.3f}, yaw={math.degrees(self.yaw):.2f}° | 输入: ", end='', flush=True)
        
    def publish_initialpose(self):
        """发布initialpose"""
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_init'
        
        # 设置位置
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        
        # 设置姿态（四元数）
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        
        # 设置协方差矩阵
        msg.pose.covariance = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.01]
        
        self.initialpose_pub.publish(msg)
        self.get_logger().info(f'Published initialpose: x={self.x:.3f}, y={self.y:.3f}, yaw={math.degrees(self.yaw):.2f}°')
        
    def input_number(self, prompt):
        """输入数字"""
        print(f"\n{prompt}: ", end='', flush=True)
        input_str = ""
        
        # 保存终端设置
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            # 设置终端为原始模式
            tty.setraw(sys.stdin.fileno())
            
            while True:
                char = sys.stdin.read(1)
                if char in ['\r', '\n']:  # Enter
                    break
                elif char == '\x7f':  # Backspace
                    if input_str:
                        input_str = input_str[:-1]
                        print('\b \b', end='', flush=True)
                elif char == 'q' or char == 'Q':  # 退出
                    return None
                elif char.isdigit() or char == '.' or char == '-':
                    input_str += char
                    print(char, end='', flush=True)
                    
        finally:
            # 恢复终端设置
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            
        try:
            return float(input_str)
        except ValueError:
            print(f"\nInvalid number: {input_str}")
            return None
        
    def keyboard_input_loop(self):
        """键盘输入循环"""
        # 保存终端设置
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            # 设置终端为原始模式
            tty.setraw(sys.stdin.fileno())
            
            while True:
                # 读取一个字符
                char = sys.stdin.read(1)
                
                if char == 'x' or char == 'X':  # 设置X位置
                    value = self.input_number("Enter X position")
                    if value is not None:
                        self.x = value
                        print(f"\nX position set to: {self.x:.3f}")
                    self.print_status()
                    
                elif char == 'y' or char == 'Y':  # 设置Y位置
                    value = self.input_number("Enter Y position")
                    if value is not None:
                        self.y = value
                        print(f"\nY position set to: {self.y:.3f}")
                    self.print_status()
                    
                elif char == 'a' or char == 'A':  # 设置Yaw角度
                    value = self.input_number("Enter Yaw angle (degrees)")
                    if value is not None:
                        self.yaw = math.radians(value)
                        print(f"\nYaw angle set to: {math.degrees(self.yaw):.2f}°")
                    self.print_status()
                    
                elif char == 'p' or char == 'P':  # 发布initialpose
                    print(f"\nPublishing initialpose...")
                    self.publish_initialpose()
                    self.print_status()
                    
                elif char == 's' or char == 'S':  # 显示当前值
                    print(f"\nCurrent values:")
                    print(f"  X: {self.x:.3f}")
                    print(f"  Y: {self.y:.3f}")
                    print(f"  Yaw: {math.degrees(self.yaw):.2f}°")
                    self.print_status()
                    
                elif char == 'r' or char == 'R':  # 重置
                    self.x = 0.0
                    self.y = 0.0
                    self.yaw = 0.0
                    print(f"\nReset to (0, 0, 0°)")
                    self.print_status()
                    
                elif char == 'q' or char == 'Q':  # 退出
                    print("\nExiting...")
                    break
                    
        except KeyboardInterrupt:
            pass
        finally:
            # 恢复终端设置
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 