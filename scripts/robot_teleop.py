#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import threading
import sys
import tty
import termios
import os

class RobotTeleop(Node):
    def __init__(self):
        super().__init__('robot_teleop')
        
        # 初始化参数
        self.linear_vel = 0.5  # 线速度
        self.angular_vel = 1.0  # 角速度
        
        # 发布者
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        self.shoot_btn_pub = self.create_publisher(
            Bool,
            '/shoot_btn',
            10
        )
        
        # 创建键盘输入线程
        self.input_thread = threading.Thread(target=self.keyboard_input_loop, daemon=True)
        self.input_thread.start()
        
        self.get_logger().info('Robot Teleop initialized')
        self.get_logger().info('Controls:')
        self.get_logger().info('  WASD: Move robot')
        self.get_logger().info('  QE: Rotate robot')
        self.get_logger().info('  Space: Send shoot_btn=true')
        self.get_logger().info('  Enter: Send shoot_btn=false')
        self.get_logger().info('  X: Stop robot')
        self.get_logger().info('  q: Quit')
        
    def send_cmd_vel(self, linear_x=0.0, linear_y=0.0, angular_z=0.0):
        """发送速度命令"""
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_x
        cmd_vel.linear.y = linear_y
        cmd_vel.linear.z = 0.0
        cmd_vel.angular.x = 0.0
        cmd_vel.angular.y = 0.0
        cmd_vel.angular.z = angular_z
        self.cmd_vel_pub.publish(cmd_vel)
        
    def send_shoot_btn(self, value):
        """发送shoot_btn信号"""
        msg = Bool()
        msg.data = value
        self.shoot_btn_pub.publish(msg)
        self.get_logger().info(f'Sent shoot_btn: {value}')
        
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
                
                if char == 'w':  # 前进
                    self.send_cmd_vel(linear_x=self.linear_vel)
                    print(f"\nMoving forward: {self.linear_vel} m/s")
                elif char == 's':  # 后退
                    self.send_cmd_vel(linear_x=-self.linear_vel)
                    print(f"\nMoving backward: {self.linear_vel} m/s")
                elif char == 'a':  # 左移
                    self.send_cmd_vel(linear_y=self.linear_vel)
                    print(f"\nMoving left: {self.linear_vel} m/s")
                elif char == 'd':  # 右移
                    self.send_cmd_vel(linear_y=-self.linear_vel)
                    print(f"\nMoving right: {self.linear_vel} m/s")
                elif char == 'q':  # 左转
                    self.send_cmd_vel(angular_z=self.angular_vel)
                    print(f"\nRotating left: {self.angular_vel} rad/s")
                elif char == 'e':  # 右转
                    self.send_cmd_vel(angular_z=-self.angular_vel)
                    print(f"\nRotating right: {self.angular_vel} rad/s")
                elif char == 'x':  # 停止
                    self.send_cmd_vel()
                    print("\nStopped")
                elif char == ' ':  # 空格键 - shoot_btn=true
                    self.send_shoot_btn(True)
                elif char == '\r' or char == '\n':  # Enter键 - shoot_btn=false
                    self.send_shoot_btn(False)
                elif char == 'Q':  # 退出
                    print("\nExiting...")
                    break
                    
        except KeyboardInterrupt:
            pass
        finally:
            # 恢复终端设置
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            
    def cleanup(self):
        """清理函数"""
        # 停止机器人
        self.send_cmd_vel()

def main(args=None):
    rclpy.init(args=args)
    node = RobotTeleop()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 