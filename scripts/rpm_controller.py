#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import threading
import sys
import tty
import termios
import os

class RPMController(Node):
    def __init__(self):
        super().__init__('rpm_controller')
        
        # 初始化参数
        self.current_rpm = 0.0  # 当前RPM值
        self.manual_rpm = 1000.0  # 手动设置的RPM值
        self.rpm_step = 20.0  # RPM调节步长
        self.manual_mode = False  # 手动模式状态
        self.running = True  # 运行标志
        
        # 订阅shoot_rpm话题
        self.shoot_rpm_sub = self.create_subscription(
            Vector3,
            '/shoot_rpm',
            self.shoot_rpm_callback,
            10
        )
        
        # 发布set_rpm话题
        self.set_rpm_pub = self.create_publisher(
            Vector3,
            '/set_rpm',
            10
        )
        
        # 创建键盘输入线程
        self.input_thread = threading.Thread(target=self.keyboard_input_loop, daemon=True)
        self.input_thread.start()
        
        self.get_logger().info('RPM Controller initialized')
        self.get_logger().info('Controls:')
        self.get_logger().info('  Up/Down arrows: Adjust RPM')
        self.get_logger().info('  Enter: Send RPM value')
        self.get_logger().info('  Space: Toggle manual/auto mode')
        self.get_logger().info('  S: Sync manual RPM with current RPM')
        self.get_logger().info('  q: Quit')
        self.print_status()
        
    def shoot_rpm_callback(self, msg):
        """处理shoot_rpm消息"""
        self.current_rpm = msg.x
        
        # 只有在自动模式下才同步手动RPM值
        if not self.manual_mode:
            self.manual_rpm = self.current_rpm
            
        self.print_status()
        
    def print_status(self):
        """打印当前状态"""
        mode_str = "MANUAL" if self.manual_mode else "AUTO"
        print(f"\r当前RPM: {self.current_rpm:.1f} | 手动RPM: {self.manual_rpm:.1f} | 模式: {mode_str} | 输入: ", end='', flush=True)
        
    def send_rpm_command(self, manual_mode, rpm_value=None):
        """发送RPM命令"""
        msg = Vector3()
        if manual_mode:
            msg.x = 1.0  # 手动模式
            msg.y = rpm_value if rpm_value is not None else self.manual_rpm
            msg.z = 0.0
            self.get_logger().info(f'Sending manual RPM command: {msg.y}')
        else:
            msg.x = 0.0  # 自动模式
            msg.y = 0.0
            msg.z = 0.0
            self.get_logger().info('Sending auto RPM command')
            
        self.set_rpm_pub.publish(msg)
        
    def keyboard_input_loop(self):
        """键盘输入循环"""
        # 保存终端设置
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            # 设置终端为原始模式
            tty.setraw(sys.stdin.fileno())
            
            while self.running:
                # 读取一个字符
                char = sys.stdin.read(1)
                
                if char == '\x1b':  # ESC序列开始
                    if not self.running:
                        break
                    next_char = sys.stdin.read(1)
                    if next_char == '[':  # 方向键序列
                        if not self.running:
                            break
                        arrow = sys.stdin.read(1)
                        if arrow == 'A':  # 上箭头
                            self.manual_rpm += self.rpm_step
                            print(f"\nRPM increased to: {self.manual_rpm:.1f}")
                        elif arrow == 'B':  # 下箭头
                            self.manual_rpm -= self.rpm_step
                            if self.manual_rpm < 0:
                                self.manual_rpm = 0
                            print(f"\nRPM decreased to: {self.manual_rpm:.1f}")
                        self.print_status()
                        
                elif char == '\r' or char == '\n':  # Enter键
                    print(f"\nSending RPM: {self.manual_rpm:.1f}")
                    self.send_rpm_command(True, self.manual_rpm)
                    self.manual_mode = True
                    # 发送后，手动RPM值保持当前设置值，不跳变
                    self.print_status()
                    
                elif char == ' ':  # 空格键
                    self.manual_mode = not self.manual_mode
                    if self.manual_mode:
                        print(f"\nSwitched to MANUAL mode, RPM: {self.manual_rpm:.1f}")
                        self.send_rpm_command(True, self.manual_rpm)
                    else:
                        print("\nSwitched to AUTO mode")
                        # 切换到自动模式时，手动RPM值与当前RPM值同步
                        self.manual_rpm = self.current_rpm
                        self.send_rpm_command(False)
                    self.print_status()
                    
                elif char == 's' or char == 'S':  # 同步RPM值
                    self.manual_rpm = self.current_rpm
                    print(f"\nSynced manual RPM to current RPM: {self.manual_rpm:.1f}")
                    self.print_status()
                    
                elif char == 'q' or char == 'Q':  # 退出
                    print("\nExiting...")
                    self.running = False
                    break
                    
                elif char.isdigit() or char == '.' or char == '-':
                    # 数字输入模式
                    print(f"\nEnter RPM value (current: {self.manual_rpm:.1f}): ", end='', flush=True)
                    input_str = char
                    while self.running:
                        next_char = sys.stdin.read(1)
                        if next_char in ['\r', '\n']:  # Enter
                            try:
                                new_rpm = float(input_str)
                                if new_rpm >= 0:
                                    self.manual_rpm = new_rpm
                                    print(f"\nRPM set to: {self.manual_rpm:.1f}")
                                else:
                                    print(f"\nInvalid RPM value: {input_str}")
                            except ValueError:
                                print(f"\nInvalid number: {input_str}")
                            break
                        elif next_char == '\x7f':  # Backspace
                            if input_str:
                                input_str = input_str[:-1]
                                print('\b \b', end='', flush=True)
                        elif next_char.isdigit() or next_char == '.' or next_char == '-':
                            input_str += next_char
                            print(next_char, end='', flush=True)
                        elif next_char == 'q' or next_char == 'Q':  # 在数字输入模式下也可以退出
                            self.running = False
                            break
                    if self.running:  # 只有在没有退出时才打印状态
                        self.print_status()
                    
        except KeyboardInterrupt:
            pass
        finally:
            # 恢复终端设置
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            
    def cleanup(self):
        """清理函数"""
        # 发送自动模式命令
        self.send_rpm_command(False)

def main(args=None):
    rclpy.init(args=args)
    node = RPMController()
    
    try:
        # 等待键盘输入线程结束
        while node.running:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received, exiting...")
        node.running = False
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 