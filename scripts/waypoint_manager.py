#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import sys
import os

class WaypointManager(Node):
    """路径点管理器，用于发送保存、加载和清空命令"""
    
    def __init__(self):
        super().__init__('waypoint_manager')
        
        # 发布者
        self.save_pub = self.create_publisher(Bool, '/save_waypoints', 10)
        self.load_pub = self.create_publisher(Bool, '/load_waypoints', 10)
        self.clear_pub = self.create_publisher(Bool, '/clear_waypoints', 10)
        
        # 获取文件路径信息
        current_dir = os.path.dirname(os.path.abspath(__file__))
        waypoints_file = os.path.join(current_dir, "saved_waypoints.yaml")
        
        self.get_logger().info('Waypoint Manager initialized')
        self.get_logger().info(f'Waypoints file path: {waypoints_file}')
        self.get_logger().info('Available commands:')
        self.get_logger().info('  save   - Save current waypoints to file')
        self.get_logger().info('  load   - Load waypoints from file')
        self.get_logger().info('  clear  - Clear all waypoints')
        self.get_logger().info('  help   - Show this help message')
    
    def save_waypoints(self):
        """保存路径点"""
        msg = Bool()
        msg.data = True
        self.save_pub.publish(msg)
        self.get_logger().info('Sent save command')
    
    def load_waypoints(self):
        """加载路径点"""
        msg = Bool()
        msg.data = True
        self.load_pub.publish(msg)
        self.get_logger().info('Sent load command')
    
    def clear_waypoints(self):
        """清空路径点"""
        msg = Bool()
        msg.data = True
        self.clear_pub.publish(msg)
        self.get_logger().info('Sent clear command')
    
    def show_help(self):
        """显示帮助信息"""
        self.get_logger().info('Usage:')
        self.get_logger().info('  python3 waypoint_manager.py save   - Save waypoints')
        self.get_logger().info('  python3 waypoint_manager.py load   - Load waypoints')
        self.get_logger().info('  python3 waypoint_manager.py clear  - Clear waypoints')
        self.get_logger().info('  python3 waypoint_manager.py help   - Show help')

def main(args=None):
    rclpy.init(args=args)
    
    manager = WaypointManager()
    
    if len(sys.argv) < 2:
        manager.show_help()
        return
    
    command = sys.argv[1].lower()
    
    try:
        if command == 'save':
            manager.save_waypoints()
        elif command == 'load':
            manager.load_waypoints()
        elif command == 'clear':
            manager.clear_waypoints()
        elif command == 'help':
            manager.show_help()
        else:
            manager.get_logger().error(f'Unknown command: {command}')
            manager.show_help()
        
        # 等待消息发布
        rclpy.spin_once(manager, timeout_sec=1.0)
        
    except KeyboardInterrupt:
        pass
    finally:
        manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 