#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math

class PolarToCartesian(Node):
    def __init__(self):
        super().__init__('polar_to_cartesian_converter')
        
        # 创建发布者，发布目标位姿到/goal_pose话题
        self.goal_pub = self.create_publisher(
            PoseStamped, 
            '/goal_pose', 
            10
        )
        
        # 创建定时器，每秒检查一次用户输入
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info("极坐标转笛卡尔坐标系节点已启动")
        self.get_logger().info("请输入极坐标参数（长度 角度[度]）:")
    
    def timer_callback(self):
        """定时检查用户输入并发布目标位姿"""
        try:
            # 获取用户输入
            user_input = input("> ")
            if not user_input:
                return
                
            # 解析输入（长度和角度）
            parts = user_input.split()
            if len(parts) < 2:
                self.get_logger().warn("请输入两个参数：长度和角度")
                return
                
            r = float(parts[0])  # 长度
            theta_deg = float(parts[1])  # 角度（度）
            
            # 将角度转换为弧度
            theta_rad = math.radians(theta_deg)
            
            # 计算笛卡尔坐标
            x = r * math.cos(theta_rad)
            y = r * math.sin(theta_rad)
            
            # 创建并发布目标位姿消息
            self.publish_goal(x, y)
            self.get_logger().info(f"已发布目标位姿: x={x:.2f}, y={y:.2f}")
            
        except Exception as e:
            self.get_logger().error(f"输入错误: {str(e)}")
    
    def publish_goal(self, x, y):
        """发布目标位姿到/goal_pose话题"""
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = "map"  # 使用map坐标系
        
        # 设置位置
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.position.z = 0.0
        
        # 设置方向（默认为0）
        goal_msg.pose.orientation.w = 1.0
        
        # 发布消息
        self.goal_pub.publish(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PolarToCartesian()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()