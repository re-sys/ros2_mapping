#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class ManualTransformPublisher(Node):
    def __init__(self):
        super().__init__('manual_transform_publisher')
        self.publisher = self.create_publisher(
            Float32MultiArray, 
            '/transform_params', 
            10
        )
        self.get_logger().info('Manual Transform Publisher initialized')
        self.get_logger().info('Enter three numbers (x y yaw) separated by spaces:')
        
        # 创建定时器检查输入
        self.timer = self.create_timer(0.1, self.check_input)
    
    def check_input(self):
        """检查用户输入并发布"""
        try:
            # 获取用户输入
            user_input = input("> ")
            if not user_input:
                return
                
            # 解析输入
            values = [float(x) for x in user_input.split()]
            
            if len(values) != 3:
                self.get_logger().warn("Please enter exactly three numbers")
                return
                
            # 创建并发布消息
            msg = Float32MultiArray()
            msg.data = values
            self.publisher.publish(msg)
            self.get_logger().info(f"Published: x={values[0]}, y={values[1]}, yaw={values[2]}")
            
        except ValueError:
            self.get_logger().warn("Invalid input. Please enter numbers only.")
        except Exception as e:
            self.get_logger().error(f"Error: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = ManualTransformPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
