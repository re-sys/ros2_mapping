#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import sys

class RPMCommandSender(Node):
    def __init__(self, rpm_value):
        super().__init__('rpm_command_sender')
        self.publisher = self.create_publisher(Vector3, '/shoot_rpm', 10)
        self.rpm_value = rpm_value
        self.publish_rpm()
        
    def publish_rpm(self):
        msg = Vector3()
        msg.x = self.rpm_value
        msg.y = self.rpm_value
        msg.z = self.rpm_value
        
        self.publisher.publish(msg)
        self.get_logger().info(f'Published RPM command: {self.rpm_value} to all channels')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) < 2:
        print("Usage: ros2 run your_package send_rpm_command.py <rpm_value>")
        return
    
    try:
        rpm_value = float(sys.argv[1])
        node = RPMCommandSender(rpm_value)
        rclpy.spin(node)
    except ValueError:
        print("Error: RPM value must be a number")
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()