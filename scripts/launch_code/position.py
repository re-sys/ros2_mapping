import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class PositionNode(Node):
    def __init__(self):
        super().__init__('position_node')
        self.get_logger().info('Position node initialized')
        self.publisher_ = self.create_publisher(PoseStamped, '/visual_pose', 10)
        self.timer_ = self.create_timer(0.1, self.publish_position)
        
    def publish_position(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = 1.0
        msg.pose.position.y = 1.0
        msg.pose.position.z = 1.0
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PositionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()  