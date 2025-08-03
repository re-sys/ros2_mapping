
import rclpy
from rclpy.impl.rcutils_logger import Throttle
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist, Vector3
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster
import math

class OdomToMapConverter(Node):
    def __init__(self):
        super().__init__('odom_to_map_converter')
        
        # 初始化坐标变换参数 (x, y, yaw)
        self.transform_params = [0.0, 0.0, 0.0]  # [x_offset, y_offset, yaw_rotation]
        self.offset = -0.35
        self.linear_offset=-0.15
        self.last_rpm_value = 0.0
        self.x_seg_low = 2.6
        self.x_seg_high = 3.5
        self.tol=0.015
        self.user_tol=0.05
        self.left_reach_times=3
        # RPM控制模式
        self.manual_rpm_mode = False  # 手动RPM模式
        self.manual_rpm_value = 0.0   # 手动设置的RPM值
        
        # 初始化变换后的坐标
        self.transformed_x = 0.0
        self.transformed_y = 0.0
        self.transformed_yaw = 0.0
        
        # 初始化map原点
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_origin_yaw = 0.0
        self.map_origin_set = False
        
        # 初始化目标位姿
        self.goal_yaw = 0.0
        self.has_goal = False
        
        # PID控制参数
        self.kp_angular = 4.0
        self.kd_angular = 0.0
        self.max_angular_vel = 2.2
        
        
        # 订阅initialpose话题
        self.initialpose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initialpose_callback,
            10
        )
        
        # 订阅Odometry话题
        self.odom_sub = self.create_subscription(
            Odometry,
            '/Odometry',
            self.odom_callback,
            10
        )
        
        # 订阅shoot_btn话题
        self.shoot_btn_sub = self.create_subscription(
            Bool,
            '/shoot_btn',
            self.shoot_btn_callback,
            10
        )
        
        # 订阅set_rpm话题
        self.set_rpm_sub = self.create_subscription(
            Vector3,
            '/set_rpm',
            self.set_rpm_callback,
            10
        )
        
        # 发布cmd_vel
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # 发布shoot_rpm
        self.rpm_pub = self.create_publisher(
            Vector3,
            '/shoot_rpm',
            10
        )
        
        # 发布transformedxyyaw
        self.transformed_pub = self.create_publisher(
            Vector3,
            '/transformedxyyaw',
            10
        )
        
        # 发布transformed_params
        self.transformed_params_pub = self.create_publisher(
            Vector3,
            '/transformed_params',
            10
        )
        
        # TF广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 发布静态变换
        self.publish_static_transform()
        
        # 创建控制定时器，10Hz
        self.control_timer = self.create_timer(0.05, self.control_callback)
        
        self.get_logger().info('Odom to Map Converter initialized')
        self.get_logger().info('Transform parameters: x={:.3f}, y={:.3f}, yaw={:.3f}'.format(
            self.transform_params[0], self.transform_params[1], self.transform_params[2]))
        self.get_logger().info(f"self.offseset:{self.offset:.2f}")

    def shoot_btn_callback(self, msg):
        """处理shoot_btn消息，设置目标角度"""
        if msg.data:  # 当收到true时
            # 计算当前距离
            distance = math.sqrt(self.transformed_x**2 + self.transformed_y**2)
            
            # 检查是否在直接发射模式范围内
            if distance < self.x_seg_low:
                self.get_logger().info(f'Direct shoot mode! Distance: {distance:.2f}m < {self.x_seg_low}m')
                # 直接发射模式：不转动，直接设置RPM为1500
                rpm_msg = Vector3()
                rpm_msg.x = 1500.0
                rpm_msg.y = 1500.0
                rpm_msg.z = 1500.0
                self.rpm_pub.publish(rpm_msg)
                return
            
            # 正常模式：将当前goal_pose的角度设为目标
            self.get_goal()
            self.left_reach_times=3
            error_yaw = self.calculate_error_yaw()
            self.last_error_yaw = error_yaw
            if abs(error_yaw)<self.user_tol:
                self.get_logger().info('User mode!')
                self.tol = self.user_tol
            else:
                self.get_logger().info('Normal mode!')
                self.tol = 0.015

    def set_rpm_callback(self, msg):
        """处理set_rpm消息，设置RPM控制模式"""
        if msg.x == 1.0:  # 手动RPM模式
            self.manual_rpm_mode = True
            self.manual_rpm_value = msg.y
            self.get_logger().info(f'Manual RPM mode enabled, RPM set to: {self.manual_rpm_value}')
        elif msg.x == 0.0:  # 自动RPM模式
            self.manual_rpm_mode = False
            self.get_logger().info('Auto RPM mode enabled')

    def get_goal(self):
        self.goal_yaw = math.atan2(-self.transformed_y, -self.transformed_x)
        self.has_goal = True
        self.get_logger().info('Shoot button pressed! Target yaw set to: {:.3f}°'.format(
            math.degrees(self.goal_yaw)))

    def calculate_error_yaw(self):
        if (self.transformed_yaw-self.goal_yaw)>math.pi:
            self.transformed_yaw -= 2*math.pi
        elif (self.transformed_yaw-self.goal_yaw)<-math.pi:
            self.transformed_yaw += 2*math.pi
        error_yaw = self.goal_yaw - self.transformed_yaw
        return error_yaw

    def compute_cmd_and_publish(self):           
        # 计算角度误差
        error_yaw = self.calculate_error_yaw()

        # 创建cmd_vel消息
        cmd_vel = Twist()
        
        # 角速度控制（基于yaw误差）
        cmd_vel.angular.z = self.kp_angular * error_yaw + self.kd_angular * (error_yaw - self.last_error_yaw)
        # self.get_logger().info(f'diff_yaw: {error_yaw-self.last_error_yaw}',throttle_duration_sec=0.4)
        self.last_error_yaw = error_yaw
        
        # 限制角速度
        cmd_vel.angular.z = max(-self.max_angular_vel, min(self.max_angular_vel, cmd_vel.angular.z))
        
        # 发布cmd_vel
        self.cmd_vel_pub.publish(cmd_vel)
        return None
    def control_callback(self):
        """旋转控制回调函数，控制到指定角度"""
        if not self.has_goal:
            return
        # 检查是否到达目标角度
        error_yaw = self.calculate_error_yaw()
        if abs(error_yaw) < self.tol:  # 约3度误差
            self.left_reach_times-=1
            self.cmd_vel_pub.publish(Twist())
            self.get_logger().info(f'Left reach times: {self.left_reach_times}')
        else:
            self.left_reach_times=3
            self.get_logger().info(f'error_yaw: {error_yaw}',throttle_duration_sec=1.0)
            self.compute_cmd_and_publish()

        if self.left_reach_times==0:
            self.get_logger().info('Target angle reached! Current yaw: {:.3f}°, Target yaw: {:.3f}°'.format(
                math.degrees(self.transformed_yaw), math.degrees(self.goal_yaw)))
            # 计算到map原点的距离并发布RPM值
            distance = math.sqrt(self.transformed_x**2 + self.transformed_y**2)

            if distance >6.0:
                self.get_logger().info('Distance too far!')
                self.has_goal=False
                return
            # 根据模式选择RPM值
            if self.manual_rpm_mode:
                rpm_value = self.manual_rpm_value
                self.get_logger().info(f'Using manual RPM: {rpm_value}')
            else:
                rpm_value = self.calculate_rpm(distance+self.offset)
                self.get_logger().info(f'Using calculated RPM: {rpm_value} for distance: {distance:.2f}m')
                
            
            rpm_msg = Vector3()
            rpm_msg.x = rpm_value
            rpm_msg.y = rpm_value
            rpm_msg.z = rpm_value
            self.rpm_pub.publish(rpm_msg)
            self.has_goal = False

    def calculate_rpm(self,x):
        if  x > self.x_seg_low and x<self.x_seg_high:
            RPM = -54.428954 * x**2 + 860.393555 * x + 29.221919
            print('use quad')
        else:
            RPM = 430.9047619047621*(x+self.linear_offset-self.offset)+ 809.6428571428569  
            print('use linear')
        # RPM = 425*(x-2.6)+ 1850
        # RPM = 436.262883 * x + 834.703062
        return RPM
    def inverse_transform(self,x,y,yaw):
        yaw = -yaw
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        rotated_x = -(x * cos_yaw - y * sin_yaw)
        rotated_y = -(x * sin_yaw + y * cos_yaw)
        return rotated_x,rotated_y,yaw
    def initialpose_callback(self, msg):
        """处理initialpose消息，设置map坐标系原点"""
        # 提取initialpose的位置和姿态
        self.map_origin_x = msg.pose.pose.position.x
        self.map_origin_y = msg.pose.pose.position.y
        self.map_origin_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
        
        # 计算从map原点到当前odom坐标系的变换参数
        rotated_x,rotated_y,yaw = self.inverse_transform(self.map_origin_x,self.map_origin_y,self.map_origin_yaw)
        self.transform_params[0] = rotated_x
        self.transform_params[1] = rotated_y
        self.transform_params[2] = yaw
        params_msg = Vector3()
        params_msg.x = self.map_origin_x
        params_msg.y = self.map_origin_y
        params_msg.z = self.map_origin_yaw
        self.transformed_params_pub.publish(params_msg)
        self.map_origin_set = True
        
        # 更新静态变换
        self.publish_static_transform()
        
        self.get_logger().info('Map origin set from initialpose: x={:.3f}, y={:.3f}, yaw={:.3f}'.format(
            self.map_origin_x, self.map_origin_y, self.map_origin_yaw))
        self.get_logger().info('Updated transform parameters: x={:.3f}, y={:.3f}, yaw={:.3f}'.format(
            self.transform_params[0], self.transform_params[1], self.transform_params[2]))

    def odom_callback(self, msg):
        """处理Odometry消息并进行坐标转换"""
        # 提取原始位姿
        original_x = msg.pose.pose.position.x
        original_y = msg.pose.pose.position.y
        
        # 从四元数提取yaw角
        original_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
        # 应用坐标变换
        # 1. 先旋转
        cos_yaw = math.cos(self.transform_params[2])
        sin_yaw = math.sin(self.transform_params[2])
        
        rotated_x = original_x * cos_yaw - original_y * sin_yaw
        rotated_y = original_x * sin_yaw + original_y * cos_yaw
        
        # 2. 再平移
        self.transformed_x = rotated_x + self.transform_params[0]
        self.transformed_y = rotated_y + self.transform_params[1]
        
        # 3. 更新yaw角
        self.transformed_yaw = original_yaw + self.transform_params[2]
        
        # 打印转换后的坐标信息
        self.print_transformed_coordinates(original_x, original_y, original_yaw)
        
        # 发布TF变换
        self.publish_static_transform()
        
        # 发布transformedxyyaw
        transformed_msg = Vector3()
        transformed_msg.x = self.transformed_x
        transformed_msg.y = self.transformed_y
        transformed_msg.z = self.transformed_yaw
        self.transformed_pub.publish(transformed_msg)
        
        

    def print_transformed_coordinates(self, original_x, original_y, original_yaw):
        """打印转换后的坐标信息"""
        # print("=" * 60)
        # print("坐标转换结果:")
        # print(f"原始Odometry坐标: x={original_x:.3f}, y={original_y:.3f}, yaw={math.degrees(original_yaw):.2f}°")
        # print(f"转换后Map坐标:    x={self.transformed_x:.3f}, y={self.transformed_y:.3f}, yaw={math.degrees(self.transformed_yaw):.2f}°")
        self.get_logger().info(f"当前距离:         {math.sqrt(self.transformed_x**2 + self.transformed_y**2):.2f}m",throttle_duration_sec=5.0)
        # print(f"变换参数:         x_offset={self.transform_params[0]:.3f}, y_offset={self.transform_params[1]:.3f}, yaw_rotation={math.degrees(self.transform_params[2]):.2f}°")
        # print(f"当前计算的转速:   {self.calculate_rpm(math.sqrt(self.transformed_x**2 + self.transformed_y**2)):.2f}RPM")
        # print(f"上次计算的转速:   {self.last_rpm_value:.2f}RPM")
        # if self.map_origin_set:
        #     print(f"Map原点:          x={self.map_origin_x:.3f}, y={self.map_origin_y:.3f}, yaw={math.degrees(self.map_origin_yaw):.2f}°")
        # else:
        #     print("Map原点:          未设置")
        # print("=" * 60)

    def quaternion_to_yaw(self, quaternion):
        """从四元数提取yaw角"""
        # 使用四元数的z分量计算yaw角
        siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw
    
    def yaw_to_quaternion(self, yaw):
        """从yaw角创建四元数"""
        from geometry_msgs.msg import Quaternion
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = math.sin(yaw / 2.0)
        quaternion.w = math.cos(yaw / 2.0)
        return quaternion
    
    def publish_static_transform(self):
        """发布map到camera_init的静态变换"""
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'camera_init'
        
        # 设置变换参数
        transform.transform.translation.x = self.transform_params[0]
        transform.transform.translation.y = self.transform_params[1]
        transform.transform.translation.z = 0.0
        
        # 设置旋转
        transform.transform.rotation = self.yaw_to_quaternion(self.transform_params[2])
        
        # 发布变换
        self.tf_broadcaster.sendTransform(transform)
    

def main(args=None):
    rclpy.init(args=args)
    node = OdomToMapConverter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 