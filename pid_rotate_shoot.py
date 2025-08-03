#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist, Vector3
from std_msgs.msg import Bool, Float32
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster
import math
import time
from collections import deque

class AdvancedPIDController:
    """
    简化的PID控制器类
    """
    
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, 
                 output_min=-float('inf'), output_max=float('inf'),
                 integral_min=-float('inf'), integral_max=float('inf'),
                 deadband=0.0, sample_time=0.05):
        """
        初始化PID控制器
        
        Args:
            kp: 比例系数
            ki: 积分系数  
            kd: 微分系数
            output_min: 输出最小值
            output_max: 输出最大值
            integral_min: 积分项最小值
            integral_max: 积分项最大值
            deadband: 死区范围
            sample_time: 采样时间
        """
        # PID参数
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        # 限制参数
        self.output_min = output_min
        self.output_max = output_max
        self.integral_min = integral_min
        self.integral_max = integral_max
        self.deadband = deadband
        self.sample_time = sample_time
        
        # 状态变量
        self.last_error = 0.0
        self.integral = 0.0
        self.last_output = 0.0
        self.last_time = None
        
        # 高级功能参数
        self.anti_windup = True  # 抗积分饱和
        self.derivative_filter = True  # 微分滤波
        self.filter_alpha = 0.1  # 滤波系数
        
        # 历史数据用于滤波
        self.error_history = deque(maxlen=10)
        self.output_history = deque(maxlen=10)
        
        # 自适应参数
        self.adaptive_mode = False
        self.error_threshold = 0.1
        self.adaptive_kp_multiplier = 1.0
        self.first_loop = True
        
    def compute(self, error):
        """
        计算PID输出
        
        Args:
            error: 输入误差值
            
        Returns:
            output: PID控制器输出
            p_term: 比例项
            i_term: 积分项
            d_term: 微分项
        """
        current_time = time.time()
        
        # 死区处理
        if abs(error) < self.deadband:
            error = 0.0
            
        # 自适应参数调整
        if self.adaptive_mode:
            if abs(error) > self.error_threshold:
                self.adaptive_kp_multiplier = min(2.0, self.adaptive_kp_multiplier * 1.1)
            else:
                self.adaptive_kp_multiplier = max(0.5, self.adaptive_kp_multiplier * 0.95)
        
        # 计算时间间隔
        if self.last_time is None:
            dt = self.sample_time
        else:
            dt = current_time - self.last_time
            dt = min(dt, self.sample_time * 2)  # 限制最大时间间隔
            
        # 比例项
        p_term = self.kp * error * (self.adaptive_kp_multiplier if self.adaptive_mode else 1.0)
        
        # 积分项
        if dt > 0:
            self.integral += error * dt
            # 积分限幅
            if self.anti_windup:
                self.integral = max(self.integral_min, min(self.integral_max, self.integral))
        i_term = self.ki * self.integral
        
        # 微分项
        if dt > 0:
            derivative = (error - self.last_error) / dt
            print(f'derivative: {derivative:.4f}')
            # 微分滤波
            # if self.derivative_filter and len(self.error_history) > 0:
            #     derivative = self.filter_alpha * derivative + (1 - self.filter_alpha) * self.last_output
        else:
            derivative = 0.0
        d_term = self.kd * derivative
        if self.first_loop:
            d_term = 0.0
            # print(f'd_term: {d_term:.4f},kd={self.kd:.4f}')
            self.first_loop = False
        
        # 计算总输出
        output = p_term + i_term + d_term
        
        # 输出限幅
        output = max(self.output_min, min(self.output_max, output))
        
        # 更新状态
        self.last_error = error
        self.last_output = output
        self.last_time = current_time
        
        # 更新历史数据
        self.error_history.append(error)
        self.output_history.append(output)
        
        return output, p_term, i_term, d_term
    
    def enable_adaptive_mode(self, enabled=True, threshold=0.1):
        """启用/禁用自适应模式"""
        self.adaptive_mode = enabled
        self.error_threshold = threshold
    
    def enable_derivative_filter(self, enabled=True, alpha=0.1):
        """启用/禁用微分滤波"""
        self.derivative_filter = enabled
        self.filter_alpha = alpha


class PIDRotateShootNode(Node):
    """
    使用PID控制器的旋转射击节点
    """
    
    def __init__(self):
        super().__init__('pid_rotate_shoot_node')
        
        # 初始化坐标变换参数
        self.transform_params = [0.0, 0.0, 0.0]  # [x_offset, y_offset, yaw_rotation]
        self.tol = 0.015
        self.max_check_times = 3
        
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
        
        # 创建PID控制器
        self.pid_controller = AdvancedPIDController(
            kp=4.0, ki=0.1, kd=0.5,
            output_min=-2.2, output_max=2.2,
            integral_min=-1.0, integral_max=1.0,
            deadband=0.001,
            sample_time=0.05
        )
        
        # PID控制状态
        self.current_error_yaw = 0.0
        self.p_term = 0.0
        self.i_term = 0.0
        self.d_term = 0.0
        self.output = 0.0
        
        # 订阅话题
        self.initialpose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initialpose_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/Odometry',
            self.odom_callback,
            10
        )
        
        self.shoot_btn_sub = self.create_subscription(
            Bool,
            '/shoot_btn',
            self.shoot_btn_callback,
            10
        )
        
        # 订阅PID参数调整话题
        self.pid_params_sub = self.create_subscription(
            Float32MultiArray,
            '/pid_params',
            self.pid_params_callback,
            10
        )
        
        # 发布话题
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        

        
        self.transformed_params_pub = self.create_publisher(
            Vector3,
            '/transformed_params',
            10
        )
        
        # 发布PID相关信息（合并到一个话题）
        self.pid_info_pub = self.create_publisher(
            Float32MultiArray,
            '/pid_info',
            10
        )
        
        # TF广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 发布静态变换
        self.publish_static_transform()
        
        # 创建控制定时器，20Hz
        self.control_timer = self.create_timer(0.05, self.control_callback)
        
        self.get_logger().info('PID Rotate Shoot Node initialized')
        self.get_logger().info('PID Parameters: Kp={:.2f}, Ki={:.2f}, Kd={:.2f}'.format(
            self.pid_controller.kp, self.pid_controller.ki, self.pid_controller.kd))
    
    def pid_params_callback(self, msg):
        """处理PID参数调整消息"""
        if len(msg.data) >= 3:
            # 直接赋值PID参数 [kp, ki, kd]
            self.pid_controller.kp = msg.data[0]
            self.pid_controller.ki = msg.data[1]
            self.pid_controller.kd = msg.data[2]
            self.get_logger().info(f'PID parameters updated: Kp={msg.data[0]:.2f}, Ki={msg.data[1]:.2f}, Kd={msg.data[2]:.2f}')
        
        if len(msg.data) >= 6:
            # 直接赋值限制参数 [output_min, output_max, integral_min, integral_max]
            self.pid_controller.output_min = msg.data[3]
            self.pid_controller.output_max = msg.data[4]
            self.pid_controller.integral_min = msg.data[5]
            self.pid_controller.integral_max = msg.data[6] if len(msg.data) > 6 else msg.data[5]
            self.get_logger().info(f'PID limits updated: output[{msg.data[3]:.2f}, {msg.data[4]:.2f}], integral[{msg.data[5]:.2f}, {msg.data[6] if len(msg.data) > 6 else msg.data[5]:.2f}]')
        
        if len(msg.data) >= 8:
            # 直接赋值死区和采样时间
            self.pid_controller.deadband = msg.data[7]
            self.pid_controller.sample_time = msg.data[8] if len(msg.data) > 8 else 0.05
            self.get_logger().info(f'PID deadband: {msg.data[7]:.3f}, sample_time: {msg.data[8] if len(msg.data) > 8 else 0.05:.3f}')
    
    def shoot_btn_callback(self, msg):
        """处理shoot_btn消息，设置目标角度"""
        if msg.data:  # 当收到true时
            # 计算当前距离
            distance = math.sqrt(self.transformed_x**2 + self.transformed_y**2)
            # 正常模式：将当前goal_pose的角度设为目标
            self.get_goal()
            self.pid_controller.first_loop = True
            self.left_reach_times = self.max_check_times
            error_yaw = self.calculate_error_yaw()
            
    
    def get_goal(self):
        """计算目标角度"""
        self.goal_yaw = math.atan2(-self.transformed_y, -self.transformed_x)
        self.has_goal = True
        self.get_logger().info('Shoot button pressed! Target yaw set to: {:.3f}°'.format(
            math.degrees(self.goal_yaw)))
    
    def calculate_error_yaw(self):
        """计算角度误差"""
        # 角度归一化
        current_yaw = self.transformed_yaw
        target_yaw = self.goal_yaw
        
        # 处理角度跨越±π的情况
        while current_yaw - target_yaw > math.pi:
            current_yaw -= 2 * math.pi
        while current_yaw - target_yaw < -math.pi:
            current_yaw += 2 * math.pi
            
        error_yaw = target_yaw - current_yaw
        return error_yaw
    
    def compute_pid_and_publish(self):
        """计算PID输出并发布控制命令"""
        # 计算角度误差
        error_yaw = self.calculate_error_yaw()
        
        # 使用PID控制器计算输出（目标值为0，输入为error_yaw）
        output, p_term, i_term, d_term = self.pid_controller.compute(error_yaw)
        
        # 更新状态变量
        self.current_error_yaw = error_yaw
        self.p_term = p_term
        self.i_term = i_term
        self.d_term = d_term
        self.output = output
        
        # 创建cmd_vel消息
        cmd_vel = Twist()
        cmd_vel.angular.z = output
        
        # 发布cmd_vel
        self.cmd_vel_pub.publish(cmd_vel)
        
        # 发布PID相关信息（合并到一个话题）
        self.publish_pid_info()
        
        return output
    
    def publish_pid_info(self):
        """发布PID相关信息（合并到一个Float32MultiArray话题）"""
        msg = Float32MultiArray()
        msg.data = [
            self.current_error_yaw,  # error_yaw
            self.p_term,             # p_term
            self.i_term,             # i_term
            self.d_term,              # d_term
            self.output              # output
        ]
        self.pid_info_pub.publish(msg)
    
    def control_callback(self):
        """旋转控制回调函数，使用PID控制到指定角度"""
        if not self.has_goal:
            return
            
        # 检查是否到达目标角度
        error_yaw = self.calculate_error_yaw()
        if abs(error_yaw) < self.tol:
            self.left_reach_times -= 1
            self.cmd_vel_pub.publish(Twist())  # 停止
            self.get_logger().info(f'Left reach times: {self.left_reach_times}')
        else:
            self.left_reach_times = 3
            self.get_logger().info(f'error_yaw: {error_yaw:.4f}', throttle_duration_sec=1.0)
            self.compute_pid_and_publish()
        
        if self.left_reach_times == 0:
            self.get_logger().info('Target angle reached! Current yaw: {:.3f}°, Target yaw: {:.3f}°'.format(
                math.degrees(self.transformed_yaw), math.degrees(self.goal_yaw)))
            self.cmd_vel_pub.publish(Twist())
            self.has_goal = False
    
    def inverse_transform(self, x, y, yaw):
        """逆变换"""
        yaw = -yaw
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        rotated_x = -(x * cos_yaw - y * sin_yaw)
        rotated_y = -(x * sin_yaw + y * cos_yaw)
        return rotated_x, rotated_y, yaw
    
    def initialpose_callback(self, msg):
        """处理initialpose消息，设置map坐标系原点"""
        # 提取initialpose的位置和姿态
        self.map_origin_x = msg.pose.pose.position.x
        self.map_origin_y = msg.pose.pose.position.y
        self.map_origin_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
        
        # 计算从map原点到当前odom坐标系的变换参数
        rotated_x, rotated_y, yaw = self.inverse_transform(self.map_origin_x, self.map_origin_y, self.map_origin_yaw)
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
        
        # 发布TF变换
        self.publish_static_transform()
        
        # 发布transformedxyyaw
        transformed_msg = Vector3()
        transformed_msg.x = self.transformed_x
        transformed_msg.y = self.transformed_y
        transformed_msg.z = self.transformed_yaw
        # self.transformed_pub.publish(transformed_msg)
        
        # 打印转换后的坐标信息
        self.print_transformed_coordinates(original_x, original_y, original_yaw)
    
    def print_transformed_coordinates(self, original_x, original_y, original_yaw):
        """打印转换后的坐标信息"""
        self.get_logger().info(f"当前距离: {math.sqrt(self.transformed_x**2 + self.transformed_y**2):.2f}m", throttle_duration_sec=5.0)
    
    def quaternion_to_yaw(self, quaternion):
        """从四元数提取yaw角"""
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
    node = PIDRotateShootNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 