#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, PointCloud2
from nav_msgs.msg import Odometry
import numpy as np
from scipy.spatial.transform import Rotation as R
import yaml
import os

class ExtrinsicCalibrationChecker(Node):
    def __init__(self):
        super().__init__('extrinsic_calibration_checker')
        
        # 数据存储
        self.imu_rotations = []
        self.odom_rotations = []
        self.timestamps = []
        
        # 订阅者
        self.imu_sub = self.create_subscription(Imu, '/livox/imu', self.imu_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/Odometry', self.odom_callback, 10)
        
        # 校准参数
        self.collecting_data = False
        self.calibration_time = 30.0  # 收集30秒数据
        self.start_time = None
        
        self.get_logger().info("外参标定检查器已启动")
        self.get_logger().info("请执行以下步骤进行外参检查:")
        self.get_logger().info("1. 保持设备静止5秒")
        self.get_logger().info("2. 绕X轴缓慢旋转90度")
        self.get_logger().info("3. 绕Y轴缓慢旋转90度") 
        self.get_logger().info("4. 绕Z轴缓慢旋转90度")
        self.get_logger().info("5. 回到初始位置")
        
        # 定时器
        self.timer = self.create_timer(1.0, self.check_calibration_progress)

    def imu_callback(self, msg):
        if not self.collecting_data:
            return
            
        # 提取四元数
        quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        if np.linalg.norm(quat) > 0.1:  # 确保四元数有效
            self.imu_rotations.append(quat)

    def odom_callback(self, msg):
        if not self.collecting_data:
            # 开始收集数据
            self.collecting_data = True
            self.start_time = self.get_clock().now()
            self.get_logger().info("开始收集标定数据...")
            
        # 提取四元数
        quat = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y, 
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]
        self.odom_rotations.append(quat)
        self.timestamps.append(self.get_clock().now())

    def check_calibration_progress(self):
        if not self.collecting_data or self.start_time is None:
            return
            
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        if elapsed < self.calibration_time:
            remaining = self.calibration_time - elapsed
            self.get_logger().info(f"数据收集中... 剩余时间: {remaining:.1f}秒")
        else:
            self.analyze_calibration()
            self.collecting_data = False

    def analyze_calibration(self):
        if len(self.imu_rotations) < 10 or len(self.odom_rotations) < 10:
            self.get_logger().error("数据不足，无法进行标定分析")
            return
            
        self.get_logger().info("开始分析外参标定...")
        
        # 确保数据长度一致
        min_len = min(len(self.imu_rotations), len(self.odom_rotations))
        imu_rots = np.array(self.imu_rotations[:min_len])
        odom_rots = np.array(self.odom_rotations[:min_len])
        
        # 计算旋转差异
        rotation_errors = []
        for i in range(min_len):
            try:
                # 将四元数转换为旋转矩阵
                imu_rot = R.from_quat(imu_rots[i])
                odom_rot = R.from_quat(odom_rots[i])
                
                # 计算相对旋转
                rel_rot = odom_rot * imu_rot.inv()
                angle_error = rel_rot.magnitude()
                rotation_errors.append(angle_error)
                
            except Exception as e:
                continue
        
        if len(rotation_errors) > 0:
            mean_error = np.mean(rotation_errors)
            std_error = np.std(rotation_errors)
            max_error = np.max(rotation_errors)
            
            print("\n" + "="*50)
            print("外参标定分析结果")
            print("="*50)
            print(f"平均旋转误差: {np.degrees(mean_error):.2f}°")
            print(f"旋转误差标准差: {np.degrees(std_error):.2f}°")
            print(f"最大旋转误差: {np.degrees(max_error):.2f}°")
            
            if mean_error > np.radians(5):
                print("🚨 警告: 外参标定可能不准确!")
                print("建议:")
                print("1. 重新进行LiDAR-IMU外参标定")
                print("2. 使用li_calibration包进行精确标定")
                print("3. 检查IMU和LiDAR的坐标系定义")
                
                # 提供修正建议
                self.suggest_extrinsic_correction(imu_rots, odom_rots)
                
            elif mean_error > np.radians(2):
                print("⚠️  外参标定精度一般，建议优化")
            else:
                print("✅ 外参标定精度良好")
                
        else:
            self.get_logger().error("无法计算旋转误差")

    def suggest_extrinsic_correction(self, imu_rots, odom_rots):
        """建议外参修正值"""
        try:
            # 计算平均相对旋转
            rel_rotations = []
            for i in range(len(imu_rots)):
                imu_rot = R.from_quat(imu_rots[i])
                odom_rot = R.from_quat(odom_rots[i])
                rel_rot = odom_rot * imu_rot.inv()
                rel_rotations.append(rel_rot)
            
            # 计算平均旋转
            if len(rel_rotations) > 0:
                # 简单平均 (可以改进为更robust的方法)
                avg_rot_matrix = np.mean([r.as_matrix() for r in rel_rotations], axis=0)
                avg_rot = R.from_matrix(avg_rot_matrix)
                
                print("\n建议的外参旋转修正:")
                euler = avg_rot.as_euler('xyz', degrees=True)
                print(f"Roll修正: {euler[0]:.2f}°")
                print(f"Pitch修正: {euler[1]:.2f}°") 
                print(f"Yaw修正: {euler[2]:.2f}°")
                
                # 生成修正后的旋转矩阵
                corrected_rot = avg_rot.as_matrix().flatten()
                print("\n建议的extrinsic_R配置:")
                print(f"extrinsic_R: [{corrected_rot[0]:.6f}, {corrected_rot[1]:.6f}, {corrected_rot[2]:.6f},")
                print(f"              {corrected_rot[3]:.6f}, {corrected_rot[4]:.6f}, {corrected_rot[5]:.6f},")
                print(f"              {corrected_rot[6]:.6f}, {corrected_rot[7]:.6f}, {corrected_rot[8]:.6f}]")
                
        except Exception as e:
            self.get_logger().error(f"计算修正值时出错: {e}")

def main():
    rclpy.init()
    checker = ExtrinsicCalibrationChecker()
    
    try:
        rclpy.spin(checker)
    except KeyboardInterrupt:
        print("\n外参检查器已停止")
    finally:
        checker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 