#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, PointCloud2
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
import threading
import time

class PointLIOAnalyzer(Node):
    def __init__(self):
        super().__init__('point_lio_analyzer')
        
        # 数据存储
        self.imu_data = deque(maxlen=1000)
        self.odom_data = deque(maxlen=1000)
        self.lidar_freq = deque(maxlen=50)
        self.imu_freq = deque(maxlen=100)
        
        # 时间戳
        self.last_lidar_time = None
        self.last_imu_time = None
        self.last_odom_time = None
        
        # 统计数据
        self.imu_acc_std = [0, 0, 0]
        self.imu_gyro_std = [0, 0, 0]
        self.drift_distance = 0
        self.last_position = None
        
        # 订阅者
        self.imu_sub = self.create_subscription(Imu, '/livox/imu', self.imu_callback, 10)
        self.lidar_sub = self.create_subscription(PointCloud2, '/livox/lidar', self.lidar_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/Odometry', self.odom_callback, 10)
        self.path_sub = self.create_subscription(Path, '/path', self.path_callback, 10)
        
        # 定时器用于打印统计信息
        self.timer = self.create_timer(2.0, self.print_statistics)
        
        self.get_logger().info("Point-LIO分析器已启动")
        self.get_logger().info("正在监控: /livox/imu, /livox/lidar, /Odometry, /path")

    def imu_callback(self, msg):
        current_time = time.time()
        
        # 计算频率
        if self.last_imu_time is not None:
            freq = 1.0 / (current_time - self.last_imu_time)
            self.imu_freq.append(freq)
        self.last_imu_time = current_time
        
        # 存储IMU数据
        acc = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        gyro = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        
        self.imu_data.append({
            'time': current_time,
            'acc': acc,
            'gyro': gyro
        })
        
        # 计算噪声标准差 (使用最近50个数据点)
        if len(self.imu_data) >= 50:
            recent_acc = np.array([d['acc'] for d in list(self.imu_data)[-50:]])
            recent_gyro = np.array([d['gyro'] for d in list(self.imu_data)[-50:]])
            
            self.imu_acc_std = np.std(recent_acc, axis=0)
            self.imu_gyro_std = np.std(recent_gyro, axis=0)

    def lidar_callback(self, msg):
        current_time = time.time()
        
        # 计算频率
        if self.last_lidar_time is not None:
            freq = 1.0 / (current_time - self.last_lidar_time)
            self.lidar_freq.append(freq)
        self.last_lidar_time = current_time

    def odom_callback(self, msg):
        current_time = time.time()
        self.last_odom_time = current_time
        
        # 提取位置信息
        pos = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ]
        
        # 计算漂移距离
        if self.last_position is not None:
            distance = np.linalg.norm(np.array(pos) - np.array(self.last_position))
            self.drift_distance += distance
        
        self.last_position = pos
        
        self.odom_data.append({
            'time': current_time,
            'position': pos,
            'orientation': [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            ]
        })

    def path_callback(self, msg):
        # 路径回调，可以用于分析轨迹平滑度
        pass

    def print_statistics(self):
        print("\n" + "="*60)
        print("Point-LIO 实时性能分析")
        print("="*60)
        
        # 频率统计
        if len(self.imu_freq) > 0:
            imu_freq_avg = np.mean(self.imu_freq)
            imu_freq_std = np.std(self.imu_freq)
            print(f"📊 IMU频率: {imu_freq_avg:.1f}±{imu_freq_std:.1f} Hz")
            
            if imu_freq_avg < 180:
                print("⚠️  警告: IMU频率过低 (建议>200Hz)")
        else:
            print("❌ 未接收到IMU数据")
            
        if len(self.lidar_freq) > 0:
            lidar_freq_avg = np.mean(self.lidar_freq)
            lidar_freq_std = np.std(self.lidar_freq)
            print(f"📊 LiDAR频率: {lidar_freq_avg:.1f}±{lidar_freq_std:.1f} Hz")
            
            if lidar_freq_avg < 8:
                print("⚠️  警告: LiDAR频率过低 (建议~10Hz)")
        else:
            print("❌ 未接收到LiDAR数据")
        
        # IMU噪声统计
        print(f"\n🔊 IMU噪声水平:")
        print(f"   加速度标准差: [{self.imu_acc_std[0]:.4f}, {self.imu_acc_std[1]:.4f}, {self.imu_acc_std[2]:.4f}] m/s²")
        print(f"   角速度标准差: [{self.imu_gyro_std[0]:.4f}, {self.imu_gyro_std[1]:.4f}, {self.imu_gyro_std[2]:.4f}] rad/s")
        
        # 噪声水平评估
        acc_noise_level = np.max(self.imu_acc_std)
        gyro_noise_level = np.max(self.imu_gyro_std)
        
        if acc_noise_level > 0.1:
            print("⚠️  警告: 加速度噪声过大，可能影响定位精度")
        if gyro_noise_level > 0.01:
            print("⚠️  警告: 角速度噪声过大，可能影响姿态估计")
        
        # 里程计统计
        if len(self.odom_data) > 0:
            print(f"\n🗺️  定位状态:")
            latest_pos = self.odom_data[-1]['position']
            print(f"   当前位置: [{latest_pos[0]:.3f}, {latest_pos[1]:.3f}, {latest_pos[2]:.3f}]")
            print(f"   累计移动距离: {self.drift_distance:.3f} m")
            
            # 检查异常大的位置跳跃
            if len(self.odom_data) >= 2:
                prev_pos = self.odom_data[-2]['position']
                position_jump = np.linalg.norm(np.array(latest_pos) - np.array(prev_pos))
                if position_jump > 1.0:  # 1米的位置跳跃
                    print(f"🚨 警告: 检测到大幅位置跳跃 ({position_jump:.2f}m)")
        else:
            print("❌ 未接收到里程计数据")
        
        # 时间同步检查
        current_time = time.time()
        if self.last_imu_time and (current_time - self.last_imu_time) > 1.0:
            print("⚠️  警告: IMU数据超时")
        if self.last_lidar_time and (current_time - self.last_lidar_time) > 1.0:
            print("⚠️  警告: LiDAR数据超时")
        if self.last_odom_time and (current_time - self.last_odom_time) > 1.0:
            print("⚠️  警告: 里程计数据超时")
        
        print("\n💡 建议:")
        print("   1. 保持设备在启动时静止10秒以上")
        print("   2. 确保环境有足够几何特征")
        print("   3. 避免快速旋转或加速运动")
        print("   4. 检查IMU和LiDAR的外参标定")

def main():
    rclpy.init()
    analyzer = PointLIOAnalyzer()
    
    try:
        rclpy.spin(analyzer)
    except KeyboardInterrupt:
        print("\n分析器已停止")
    finally:
        analyzer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 