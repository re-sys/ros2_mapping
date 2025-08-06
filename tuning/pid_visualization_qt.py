#!/usr/bin/env python3

import sys
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QLineEdit, QPushButton, 
                             QGroupBox, QGridLayout, QTextEdit, QSlider)
from PyQt5.QtCore import QTimer, pyqtSignal, QThread
from PyQt5.QtGui import QPainter, QPen, QBrush, QColor, QFont
import numpy as np
import time

class PlotWidget(QWidget):
    """实时曲线图显示组件"""
    
    def __init__(self, size=800):
        super().__init__()
        self.size = size
        self.setFixedSize(size, size)
        
        # 数据存储
        self.error_yaw_data = []
        self.p_data = []
        self.i_data = []
        self.d_data = []
        self.output_data = []
        self.max_points = 150
        
        # 颜色定义
        self.colors = {
            'error_yaw': QColor(255, 0, 0),    # 红色
            'p': QColor(0, 255, 0),            # 绿色
            'i': QColor(0, 0, 255),            # 蓝色
            'd': QColor(255, 165, 0),          # 橙色
            'output': QColor(128, 0, 128)      # 紫色
        }
        
        # 标签
        self.labels = {
            'error_yaw': 'Error Yaw',
            'p': 'P Component',
            'i': 'I Component', 
            'd': 'D Component',
            'output': 'Output'
        }
        
        # 数据范围 - 初始化为更合理的范围
        self.y_min = -1.0
        self.y_max = 1.0
        
        # 数据采集状态
        self.is_collecting = False
        self.data_count = 0
        
        # 性能指标
        self.overshoot = 0.0
        self.rise_time = 0.0
        self.settling_time = 0.0
        self.steady_state_error = 0.0
        self.performance_calculated = False
        
    def start_collection(self):
        """开始数据采集"""
        self.clear_data()
        self.is_collecting = True
        self.data_count = 0
        
    def add_data(self, error_yaw, p, i, d, output):
        """添加新的数据点"""
        if not self.is_collecting:
            return
            
        self.error_yaw_data.append(error_yaw)
        self.p_data.append(p)
        self.i_data.append(i)
        self.d_data.append(d)
        self.output_data.append(output)
        
        self.data_count += 1
        
        # 达到150个数据点后停止采集
        if self.data_count >= self.max_points:
            self.is_collecting = False
            # 计算性能指标
            self.calculate_performance_metrics()
        
        # 更新Y轴范围 - 使用实际数据的范围
        all_data = self.error_yaw_data + self.p_data + self.i_data + self.d_data + self.output_data
        if all_data:
            data_min = min(all_data)
            data_max = max(all_data)
            data_range = data_max - data_min
            
            # 确保有足够的显示范围
            if data_range < 0.1:
                data_range = 0.1
                data_center = (data_max + data_min) / 2
                self.y_min = data_center - data_range / 2
                self.y_max = data_center + data_range / 2
            else:
                # 添加一些边距
                margin = data_range * 0.1
                self.y_min = data_min - margin
                self.y_max = data_max + margin
        
        self.update()
    
    def calculate_performance_metrics(self):
        """计算性能指标：超调量、上升时间、稳定时间"""
        if len(self.error_yaw_data) < 10:
            return
        
        # 采样时间间隔（60Hz）
        dt = 1.0 / 60.0
        
        # 计算稳态值（最后20个点的平均值）
        steady_state_samples = min(20, len(self.error_yaw_data) // 4)
        steady_state_value = np.mean(self.error_yaw_data[-steady_state_samples:])
        
        # 计算稳态误差
        self.steady_state_error = abs(steady_state_value)
        
        # 计算超调量
        max_value = max(self.error_yaw_data)
        min_value = min(self.error_yaw_data)
        
        if abs(max_value) > abs(min_value):
            # 正超调
            self.overshoot = (max_value - steady_state_value) / abs(steady_state_value) * 100 if abs(steady_state_value) > 1e-6 else 0
        else:
            # 负超调
            self.overshoot = (min_value - steady_state_value) / abs(steady_state_value) * 100 if abs(steady_state_value) > 1e-6 else 0
        
        # 计算上升时间（从10%到90%的响应时间）
        initial_value = self.error_yaw_data[0]
        target_range = abs(steady_state_value - initial_value)
        
        if target_range < 1e-6:
            self.rise_time = 0.0
        else:
            # 找到10%和90%的时间点
            threshold_10 = initial_value + 0.1 * target_range
            threshold_90 = initial_value + 0.9 * target_range
            
            t_10 = None
            t_90 = None
            
            for i, value in enumerate(self.error_yaw_data):
                if t_10 is None and value >= threshold_10:
                    t_10 = i * dt
                if t_90 is None and value >= threshold_90:
                    t_90 = i * dt
                    break
            
            if t_10 is not None and t_90 is not None:
                self.rise_time = t_90 - t_10
            else:
                self.rise_time = 0.0
        
        # 计算稳定时间（误差进入±5%稳态值范围内的时间）
        tolerance = 0.05 * abs(steady_state_value)
        if tolerance < 1e-6:
            tolerance = 0.01  # 最小容差
        
        settling_time = 0.0
        for i, value in enumerate(self.error_yaw_data):
            if abs(value - steady_state_value) <= tolerance:
                # 检查后续点是否都在容差范围内
                all_settled = True
                for j in range(i, min(i + 10, len(self.error_yaw_data))):
                    if abs(self.error_yaw_data[j] - steady_state_value) > tolerance:
                        all_settled = False
                        break
                if all_settled:
                    settling_time = i * dt
                    break
        
        self.settling_time = settling_time
        self.performance_calculated = True
        
        print(f"性能指标计算结果:")
        print(f"  超调量: {self.overshoot:.2f}%")
        print(f"  上升时间: {self.rise_time:.3f}s")
        print(f"  稳定时间: {self.settling_time:.3f}s")
        print(f"  稳态误差: {self.steady_state_error:.4f}")
    
    def clear_data(self):
        """清空所有数据"""
        self.error_yaw_data = []
        self.p_data = []
        self.i_data = []
        self.d_data = []
        self.output_data = []
        self.is_collecting = False
        self.data_count = 0
        # 重置性能指标
        self.overshoot = 0.0
        self.rise_time = 0.0
        self.settling_time = 0.0
        self.steady_state_error = 0.0
        self.performance_calculated = False
        # 重置Y轴范围
        self.y_min = -1.0
        self.y_max = 1.0
        self.update()
    
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # 设置背景
        painter.fillRect(self.rect(), QColor(240, 240, 240))
        
        # 绘制网格
        self.draw_grid(painter)
        
        # 绘制坐标轴
        self.draw_axes(painter)
        
        # 绘制图例
        self.draw_legend(painter)
        
        # 绘制曲线
        self.draw_curves(painter)
        
        # 绘制状态信息
        self.draw_status(painter)
    
    def draw_grid(self, painter):
        """绘制网格"""
        # 主网格线
        painter.setPen(QPen(QColor(180, 180, 180), 1))
        
        # 水平网格线
        for i in range(11):
            y = self.height() - 50 - i * (self.height() - 100) // 10
            painter.drawLine(50, y, self.width() - 50, y)
        
        # 垂直网格线
        for i in range(11):
            x = 50 + i * (self.width() - 100) // 10
            painter.drawLine(x, 50, x, self.height() - 50)
        
        # 次网格线（更细的网格）
        painter.setPen(QPen(QColor(220, 220, 220), 1))
        
        # 水平次网格线
        for i in range(21):
            y = self.height() - 50 - i * (self.height() - 100) // 20
            painter.drawLine(50, y, self.width() - 50, y)
        
        # 垂直次网格线
        for i in range(21):
            x = 50 + i * (self.width() - 100) // 20
            painter.drawLine(x, 50, x, self.height() - 50)
    
    def draw_axes(self, painter):
        """绘制坐标轴"""
        painter.setPen(QPen(QColor(0, 0, 0), 2))
        
        # X轴
        painter.drawLine(50, self.height() - 50, self.width() - 50, self.height() - 50)
        # Y轴
        painter.drawLine(50, 50, 50, self.height() - 50)
        
        # 绘制坐标轴标签
        painter.setFont(QFont("Arial", 12, QFont.Bold))
        
        # X轴标签 - 时间
        x_label = "Time (Data Points)"
        x_label_width = painter.fontMetrics().width(x_label)
        painter.drawText(self.width() // 2 - x_label_width // 2, self.height() - 10, x_label)
        
        # Y轴标签 - 向左偏移很多像素
        y_label = "Value"
        y_label_width = painter.fontMetrics().width(y_label)
        # 向左偏移80像素，避免与Y轴重合
        painter.drawText(10, self.height() // 2 + y_label_width // 2, y_label)
        
        # 绘制Y轴刻度
        painter.setFont(QFont("Arial", 10))
        for i in range(11):
            y = self.height() - 50 - i * (self.height() - 100) // 10
            value = self.y_min + i * (self.y_max - self.y_min) / 10
            # 刻度值也向左偏移，避免与Y轴重合
            painter.drawText(10, y + 5, f"{value:.3f}")
        
        # 绘制X轴刻度（数据点数量）
        if self.error_yaw_data:
            max_points = len(self.error_yaw_data)
            # 每25个数据点显示一个刻度
            step = max(1, max_points // 10)
            for i in range(0, max_points, step):
                if i < max_points:
                    x = 50 + i * (self.width() - 100) / (max_points - 1) if max_points > 1 else 50
                    painter.drawText(x - 15, self.height() - 30, str(i))
            painter.drawText(0, y + 5, f"{value:.3f}")
        
        # 绘制X轴刻度
        painter.setFont(QFont("Arial", 10))
        for i in range(11):
            x = 50 + i * (self.width() - 100) // 10
            # 计算时间值（基于数据点数量，每个数据点间隔1/60秒，因为更新频率是60Hz）
            time_value = i * (self.max_points - 1) / 60.0 / 10
            painter.drawText(x - 20, self.height() - 30, f"{time_value:.2f}")
    
    def draw_legend(self, painter):
        """绘制图例"""
        painter.setFont(QFont("Arial", 10))
        legend_y = 20
        legend_x = self.width() - 200
        
        for i, (key, color) in enumerate(self.colors.items()):
            painter.setPen(QPen(color, 3))
            painter.drawLine(legend_x, legend_y + i * 25, legend_x + 25, legend_y + i * 25)
            painter.setPen(QPen(QColor(0, 0, 0), 1))
            painter.drawText(legend_x + 30, legend_y + i * 25 + 5, self.labels[key])
    
    def draw_status(self, painter):
        """绘制状态信息"""
        painter.setFont(QFont("Arial", 12, QFont.Bold))
        
        if self.is_collecting:
            status_text = f"Collecting: {self.data_count}/{self.max_points}"
            painter.setPen(QColor(0, 128, 0))
        else:
            if self.data_count >= self.max_points:
                status_text = f"Complete: {self.data_count} points collected"
                painter.setPen(QColor(0, 0, 128))
            else:
                status_text = "Waiting for shoot_btn message..."
                painter.setPen(QColor(128, 128, 128))
        
        painter.drawText(60, 30, status_text)
        
        # 显示性能指标
        if self.performance_calculated:
            painter.setFont(QFont("Arial", 10))
            painter.setPen(QColor(0, 0, 0))
            
            # 性能指标显示位置
            metrics_x = 60
            metrics_y = 60
            
            painter.drawText(metrics_x, metrics_y, f"Overshoot: {self.overshoot:.1f}%")
            painter.drawText(metrics_x, metrics_y + 20, f"Rise Time: {self.rise_time:.3f}s")
            painter.drawText(metrics_x, metrics_y + 40, f"Settling Time: {self.settling_time:.3f}s")
            painter.drawText(metrics_x, metrics_y + 60, f"Steady State Error: {self.steady_state_error:.4f}")
    
    def draw_curves(self, painter):
        """绘制曲线"""
        if not self.error_yaw_data:
            return
        
        # 绘制每条曲线
        curves = [
            (self.error_yaw_data, self.colors['error_yaw']),
            (self.p_data, self.colors['p']),
            (self.i_data, self.colors['i']),
            (self.d_data, self.colors['d']),
            (self.output_data, self.colors['output'])
        ]
        
        for data, color in curves:
            if len(data) < 2:
                continue
                
            painter.setPen(QPen(color, 3))
            
            # 计算坐标转换
            x_scale = (self.width() - 100) / (len(data) - 1) if len(data) > 1 else 1
            y_scale = (self.height() - 100) / (self.y_max - self.y_min)
            
            # 绘制线段
            for i in range(len(data) - 1):
                x1 = 50 + i * x_scale
                y1 = self.height() - 50 - (data[i] - self.y_min) * y_scale
                x2 = 50 + (i + 1) * x_scale
                y2 = self.height() - 50 - (data[i + 1] - self.y_min) * y_scale
                
                painter.drawLine(int(x1), int(y1), int(x2), int(y2))

class ROSThread(QThread):
    """ROS线程，处理ROS通信"""
    
    # 定义信号
    shoot_triggered = pyqtSignal()
    
    def __init__(self):
        super().__init__()
        self.node = None
        self.running = True
    
    def run(self):
        rclpy.init()
        self.node = PIDVisualizationQtNode()
        
        # 连接信号
        self.node.shoot_triggered_signal = self.shoot_triggered
        
        while self.running and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
        
        if self.node:
            self.node.destroy_node()
        rclpy.shutdown()
    
    def stop(self):
        self.running = False

class PIDVisualizationQtNode(Node):
    """ROS2节点，处理话题订阅和发布"""
    
    def __init__(self):
        super().__init__('pid_visualization_qt_node')
        
        # 订阅pid_info话题（从pid_rotate_shoot.py发布）
        self.pid_info_sub = self.create_subscription(
            Float32MultiArray,
            '/pid_info',
            self.pid_info_callback,
            10
        )
        
        # 订阅shoot_btn话题
        self.shoot_btn_sub = self.create_subscription(
            Bool,
            '/shoot_btn',
            self.shoot_btn_callback,
            10
        )
        
        # 发布shoot_btn话题
        self.shoot_btn_pub = self.create_publisher(
            Bool,
            '/shoot_btn',
            10
        )
        
        # 发布pid_params话题
        self.pid_params_pub = self.create_publisher(
            Float32MultiArray,
            '/pid_params',
            10
        )
        
        # 发布targetgoal话题
        self.target_goal_pub = self.create_publisher(
            Float32,
            '/targetgoal',
            10
        )
        
        # 数据存储
        self.error_yaw = 0.0
        self.p_value = 0.0
        self.i_value = 0.0
        self.d_value = 0.0
        self.output_value = 0.0
        
        # 时间间隔跟踪
        self.last_pid_info_time = None
        self.pid_info_count = 0
        
        # 信号用于通知UI开始采集
        self.shoot_triggered_signal = None
        
        self.get_logger().info("PID可视化Qt节点已启动")
    
    def pid_info_callback(self, msg):
        """处理pid_info话题消息（从pid_rotate_shoot.py发布）"""
        current_time = time.time()
        
        if len(msg.data) >= 5:
            self.error_yaw = msg.data[0]  # current_error_yaw
            self.p_value = msg.data[1]    # p_term
            self.i_value = msg.data[2]    # i_term
            self.d_value = msg.data[3]    # d_term
            self.output_value = msg.data[4]  # output
            
            # 计算时间间隔
            if self.last_pid_info_time is not None:
                time_interval = current_time - self.last_pid_info_time
                frequency = 1.0 / time_interval if time_interval > 0 else 0
                self.pid_info_count += 1
                
                # 每10次消息打印一次频率信息
                if self.pid_info_count % 10 == 0:
                    self.get_logger().info(f"PID信息接收频率: {frequency:.1f}Hz (间隔: {time_interval*1000:.1f}ms)")
            
            self.last_pid_info_time = current_time
    
    def shoot_btn_callback(self, msg):
        """处理shoot_btn话题消息"""
        if msg.data:  # 如果shoot_btn为true
            self.get_logger().info("接收到shoot_btn为true，开始数据采集")
            # 通过信号通知UI开始采集
            if self.shoot_triggered_signal:
                self.shoot_triggered_signal.emit()
    
    def publish_shoot_btn(self):
        """发布shoot_btn消息"""
        msg = Bool()
        msg.data = True
        self.shoot_btn_pub.publish(msg)
        self.get_logger().info("发布shoot_btn消息")
    
    def publish_pid_params(self, kp, ki, kd, output_min, output_max, integral_min, integral_max, deadband, sample_time=0.05):
        """发布PID参数"""
        msg = Float32MultiArray()
        msg.data = [kp, ki, kd, output_min, output_max, integral_min, integral_max, deadband, sample_time]
        self.pid_params_pub.publish(msg)
        self.get_logger().info(f"发布PID参数: Kp={kp}, Ki={ki}, Kd={kd}, output_min={output_min}, output_max={output_max}, integral_min={integral_min}, integral_max={integral_max}, deadband={deadband}, sample_time={sample_time}")
    
    def publish_target_goal(self, offset_degrees):
        """发布目标角度偏置"""
        import math
        offset_radians = math.radians(offset_degrees)
        msg = Float32()
        msg.data = offset_radians
        self.target_goal_pub.publish(msg)
        self.get_logger().info(f"发布目标角度偏置: {offset_degrees:.1f}° ({offset_radians:.3f} rad)")

class MainWindow(QMainWindow):
    """主窗口"""
    
    def __init__(self):
        super().__init__()
        self.ros_thread = None
        self.plot_widget = None
        self.update_timer = None
        
        self.init_ui()
        self.start_ros_thread()
    
    def init_ui(self):
        """初始化用户界面"""
        self.setWindowTitle("PID参数可视化器")
        self.setGeometry(100, 100, 1400, 1000)
        
        # 创建中央部件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 主布局
        main_layout = QVBoxLayout(central_widget)
        
        # 创建水平布局来居中显示图表
        chart_layout = QHBoxLayout()
        
        # 添加左侧空白
        chart_layout.addStretch()
        
        # 创建曲线图组件
        self.plot_widget = PlotWidget(800)
        chart_layout.addWidget(self.plot_widget)
        
        # 添加右侧空白
        chart_layout.addStretch()
        
        main_layout.addLayout(chart_layout)
        
        # 控制面板
        control_layout = QHBoxLayout()
        
        # Shoot按钮组
        shoot_group = QGroupBox("Shoot Control")
        shoot_layout = QVBoxLayout()
        
        # 目标角度偏置输入框
        shoot_layout.addWidget(QLabel("目标角度偏置 (度):"))
        self.target_goal_input = QLineEdit("0.0")
        self.target_goal_input.setMinimumHeight(30)
        self.target_goal_input.setPlaceholderText("输入角度偏置，正值顺时针，负值逆时针")
        shoot_layout.addWidget(self.target_goal_input)
        
        self.shoot_button = QPushButton("Shoot")
        self.shoot_button.setMinimumHeight(40)
        self.shoot_button.clicked.connect(self.on_shoot_clicked)
        shoot_layout.addWidget(self.shoot_button)
        
        shoot_group.setLayout(shoot_layout)
        control_layout.addWidget(shoot_group)
        
        # PID参数输入组 - 分为两列
        pid_group = QGroupBox("PID Parameters")
        pid_layout = QGridLayout()
        
        # 第一列：基本PID参数
        pid_layout.addWidget(QLabel("Kp:"), 0, 0)
        self.kp_input = QLineEdit("4.0")
        self.kp_input.setMinimumHeight(30)
        pid_layout.addWidget(self.kp_input, 0, 1)
        
        pid_layout.addWidget(QLabel("Ki:"), 1, 0)
        self.ki_input = QLineEdit("0.1")
        self.ki_input.setMinimumHeight(30)
        pid_layout.addWidget(self.ki_input, 1, 1)
        
        pid_layout.addWidget(QLabel("Kd:"), 2, 0)
        self.kd_input = QLineEdit("0.5")
        self.kd_input.setMinimumHeight(30)
        pid_layout.addWidget(self.kd_input, 2, 1)
        
        # 第二列：输出限制参数
        pid_layout.addWidget(QLabel("Output Min:"), 0, 2)
        self.output_min_input = QLineEdit("-2.2")
        self.output_min_input.setMinimumHeight(30)
        pid_layout.addWidget(self.output_min_input, 0, 3)
        
        pid_layout.addWidget(QLabel("Output Max:"), 1, 2)
        self.output_max_input = QLineEdit("2.2")
        self.output_max_input.setMinimumHeight(30)
        pid_layout.addWidget(self.output_max_input, 1, 3)
        
        # 第三列：积分限制参数
        pid_layout.addWidget(QLabel("Integral Min:"), 2, 2)
        self.integral_min_input = QLineEdit("-1.0")
        self.integral_min_input.setMinimumHeight(30)
        pid_layout.addWidget(self.integral_min_input, 2, 3)
        
        pid_layout.addWidget(QLabel("Integral Max:"), 3, 0)
        self.integral_max_input = QLineEdit("1.0")
        self.integral_max_input.setMinimumHeight(30)
        pid_layout.addWidget(self.integral_max_input, 3, 1)
        
        # 死区参数
        pid_layout.addWidget(QLabel("Deadband:"), 3, 2)
        self.deadband_input = QLineEdit("0.001")
        self.deadband_input.setMinimumHeight(30)
        pid_layout.addWidget(self.deadband_input, 3, 3)
        
        # 发送按钮
        self.send_pid_button = QPushButton("Send PID")
        self.send_pid_button.setMinimumHeight(40)
        self.send_pid_button.clicked.connect(self.on_send_pid_clicked)
        pid_layout.addWidget(self.send_pid_button, 4, 0, 1, 4)
        
        pid_group.setLayout(pid_layout)
        control_layout.addWidget(pid_group)
        
        # 状态显示组
        status_group = QGroupBox("Status")
        status_layout = QVBoxLayout()
        
        self.status_label = QLabel("Ready - Waiting for shoot_btn message")
        self.status_label.setMinimumHeight(40)
        self.status_label.setStyleSheet("QLabel { font-size: 14px; font-weight: bold; }")
        status_layout.addWidget(self.status_label)
        
        status_group.setLayout(status_layout)
        control_layout.addWidget(status_group)
        
        # 性能指标显示组
        performance_group = QGroupBox("Performance Metrics")
        performance_layout = QVBoxLayout()
        
        self.overshoot_label = QLabel("Overshoot: --")
        self.rise_time_label = QLabel("Rise Time: --")
        self.settling_time_label = QLabel("Settling Time: --")
        self.steady_error_label = QLabel("Steady State Error: --")
        
        for label in [self.overshoot_label, self.rise_time_label, self.settling_time_label, self.steady_error_label]:
            label.setStyleSheet("QLabel { font-size: 12px; }")
            performance_layout.addWidget(label)
        
        performance_group.setLayout(performance_layout)
        control_layout.addWidget(performance_group)
        
        main_layout.addLayout(control_layout)
        
        # 设置定时器用于更新曲线图 - 60Hz更新频率
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_plot)
        self.update_timer.start(16)  # 约60Hz (1000ms/60 ≈ 16.67ms)
    
    def start_ros_thread(self):
        """启动ROS线程"""
        self.ros_thread = ROSThread()
        # 连接信号到槽函数
        self.ros_thread.shoot_triggered.connect(self.on_shoot_triggered)
        self.ros_thread.start()
    
    def on_shoot_clicked(self):
        """处理Shoot按钮点击"""
        if self.ros_thread and self.ros_thread.node:
            try:
                # 获取目标角度偏置
                offset_degrees = float(self.target_goal_input.text())
                
                # 发布目标角度偏置
                self.ros_thread.node.publish_target_goal(offset_degrees)
                
                # 发布发射按钮信号
                self.ros_thread.node.publish_shoot_btn()
                
                # 开始数据采集
                self.plot_widget.start_collection()
                self.status_label.setText(f"Shoot triggered with offset: {offset_degrees:.1f}°, collecting 150 data points...")
                
            except ValueError:
                self.status_label.setText("错误：请输入有效的角度偏置数字")
                return
            self.ros_thread.node.publish_shoot_btn()
            self.status_label.setText("Shoot button clicked, waiting for shoot_btn message...")
    
    def on_shoot_triggered(self):
        """处理接收到shoot_btn为true的信号"""
        self.plot_widget.start_collection()
        self.status_label.setText("Shoot triggered, collecting 150 data points...")
    
    def on_send_pid_clicked(self):
        """处理发送PID参数按钮点击"""
        try:
            kp = float(self.kp_input.text())
            ki = float(self.ki_input.text())
            kd = float(self.kd_input.text())
            output_min = float(self.output_min_input.text())
            output_max = float(self.output_max_input.text())
            integral_min = float(self.integral_min_input.text())
            integral_max = float(self.integral_max_input.text())
            deadband = float(self.deadband_input.text())
            
            if self.ros_thread and self.ros_thread.node:
                self.ros_thread.node.publish_pid_params(kp, ki, kd, output_min, output_max, integral_min, integral_max, deadband)
                self.status_label.setText(f"PID sent: Kp={kp}, Ki={ki}, Kd={kd}")
        except ValueError:
            self.status_label.setText("Invalid PID parameters")
    
    def update_plot(self):
        """更新曲线图"""
        if self.ros_thread and self.ros_thread.node:
            self.plot_widget.add_data(
                self.ros_thread.node.error_yaw,
                self.ros_thread.node.p_value,
                self.ros_thread.node.i_value,
                self.ros_thread.node.d_value,
                self.ros_thread.node.output_value
            )
            
            # 更新性能指标显示
            if self.plot_widget.performance_calculated:
                self.overshoot_label.setText(f"Overshoot: {self.plot_widget.overshoot:.1f}%")
                self.rise_time_label.setText(f"Rise Time: {self.plot_widget.rise_time:.3f}s")
                self.settling_time_label.setText(f"Settling Time: {self.plot_widget.settling_time:.3f}s")
                self.steady_error_label.setText(f"Steady State Error: {self.plot_widget.steady_state_error:.4f}")
    
    def closeEvent(self, event):
        """窗口关闭事件"""
        if self.ros_thread:
            self.ros_thread.stop()
            self.ros_thread.wait()
        event.accept()

def main():
    """主函数"""
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main() 