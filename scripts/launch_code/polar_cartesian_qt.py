#!/usr/bin/env python3

import sys
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QLineEdit, QPushButton, 
                             QGroupBox, QGridLayout, QTextEdit, QSlider)
from PyQt5.QtCore import QTimer, pyqtSignal, QThread
from PyQt5.QtGui import QPainter, QPen, QBrush, QColor, QFont
import numpy as np

class CoordinateWidget(QWidget):
    """坐标系显示和交互组件"""
    point_clicked = pyqtSignal(float, float)  # 发送点击的坐标
    
    def __init__(self, size=400):
        super().__init__()
        self.size = size
        self.setFixedSize(size, size)
        self.setMouseTracking(True)
        
        # 当前显示的点
        self.current_point = None
        self.scale = 5.0  # 坐标系缩放（米）
        
    def set_point(self, x, y):
        """设置要显示的点"""
        self.current_point = (x, y)
        self.update()
    
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # 设置坐标系中心
        center_x = self.width() // 2
        center_y = self.height() // 2
        
        # 绘制背景
        painter.fillRect(self.rect(), QColor(240, 240, 240))
        
        # 绘制网格
        self.draw_grid(painter, center_x, center_y)
        
        # 绘制坐标轴
        self.draw_axes(painter, center_x, center_y)
        
        # 绘制当前点
        if self.current_point:
            self.draw_point(painter, center_x, center_y, self.current_point[0], self.current_point[1])
    
    def draw_grid(self, painter, center_x, center_y):
        """绘制网格"""
        painter.setPen(QPen(QColor(200, 200, 200), 1))
        
        # 计算网格间距
        grid_spacing = self.size // (2 * self.scale)
        
        # 绘制垂直线
        for i in range(-int(self.scale), int(self.scale) + 1):
            x = center_x + i * grid_spacing
            painter.drawLine(x, 0, x, self.height())
        
        # 绘制水平线
        for i in range(-int(self.scale), int(self.scale) + 1):
            y = center_y + i * grid_spacing
            painter.drawLine(0, y, self.width(), y)
    
    def draw_axes(self, painter, center_x, center_y):
        """绘制坐标轴"""
        painter.setPen(QPen(QColor(0, 0, 0), 2))
        
        # X轴
        painter.drawLine(0, center_y, self.width(), center_y)
        # Y轴
        painter.drawLine(center_x, 0, center_x, self.height())
        
        # 绘制箭头
        arrow_size = 10
        # X轴箭头
        painter.drawLine(self.width() - arrow_size, center_y - arrow_size//2, 
                        self.width(), center_y)
        painter.drawLine(self.width() - arrow_size, center_y + arrow_size//2, 
                        self.width(), center_y)
        # Y轴箭头
        painter.drawLine(center_x - arrow_size//2, arrow_size, 
                        center_x, 0)
        painter.drawLine(center_x + arrow_size//2, arrow_size, 
                        center_x, 0)
        
        # 绘制标签
        painter.setFont(QFont("Arial", 10))
        painter.drawText(self.width() - 20, center_y - 10, "X")
        painter.drawText(center_x + 5, 15, "Y")
    
    def draw_point(self, painter, center_x, center_y, x, y):
        """绘制点"""
        # 转换坐标
        pixel_x = center_x + (x / self.scale) * (self.size // 2)
        pixel_y = center_y - (y / self.scale) * (self.size // 2)
        
        # 绘制点
        painter.setPen(QPen(QColor(255, 0, 0), 3))
        painter.setBrush(QBrush(QColor(255, 0, 0)))
        painter.drawEllipse(int(pixel_x - 5), int(pixel_y - 5), 10, 10)
        
        # 绘制坐标标签
        painter.setPen(QColor(0, 0, 0))
        painter.setFont(QFont("Arial", 8))
        label = f"({x:.2f}, {y:.2f})"
        painter.drawText(int(pixel_x + 10), int(pixel_y), label)
    
    def mousePressEvent(self, event):
        """鼠标点击事件"""
        if event.button() == 1:  # 左键点击
            center_x = self.width() // 2
            center_y = self.height() // 2
            
            # 转换像素坐标到实际坐标
            pixel_x = event.x() - center_x
            pixel_y = center_y - event.y()
            
            x = (pixel_x / (self.size // 2)) * self.scale
            y = (pixel_y / (self.size // 2)) * self.scale
            
            # 发送点击信号
            self.point_clicked.emit(x, y)

class ROSThread(QThread):
    """ROS节点线程"""
    def __init__(self):
        super().__init__()
        self.node = None
        
    def run(self):
        rclpy.init()
        self.node = PolarCartesianQtNode()
        rclpy.spin(self.node)
        
    def stop(self):
        if self.node:
            self.node.destroy_node()
        rclpy.shutdown()

class PolarCartesianQtNode(Node):
    """ROS节点类"""
    def __init__(self):
        super().__init__('polar_cartesian_qt_node')
        
        # 创建发布者
        self.goal_pub = self.create_publisher(
            PoseStamped, 
            '/goal_pose', 
            10
        )
        
        self.get_logger().info("极坐标转换器Qt节点已启动")
    
    def publish_goal(self, x, y):
        """发布目标位姿"""
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = "map"
        
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.w = 1.0
        
        self.goal_pub.publish(goal_msg)
        self.get_logger().info(f"已发布目标位姿: x={x:.2f}, y={y:.2f}")

class MainWindow(QMainWindow):
    """主窗口"""
    def __init__(self):
        super().__init__()
        self.ros_thread = ROSThread()
        self.ros_thread.start()
        
        self.init_ui()
        
    def init_ui(self):
        """初始化用户界面"""
        self.setWindowTitle("极坐标转换器 - Qt界面")
        self.setGeometry(100, 100, 800, 600)
        
        # 创建中央部件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 创建主布局
        main_layout = QHBoxLayout(central_widget)
        
        # 左侧：坐标系显示
        left_layout = QVBoxLayout()
        
        # 坐标系组件
        self.coord_widget = CoordinateWidget()
        self.coord_widget.point_clicked.connect(self.on_point_clicked)
        left_layout.addWidget(QLabel("坐标系（点击选择位置）"))
        left_layout.addWidget(self.coord_widget)
        
        # 缩放控制
        scale_layout = QHBoxLayout()
        scale_layout.addWidget(QLabel("缩放:"))
        self.scale_slider = QSlider()
        self.scale_slider.setRange(1, 20)
        self.scale_slider.setValue(5)
        self.scale_slider.valueChanged.connect(self.on_scale_changed)
        scale_layout.addWidget(self.scale_slider)
        left_layout.addLayout(scale_layout)
        
        main_layout.addLayout(left_layout)
        
        # 右侧：控制面板
        right_layout = QVBoxLayout()
        
        # 手动输入组
        input_group = QGroupBox("手动输入")
        input_layout = QGridLayout()
        
        input_layout.addWidget(QLabel("极坐标:"), 0, 0)
        self.r_input = QLineEdit("0.0")
        self.theta_input = QLineEdit("0.0")
        input_layout.addWidget(QLabel("距离 (m):"), 1, 0)
        input_layout.addWidget(self.r_input, 1, 1)
        input_layout.addWidget(QLabel("角度 (度):"), 2, 0)
        input_layout.addWidget(self.theta_input, 2, 1)
        
        self.convert_btn = QPushButton("转换并发布")
        self.convert_btn.clicked.connect(self.on_convert_clicked)
        input_layout.addWidget(self.convert_btn, 3, 0, 1, 2)
        
        input_group.setLayout(input_layout)
        right_layout.addWidget(input_group)
        
        # 笛卡尔坐标显示组
        cartesian_group = QGroupBox("笛卡尔坐标")
        cartesian_layout = QGridLayout()
        
        self.x_label = QLabel("0.00")
        self.y_label = QLabel("0.00")
        self.r_label = QLabel("0.00")
        self.theta_label = QLabel("0.00")
        
        cartesian_layout.addWidget(QLabel("X (m):"), 0, 0)
        cartesian_layout.addWidget(self.x_label, 0, 1)
        cartesian_layout.addWidget(QLabel("Y (m):"), 1, 0)
        cartesian_layout.addWidget(self.y_label, 1, 1)
        cartesian_layout.addWidget(QLabel("距离 (m):"), 2, 0)
        cartesian_layout.addWidget(self.r_label, 2, 1)
        cartesian_layout.addWidget(QLabel("角度 (度):"), 3, 0)
        cartesian_layout.addWidget(self.theta_label, 3, 1)
        
        cartesian_group.setLayout(cartesian_layout)
        right_layout.addWidget(cartesian_group)
        
        # 转速映射组
        speed_group = QGroupBox("转速映射")
        speed_layout = QGridLayout()
        
        self.speed_label = QLabel("0.00")
        self.speed_slider = QSlider()
        self.speed_slider.setRange(0, 100)
        self.speed_slider.setValue(50)
        self.speed_slider.valueChanged.connect(self.on_speed_changed)
        
        speed_layout.addWidget(QLabel("目标转速 (RPM):"), 0, 0)
        speed_layout.addWidget(self.speed_label, 0, 1)
        speed_layout.addWidget(QLabel("转速调节:"), 1, 0)
        speed_layout.addWidget(self.speed_slider, 1, 1)
        
        speed_group.setLayout(speed_layout)
        right_layout.addWidget(speed_group)
        
        # 日志显示
        log_group = QGroupBox("日志")
        log_layout = QVBoxLayout()
        self.log_text = QTextEdit()
        self.log_text.setMaximumHeight(150)
        log_layout.addWidget(self.log_text)
        log_group.setLayout(log_layout)
        right_layout.addWidget(log_group)
        
        main_layout.addLayout(right_layout)
        
        # 设置定时器更新显示
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_display)
        self.update_timer.start(100)  # 10Hz更新
        
        self.log("极坐标转换器Qt界面已启动")
    
    def on_point_clicked(self, x, y):
        """坐标系点击事件"""
        self.update_coordinates(x, y)
        self.log(f"点击位置: ({x:.2f}, {y:.2f})")
    
    def on_convert_clicked(self):
        """转换按钮点击事件"""
        try:
            r = float(self.r_input.text())
            theta_deg = float(self.theta_input.text())
            
            # 转换为笛卡尔坐标
            theta_rad = math.radians(theta_deg)
            x = r * math.cos(theta_rad)
            y = r * math.sin(theta_rad)
            
            self.update_coordinates(x, y)
            self.publish_goal(x, y)
            self.log(f"手动输入转换: r={r:.2f}, θ={theta_deg:.2f}° → ({x:.2f}, {y:.2f})")
            
        except ValueError as e:
            self.log(f"输入错误: {str(e)}")
    
    def on_scale_changed(self, value):
        """缩放改变事件"""
        self.coord_widget.scale = float(value)
        self.coord_widget.update()
    
    def on_speed_changed(self, value):
        """转速改变事件"""
        self.speed_label.setText(f"{value:.0f}")
    
    def update_coordinates(self, x, y):
        """更新坐标显示"""
        # 更新坐标系显示
        self.coord_widget.set_point(x, y)
        
        # 更新标签
        self.x_label.setText(f"{x:.2f}")
        self.y_label.setText(f"{y:.2f}")
        
        # 计算极坐标
        r = math.sqrt(x*x + y*y)
        theta_rad = math.atan2(y, x)
        theta_deg = math.degrees(theta_rad)
        
        self.r_label.setText(f"{r:.2f}")
        self.theta_label.setText(f"{theta_deg:.2f}")
        
        # 计算转速（简单映射函数）
        speed = self.distance_to_speed(r)
        self.speed_slider.setValue(int(speed))
        self.speed_label.setText(f"{speed:.0f}")
    
    def distance_to_speed(self, distance):
        """距离到转速的映射函数"""
        # 简单的线性映射：距离越大，转速越高
        # 可以根据实际需求修改这个函数
        min_speed = 10   # 最小转速
        max_speed = 100  # 最大转速
        max_distance = 10.0  # 最大距离
        
        if distance <= 0:
            return min_speed
        elif distance >= max_distance:
            return max_speed
        else:
            # 线性插值
            speed = min_speed + (max_speed - min_speed) * (distance / max_distance)
            return speed
    
    def publish_goal(self, x, y):
        """发布目标位姿到ROS"""
        if self.ros_thread.node:
            self.ros_thread.node.publish_goal(x, y)
    
    def update_display(self):
        """定时更新显示"""
        # 这里可以添加实时更新的逻辑
        pass
    
    def log(self, message):
        """添加日志"""
        self.log_text.append(f"[{QTimer().remainingTime()}] {message}")
        # 滚动到底部
        self.log_text.verticalScrollBar().setValue(
            self.log_text.verticalScrollBar().maximum()
        )
    
    def closeEvent(self, event):
        """窗口关闭事件"""
        self.ros_thread.stop()
        self.ros_thread.wait()
        event.accept()

def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main() 