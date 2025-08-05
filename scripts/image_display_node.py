#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Int32, Float32
import cv2
import numpy as np
import threading
import time

class ImageDisplayNode(Node):
    """
    图像显示节点
    订阅检测结果和投影图像，在独立线程中显示图像
    """
    
    def __init__(self):
        super().__init__('image_display_node')
        
        # 订阅检测结果
        self.circle_detected_sub = self.create_subscription(
            Bool,
            '/circle_detected',
            self.circle_detected_callback,
            10
        )
        
        self.circle_center_sub = self.create_subscription(
            Point,
            '/circle_center',
            self.circle_center_callback,
            10
        )
        
        self.circle_radius_sub = self.create_subscription(
            Float32,
            '/circle_radius',
            self.circle_radius_callback,
            10
        )
        
        # 订阅投影图像
        self.projection_image_sub = self.create_subscription(
            Image,
            '/projection_image',
            self.projection_image_callback,
            10
        )
        
        # 订阅积累进度
        self.accumulation_progress_sub = self.create_subscription(
            Int32,
            '/accumulation_progress',
            self.accumulation_progress_callback,
            10
        )
        
        # 显示参数
        self.display_scale = 1.0        # 显示缩放比例
        self.window_name = "Point Cloud Circle Detection"
        
        # 创建显示窗口
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 
                        int(600 * self.display_scale), 
                        int(600 * self.display_scale))
        
        # 数据存储
        self.latest_projection_image = None
        self.latest_circle_info = None
        self.latest_accumulation_progress = 0
        self.accumulation_frames = 10  # 总积累帧数
        
        # 线程锁
        self.data_lock = threading.Lock()
        
        # 启动显示线程
        self.display_thread = threading.Thread(target=self.display_loop, daemon=True)
        self.display_thread.start()
        
        # 显示控制
        self.should_display = True
        self.circle_detected = False
        
        self.get_logger().info('图像显示节点已启动')
        
    def circle_detected_callback(self, msg):
        """处理圆形检测结果"""
        with self.data_lock:
            self.circle_detected = msg.data
            if msg.data:
                self.get_logger().info('收到圆形检测结果')
            else:
                self.get_logger().info('未检测到圆形')
    
    def circle_center_callback(self, msg):
        """处理圆心坐标"""
        with self.data_lock:
            if self.latest_circle_info is None:
                self.latest_circle_info = {}
            self.latest_circle_info['center'] = (msg.x, msg.y, msg.z)
    
    def circle_radius_callback(self, msg):
        """处理圆形半径"""
        with self.data_lock:
            if self.latest_circle_info is None:
                self.latest_circle_info = {}
            self.latest_circle_info['radius'] = msg.data
    
    def projection_image_callback(self, msg):
        """处理投影图像"""
        try:
            # 将ROS Image消息转换为OpenCV图像
            height = msg.height
            width = msg.width
            data = np.frombuffer(msg.data, dtype=np.uint8)
            image = data.reshape((height, width))
            
            with self.data_lock:
                self.latest_projection_image = image
                
        except Exception as e:
            self.get_logger().error(f'处理投影图像时出错: {str(e)}')
    
    def accumulation_progress_callback(self, msg):
        """处理积累进度"""
        with self.data_lock:
            self.latest_accumulation_progress = msg.data
    
    def display_loop(self):
        """显示循环，在独立线程中运行"""
        while self.should_display:
            try:
                # 获取当前数据
                with self.data_lock:
                    projection_image = self.latest_projection_image.copy() if self.latest_projection_image is not None else None
                    circle_info = self.latest_circle_info.copy() if self.latest_circle_info is not None else None
                    progress = self.latest_accumulation_progress
                    detected = self.circle_detected
                
                # 创建显示图像
                display_image = self.create_display_image(projection_image, circle_info, progress, detected)
                
                # 显示图像
                if display_image is not None:
                    cv2.imshow(self.window_name, display_image)
                    
                    # 检查按键
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q') or key == 27:  # 'q' 或 ESC
                        self.should_display = False
                        break
                
                # 如果检测到圆形，等待用户确认
                if detected:
                    self.get_logger().info('圆形检测完成，按任意键关闭窗口')
                    cv2.waitKey(0)
                    self.should_display = False
                    break
                
                # 短暂休眠
                time.sleep(0.1)
                
            except Exception as e:
                self.get_logger().error(f'显示循环出错: {str(e)}')
                time.sleep(0.1)
        
        # 清理窗口
        cv2.destroyAllWindows()
    
    def create_display_image(self, projection_image, circle_info, progress, detected):
        """创建显示图像"""
        if projection_image is None:
            # 创建空白图像
            display_image = np.zeros((600, 600, 3), dtype=np.uint8)
            
            # 添加等待信息
            cv2.putText(display_image, "Waiting for point cloud data...", 
                       (50, 300), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            
            return display_image
        
        # 转换为彩色图像
        display_image = cv2.cvtColor(projection_image, cv2.COLOR_GRAY2BGR)
        
        # 添加坐标轴信息
        height, width = display_image.shape[:2]
        
        # 绘制坐标轴
        cv2.line(display_image, (0, height//2), (width, height//2), (0, 255, 0), 1)  # X轴
        cv2.line(display_image, (width//2, 0), (width//2, height), (0, 255, 0), 1)   # Y轴
        
        # 添加积累进度信息
        progress_text = f"Accumulation Progress: {progress}/{self.accumulation_frames}"
        cv2.putText(display_image, progress_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # 如果有检测到圆形，绘制圆形和信息
        if circle_info is not None and detected:
            center = circle_info.get('center')
            radius = circle_info.get('radius')
            
            if center is not None and radius is not None:
                center_x, center_y, center_z = center
                
                # 将世界坐标转换为像素坐标（简化处理）
                # 这里需要根据实际的投影参数进行调整
                pixel_center_x = int((center_x - 2.5) / 0.01)  # 假设x范围从2.5开始
                pixel_center_y = int((center_y - (-0.3)) / 0.01)  # 假设y范围从-0.3开始
                pixel_radius = int(radius / 0.01)
                
                # 检查边界
                if (0 <= pixel_center_x < width and 0 <= pixel_center_y < height and 
                    pixel_radius > 0 and pixel_radius < min(width, height)//2):
                    
                    # 绘制圆形
                    cv2.circle(display_image, (pixel_center_x, pixel_center_y), pixel_radius, (0, 0, 255), 2)
                    
                    # 绘制圆心
                    cv2.circle(display_image, (pixel_center_x, pixel_center_y), 3, (255, 0, 0), -1)
                    
                    # 添加圆形信息文本
                    info_text = [
                        f"Center: ({center_x:.3f}, {center_y:.3f}, {center_z:.3f})",
                        f"Radius: {radius:.3f}m",
                        f"Pixel Center: ({pixel_center_x}, {pixel_center_y})",
                        f"Pixel Radius: {pixel_radius}"
                    ]
                    
                    for i, text in enumerate(info_text):
                        cv2.putText(display_image, text, (10, 60 + i*25), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                else:
                    # 圆形超出边界，显示警告
                    cv2.putText(display_image, "Circle out of bounds", (10, 60), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        elif detected:
            # 没有检测到圆形
            cv2.putText(display_image, "No circle detected", (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # 添加坐标范围信息
        range_text = [
            "X Range: 2.50 to 3.50",
            "Y Range: -0.30 to 0.90",
            "Z Range: 0.00 to 2.00"
        ]
        
        for i, text in enumerate(range_text):
            cv2.putText(display_image, text, (10, height - 80 + i*20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        
        return display_image
    
    def destroy_node(self):
        """销毁节点时的清理工作"""
        self.should_display = False
        if self.display_thread.is_alive():
            self.display_thread.join(timeout=1.0)
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = ImageDisplayNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 