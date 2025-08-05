# 点云圆形检测系统

## 概述

这是一个重构后的点云圆形检测系统，采用多节点架构，将点云处理和图像显示分离，避免图像显示阻塞主线程。

## 系统架构

### 节点组成

1. **点云处理器节点** (`pointcloud_processor.py`)
   - 订阅FAST-LIO发布的点云数据
   - 进行空间滤波、投影和圆形检测
   - 将检测结果发布到ROS话题

2. **图像显示节点** (`image_display_node.py`)
   - 订阅检测结果和投影图像
   - 在独立线程中显示图像
   - 不阻塞主处理流程

### 话题通信

#### 输入话题
- `/cloud_registered` (PointCloud2): FAST-LIO发布的注册点云

#### 输出话题
- `/circle_detected` (Bool): 是否检测到圆形
- `/circle_center` (Point): 圆心坐标 (x, y, z)
- `/circle_radius` (Float32): 圆形半径
- `/projection_image` (Image): 投影图像（用于调试）
- `/accumulation_progress` (Int32): 点云积累进度

## 使用方法

### 方法1：使用启动脚本（推荐）

```bash
cd scripts
python3 launch_circle_detection.py
```

这个脚本会自动启动两个节点，并在检测完成后自动关闭。

### 方法2：分别启动节点

#### 终端1：启动点云处理器
```bash
cd scripts
python3 pointcloud_processor.py
```

#### 终端2：启动图像显示
```bash
cd scripts
python3 image_display_node.py
```

### 方法3：使用ROS2 launch文件

创建launch文件 `circle_detection.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_package_name',
            executable='pointcloud_processor.py',
            name='pointcloud_processor',
            output='screen'
        ),
        Node(
            package='your_package_name',
            executable='image_display_node.py',
            name='image_display_node',
            output='screen'
        )
    ])
```

然后运行：
```bash
ros2 launch your_package_name circle_detection.launch.py
```

## 参数配置

### 空间滤波参数
在 `pointcloud_processor.py` 中修改：

```python
# 空间滤波参数
self.x_range = (2.5, 3.5)      # x轴范围
self.y_range = (-0.3, 0.9)     # y轴范围  
self.z_range = (0.0, 2.0)       # z轴范围
```

### 圆形检测参数
```python
# 圆形检测参数
self.min_radius = 0.1           # 最小半径
self.max_radius = 0.4           # 最大半径
self.inlier_threshold = 0.05    # 内点阈值
self.min_inliers = 10           # 最少内点数量
self.max_iterations = 1000      # RANSAC最大迭代次数
```

### 点云积累参数
```python
# 点云积累参数
self.accumulation_frames = 10   # 积累帧数
```

### 投影参数
```python
# 投影参数
self.projection_resolution = 0.01  # 投影分辨率 (m/pixel)
self.projection_size = (600, 600)  # 投影图像大小
```

## 工作流程

1. **点云接收**: 订阅FAST-LIO发布的点云数据
2. **空间滤波**: 提取指定范围内的点云
3. **点云积累**: 积累多帧点云数据
4. **投影处理**: 将3D点云投影到2D平面
5. **轮廓提取**: 提取投影图像的轮廓
6. **圆形拟合**: 使用RANSAC算法拟合圆形
7. **结果发布**: 将检测结果发布到ROS话题
8. **图像显示**: 在独立线程中显示结果

## 优势

### 1. 非阻塞架构
- 图像显示在独立线程中运行
- 不会阻塞点云处理流程
- 提高系统响应性

### 2. 模块化设计
- 点云处理和图像显示分离
- 便于独立调试和优化
- 支持分布式部署

### 3. 实时反馈
- 实时显示点云积累进度
- 实时显示投影图像
- 检测结果即时反馈

### 4. 灵活配置
- 参数可独立配置
- 支持不同场景需求
- 易于扩展功能

## 故障排除

### 常见问题

1. **没有显示图像**
   - 检查是否有点云数据发布到 `/cloud_registered`
   - 检查空间滤波参数是否合适
   - 检查投影参数设置

2. **检测不到圆形**
   - 调整空间滤波范围
   - 修改圆形检测参数
   - 增加点云积累帧数

3. **图像显示卡顿**
   - 降低投影图像分辨率
   - 减少显示更新频率
   - 检查系统资源使用

### 调试技巧

1. **查看话题数据**
```bash
ros2 topic echo /circle_detected
ros2 topic echo /circle_center
ros2 topic echo /circle_radius
```

2. **查看图像话题**
```bash
ros2 run rqt_image_view rqt_image_view
```

3. **监控系统资源**
```bash
htop
nvidia-smi  # 如果有GPU
```

## 依赖项

- ROS2 (Humble或更高版本)
- Python 3.8+
- OpenCV 4.x
- NumPy
- scikit-learn

## 许可证

本项目采用MIT许可证。 