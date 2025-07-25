# Livox ROS Driver2 同时发送PointCloud2和自定义消息 - 使用指南

## 概述

本指南介绍如何使用Livox ROS Driver2的新功能，同时发送PointCloud2和自定义消息格式的点云数据。

## 功能特点

- ✅ 同时发送两种消息格式
- ✅ 支持单Topic和多Topic模式
- ✅ 保持数据同步性
- ✅ 兼容现有ROS2生态系统
- ✅ 易于配置和使用

## 快速开始

### 1. 环境准备

确保已安装ROS2环境：
```bash
# 检查ROS2版本
ros2 --version

# 设置环境变量
source /opt/ros/humble/setup.bash  # 根据你的ROS2版本调整
```

### 2. 编译项目

```bash
# 进入工作空间
cd ~/ros2_ws

# 编译livox_ros_driver2包
colcon build --packages-select livox_ros_driver2

# 设置环境变量
source install/setup.bash
```

### 3. 配置LiDAR

确保LiDAR配置文件正确：
```bash
# 检查配置文件是否存在
ls -la src/ros2_mapping/livox_ros_driver2/config/

# 如果需要，修改配置文件中的LiDAR IP地址
nano src/ros2_mapping/livox_ros_driver2/config/MID360_config.json
```

### 4. 启动节点

```bash
# 使用新的launch文件启动
ros2 launch livox_ros_driver2 mid360_both_msg.launch.py
```

### 5. 验证功能

在新的终端窗口中运行测试脚本：
```bash
# 运行测试订阅者
python3 test_both_msg.py
```

或者使用ROS2命令行工具：
```bash
# 查看话题列表
ros2 topic list

# 查看PointCloud2消息
ros2 topic echo /livox/lidar

# 查看自定义消息
ros2 topic echo /livox/lidar_custom

# 查看话题信息
ros2 topic info /livox/lidar
ros2 topic info /livox/lidar_custom
```

## 配置选项

### Launch文件参数

在`launch/mid360_both_msg.launch.py`中可以修改以下参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `xfer_format` | 4 | 传输格式：4=同时发送两种消息 |
| `multi_topic` | 0 | 话题模式：0=单话题，1=多话题 |
| `data_src` | 0 | 数据源：0=LiDAR |
| `publish_freq` | 10.0 | 发布频率(Hz) |
| `output_type` | 0 | 输出类型：0=ROS话题 |
| `frame_id` | 'livox_frame' | 坐标系ID |

### 话题命名规则

#### 单Topic模式 (`multi_topic = 0`)
- PointCloud2: `/livox/lidar`
- 自定义消息: `/livox/lidar_custom`

#### 多Topic模式 (`multi_topic = 1`)
- PointCloud2: `/livox/lidar_[IP地址]`
- 自定义消息: `/livox/lidar_custom_[IP地址]`

## 消息格式详解

### PointCloud2消息
```python
# 消息类型
sensor_msgs/msg/PointCloud2

# 主要字段
header: std_msgs/msg/Header
  frame_id: "livox_frame"
  stamp: 时间戳
height: 1
width: 点云数量
fields: [x, y, z, intensity, tag, line, timestamp]
data: 点云数据
```

### 自定义消息
```python
# 消息类型
livox_ros_driver2/msg/CustomMsg

# 主要字段
header: std_msgs/msg/Header
  frame_id: "livox_frame"
  stamp: 时间戳
timebase: 时间基准
point_num: 点云数量
lidar_id: LiDAR ID
points: [CustomPoint]
  x, y, z: 坐标
  reflectivity: 反射率
  tag: 标签
  line: 线号
  offset_time: 偏移时间
```

## 使用示例

### Python订阅示例

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from livox_ros_driver2.msg import CustomMsg

class LivoxSubscriber(Node):
    def __init__(self):
        super().__init__('livox_subscriber')
        
        # 订阅PointCloud2消息
        self.pointcloud2_sub = self.create_subscription(
            PointCloud2,
            'livox/lidar',
            self.pointcloud2_callback,
            10
        )
        
        # 订阅自定义消息
        self.custom_sub = self.create_subscription(
            CustomMsg,
            'livox/lidar_custom',
            self.custom_callback,
            10
        )
    
    def pointcloud2_callback(self, msg):
        self.get_logger().info(f'PointCloud2: {msg.width} points')
    
    def custom_callback(self, msg):
        self.get_logger().info(f'CustomMsg: {msg.point_num} points')

def main():
    rclpy.init()
    node = LivoxSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### C++订阅示例

```cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

class LivoxSubscriber : public rclcpp::Node
{
public:
    LivoxSubscriber() : Node("livox_subscriber")
    {
        // 订阅PointCloud2消息
        pointcloud2_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "livox/lidar", 10,
            std::bind(&LivoxSubscriber::pointcloud2_callback, this, std::placeholders::_1));
        
        // 订阅自定义消息
        custom_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            "livox/lidar_custom", 10,
            std::bind(&LivoxSubscriber::custom_callback, this, std::placeholders::_1));
    }

private:
    void pointcloud2_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "PointCloud2: %d points", msg->width);
    }
    
    void custom_callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "CustomMsg: %d points", msg->point_num);
    }
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud2_sub_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr custom_sub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LivoxSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

## 故障排除

### 常见问题

1. **编译错误**
   ```bash
   # 清理并重新编译
   cd ~/ros2_ws
   rm -rf build/ install/
   colcon build --packages-select livox_ros_driver2
   ```

2. **话题未发布**
   ```bash
   # 检查LiDAR连接
   ping [LiDAR_IP_ADDRESS]
   
   # 检查配置文件
   cat config/MID360_config.json
   
   # 查看节点状态
   ros2 node list
   ros2 node info /livox_lidar_publisher
   ```

3. **消息格式错误**
   ```bash
   # 检查消息类型
   ros2 interface show sensor_msgs/msg/PointCloud2
   ros2 interface show livox_ros_driver2/msg/CustomMsg
   ```

4. **性能问题**
   - 降低发布频率：修改`publish_freq`参数
   - 使用多Topic模式：设置`multi_topic = 1`
   - 检查网络带宽和CPU使用率

### 调试技巧

1. **启用详细日志**
   ```bash
   export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"
   export RCUTILS_LOGGING_USE_STDOUT=1
   ```

2. **监控话题状态**
   ```bash
   # 实时监控话题
   ros2 topic hz /livox/lidar
   ros2 topic hz /livox/lidar_custom
   
   # 查看话题统计
   ros2 topic bw /livox/lidar
   ros2 topic bw /livox/lidar_custom
   ```

3. **使用rviz2可视化**
   ```bash
   ros2 run rviz2 rviz2
   # 添加PointCloud2显示，话题选择 /livox/lidar
   ```

## 性能优化

### 推荐配置

- **高频率应用**: `publish_freq = 20.0`
- **实时应用**: `publish_freq = 10.0`
- **低延迟应用**: `multi_topic = 1`
- **带宽优化**: `multi_topic = 0`

### 监控指标

- 消息发布频率
- 网络带宽使用
- CPU使用率
- 内存使用
- 消息延迟

## 扩展功能

### 自定义配置

可以创建自定义的launch文件来满足特定需求：

```python
# custom_livox_launch.py
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            name='livox_lidar_publisher',
            parameters=[{
                'xfer_format': 4,
                'multi_topic': 1,
                'publish_freq': 20.0,
                'frame_id': 'livox_frame',
                'user_config_path': '/path/to/your/config.json'
            }]
        )
    ])
```

### 集成其他工具

- **Cartographer**: 使用PointCloud2消息进行SLAM
- **PCL**: 处理点云数据
- **Open3D**: 3D数据处理和可视化
- **ROS2 Bag**: 录制和回放数据

## 技术支持

如果遇到问题，请检查：

1. ROS2版本兼容性
2. LiDAR固件版本
3. 网络连接状态
4. 配置文件格式
5. 系统资源使用情况

更多信息请参考：
- [Livox官方文档](https://livox-wiki-cn.readthedocs.io/)
- [ROS2官方文档](https://docs.ros.org/)
- [项目README](README_BOTH_MSG.md) 