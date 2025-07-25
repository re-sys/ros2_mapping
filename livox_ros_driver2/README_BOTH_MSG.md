# Livox ROS Driver2 - 同时发送PointCloud2和自定义消息功能

## 功能说明

本功能扩展了Livox ROS Driver2，支持同时发送PointCloud2和自定义消息格式的点云数据。这对于需要同时使用两种消息格式的应用场景非常有用。

## 新增功能

### 1. 新的传输格式
- `kBothMsg = 4`: 同时发送PointCloud2和自定义消息

### 2. 新的Launch文件
- `launch/mid360_both_msg.launch.py`: 配置为同时发送两种消息格式

### 3. 新的Topic命名规则
- **多Topic模式** (`multi_topic = 1`):
  - PointCloud2: `livox/lidar_[IP地址]`
  - 自定义消息: `livox/lidar_custom_[IP地址]`
- **单Topic模式** (`multi_topic = 0`):
  - PointCloud2: `livox/lidar`
  - 自定义消息: `livox/lidar_custom`

## 使用方法

### 1. 编译
```bash
cd ~/ros2_ws
colcon build --packages-select livox_ros_driver2
source install/setup.bash
```

### 2. 启动节点
```bash
# 使用新的launch文件启动
ros2 launch livox_ros_driver2 mid360_both_msg.launch.py
```

### 3. 订阅话题
```bash
# 查看PointCloud2消息
ros2 topic echo /livox/lidar

# 查看自定义消息
ros2 topic echo /livox/lidar_custom

# 使用rviz2可视化PointCloud2
ros2 run rviz2 rviz2
```

## 配置参数

在launch文件中可以修改以下参数：

```python
xfer_format   = 4    # 4-同时发送PointCloud2和自定义消息
multi_topic   = 0    # 0-所有LiDAR共享同一话题, 1-每个LiDAR独立话题
data_src      = 0    # 0-激光雷达数据源
publish_freq  = 10.0 # 发布频率(Hz)
output_type   = 0    # 0-输出到ROS话题
frame_id      = 'livox_frame'  # 坐标系ID
```

## 消息格式

### PointCloud2消息
- 话题: `/livox/lidar` 或 `/livox/lidar_[IP]`
- 消息类型: `sensor_msgs/msg/PointCloud2`
- 字段: x, y, z, intensity, tag, line, timestamp

### 自定义消息
- 话题: `/livox/lidar_custom` 或 `/livox/lidar_custom_[IP]`
- 消息类型: `livox_ros_driver2/msg/CustomMsg`
- 字段: x, y, z, reflectivity, tag, line, offset_time

## 注意事项

1. 同时发送两种消息格式会增加CPU使用率和网络带宽
2. 两种消息格式使用相同的时间戳，确保数据同步
3. 在单Topic模式下，自定义消息使用独立的话题名称避免冲突
4. 建议根据实际需求调整发布频率以平衡性能和实时性

## 故障排除

1. **编译错误**: 确保ROS2环境已正确配置
2. **话题未发布**: 检查LiDAR连接状态和配置文件
3. **消息格式错误**: 确认订阅的消息类型与发布类型匹配

## 示例代码

### Python订阅示例
```python
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
        self.get_logger().info(f'Received PointCloud2 with {len(msg.data)} points')
    
    def custom_callback(self, msg):
        self.get_logger().info(f'Received CustomMsg with {msg.point_num} points')

def main():
    rclpy.init()
    node = LivoxSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
``` 