# Livox ROS Driver 2 - MID360 使用指南

Livox ROS Driver 2 是连接 Livox MID360 LiDAR 的 ROS2 驱动包，适用于 ROS2 Foxy 和 Humble。

## 📋 目录
1. [系统要求](#1-系统要求)
2. [安装与编译](#2-安装与编译)  
3. [配置说明](#3-配置说明)
4. [启动方式](#4-启动方式)
5. [消息类型](#5-消息类型)
6. [参数说明](#6-参数说明)
7. [故障排除](#7-故障排除)

## 1. 系统要求

* **Ubuntu 20.04** - ROS2 Foxy  
* **Ubuntu 22.04** - ROS2 Humble

### 1.1 安装依赖
```bash
# 安装 ROS2 (推荐 Desktop-Full)
# 参考: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

# 安装编译工具
sudo apt install python3-colcon-common-extensions
```

## 2. 安装与编译

### 2.1 克隆源码
```bash
# 创建工作空间
mkdir -p ~/livox_ws/src
cd ~/livox_ws/src

# 克隆驱动源码
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
```

### 2.2 安装 Livox-SDK2
```bash
# 克隆并安装 SDK
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir build && cd build
cmake .. && make -j4
sudo make install
```

### 2.3 编译驱动
```bash
cd ~/livox_ws
source /opt/ros/humble/setup.bash  # 或 foxy
colcon build
```

## 3. 配置说明

### 3.1 配置文件
配置文件位于 `config/MID360_config.json`：

```json
{
  "lidar_summary_info": {
    "lidar_type": 8
  },
  "MID360": {
    "lidar_net_info": {
      "cmd_data_port": 56100,
      "push_msg_port": 56200,
      "point_data_port": 56300,
      "imu_data_port": 56400,
      "log_data_port": 56500
    },
    "host_net_info": {
      "cmd_data_ip": "192.168.1.5",      # 主机IP
      "point_data_ip": "192.168.1.5",    # 接收点云数据的主机IP
      "imu_data_ip": "192.168.1.5",      # 接收IMU数据的主机IP
      "cmd_data_port": 56101,
      "push_msg_port": 56201,
      "point_data_port": 56301,
      "imu_data_port": 56401
    }
  },
  "lidar_configs": [
    {
      "ip": "192.168.1.100",             # 雷达IP地址
      "pcl_data_type": 1,                # 点云数据类型
      "pattern_mode": 0,                 # 扫描模式
      "extrinsic_parameter": {           # 外参设置
        "roll": 0.0,
        "pitch": 0.0, 
        "yaw": 0.0,
        "x": 0,
        "y": 0,
        "z": 0
      }
    }
  ]
}
```

### 3.2 网络配置要点
- **雷达IP**: 默认 `192.168.1.100`
- **主机IP**: 需要配置为与雷达同一网段
- **端口范围**: 56100-56500 (雷达端), 56101-56501 (主机端)

## 4. 启动方式

### 4.1 基础启动
```bash
cd ~/livox_ws
source install/setup.bash
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```

### 4.2 自定义参数启动
```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py \
    publish_freq:=20.0 \
    multi_topic:=true \
    xfer_format:=0
```

### 4.3 Launch 文件说明

| 文件名 | 用途 | 输出格式 |
|--------|------|----------|
| `rviz_MID360_launch.py` | MID360 + RViz可视化 | PointCloud2 |
| `msg_MID360_launch.py` | MID360自定义消息 | CustomMsg |

## 5. 消息类型

### 5.1 发布的 Topic

| Topic | 消息类型 | 频率 | 说明 |
|-------|----------|------|------|
| `/livox/lidar` | `sensor_msgs/PointCloud2` | 可配置 | 标准点云数据 |
| `/livox/lidar_custom` | `livox_ros_driver2/CustomMsg` | 可配置 | Livox自定义格式 |
| `/livox/imu` | `sensor_msgs/Imu` | 200Hz | IMU数据 |

### 5.2 点云消息格式

#### 标准 PointCloud2 格式 (PointXYZI)
```cpp
struct PointXYZI {
    float x, y, z;      // 位置坐标 (米)
    float intensity;    // 反射强度 (0-255)
}
```

#### Livox 扩展格式 (PointXYZRTLT)
```cpp
struct PointXYZRTLT {
    float x, y, z;          // 位置坐标 (米)
    float intensity;        // 反射强度 (0-255) 
    uint8_t tag;            // Livox标签
    uint8_t line;           // 激光线号 (0-3 for MID360)
    double timestamp;       // 点的时间戳
}
```

### 5.3 坐标系定义
- **X轴**: 向前 (雷达正面方向)
- **Y轴**: 向左 
- **Z轴**: 向上
- **原点**: 雷达几何中心
- **frame_id**: `livox_frame`

## 6. 参数说明

### 6.1 关键参数

| 参数 | 说明 | 可选值 | 默认值 |
|------|------|--------|--------|
| `publish_freq` | **点云发布频率** (Hz) | 5.0, 10.0, 20.0, 50.0 | 10.0 |
| `multi_topic` | 多topic模式 | true/false | false |
| `xfer_format` | 点云格式 | 0=Livox, 1=自定义, 2=标准PCL | 0 |
| `pcl_data_type` | 数据精度 | 1=32位, 2=16位, 3=球坐标 | 1 |
| `pattern_mode` | 扫描模式 | 0=非重复, 1=重复, 2=低频重复 | 0 |

### 6.2 重要说明

#### publish_freq 参数
- **作用**: 控制ROS topic的发布频率
- **不影响**: 雷达硬件的旋转频率 (硬件固定为5-20Hz)
- **建议值**: 
  - SLAM/建图: 10-20Hz
  - 实时应用: 20-50Hz
  - 低功耗: 5-10Hz

#### 扫描模式选择
- **非重复模式** (pattern_mode: 0): 适合SLAM，提供更密集覆盖
- **重复模式** (pattern_mode: 1): 适合监控，固定扫描轨迹
- **低频重复** (pattern_mode: 2): 适合低功耗应用

## 7. 故障排除

### 7.1 常见问题

#### 无点云显示
```bash
# 检查topic
ros2 topic list | grep livox
ros2 topic hz /livox/lidar

# 检查网络连接
ping 192.168.1.100

# 检查配置文件
cat config/MID360_config.json
```

#### 频率问题
```bash
# 检查系统资源
top
# 降低发布频率
ros2 launch livox_ros_driver2 msg_MID360_launch.py publish_freq:=5.0
```

### 7.2 网络配置

#### 修改雷达IP
1. 使用 Livox Viewer 软件
2. 或通过配置文件的 `ip` 字段
3. 确保与主机在同一网段

#### 防火墙设置
```bash
# 开放端口范围
sudo ufw allow 56100:56500/udp
sudo ufw allow 57000:59000/udp
```

### 7.3 调试技巧

#### 启用详细日志
```bash
# 在launch文件中添加
ros2 launch livox_ros_driver2 msg_MID360_launch.py --ros-args --log-level DEBUG
```

#### 录制调试数据
```bash
# 录制
ros2 bag record -o livox_data /livox/lidar /livox/imu

# 回放
ros2 bag play livox_data/
```

## 8. 示例应用

### 8.1 与 Point-LIO 集成
```bash
# 启动雷达驱动
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# 启动Point-LIO
ros2 launch point_lio mapping.launch.py
```

### 8.2 快速测试
```bash
# 1. 启动驱动
ros2 launch livox_ros_driver2 rviz_MID360_launch.py

# 2. 检查topic
ros2 topic echo /livox/lidar --once

# 3. 查看点云
# 在RViz中设置Fixed Frame为 "livox_frame"
```

## 9. 技术原理

关于MID360的技术原理、扫描模式、点云特性等详细信息，请参考：
**[技术原理文档](resources/technical_principles.md)**

该文档包含：
- 四线雷达工作原理
- 激光测距原理
- 扫描模式详解
- 点云数据特性
- 坐标系定义
- 性能优化建议

## 📞 技术支持

- **官方文档**: [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2)
- **驱动仓库**: [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)
- **问题反馈**: GitHub Issues

---

**注意**: 作为调试工具，Livox ROS Driver 不建议用于批量生产，仅限于测试场景。 