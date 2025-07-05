# ROS2 Mapping System with MID360 LiDAR

基于ROS2 Humble的Livox MID360激光雷达建图系统，集成Point-LIO算法。

## 🚀 Quick Start

### 1. 一键配置（推荐）

运行一键配置脚本，自动完成所有环境配置：

```bash
cd ~/ros2_ws/src/ros2_mapping
./scripts/setup_ros2_mapping.sh
```

该脚本会自动：
- 配置快捷命令别名
- 安装ROS依赖
- 构建工作空间
- 配置环境变量

### 2. 配置LiDAR IP

编辑配置文件，将IP最后两位改为雷达序列号的最后两位：

```bash
nano livox_ros_driver2/config/MID360_config.json
```

修改以下部分：
```json
{
  "lidar_configs" : [
    {
      "ip" : "192.168.1.1**",   // ← 改为雷达序列号最后两位
      ...
    }
  ]
}
```

### 3. 启动系统

```bash
# 启动LiDAR驱动
lidar

# 启动Point-LIO建图（新终端）
pointlio
```

## 📋 系统配置

### 硬件配置
- **LiDAR**: Livox MID360
- **Host IP**: 192.168.1.50
- **LiDAR IP**: 192.168.1.1** (根据序列号配置)
- **频率**: 50.0 Hz
- **数据格式**: CustomMsg

### 软件配置
- **ROS2版本**: Humble
- **驱动**: livox_ros_driver2
- **SLAM算法**: Point-LIO
- **消息类型**: `livox_ros_driver2::msg::CustomMsg`

## 🛠️ 快捷命令

配置完成后，可使用以下快捷命令：

| 命令 | 功能 |
|------|------|
| `lidar` | 启动LiDAR驱动 |
| `lidar-rviz` | 启动LiDAR驱动并打开RViz |
| `pointlio` | 启动Point-LIO建图 |
| `cb` | 快速构建工作空间 |
| `rlib` | 清理构建缓存 |
| `source-ros` | 加载ROS2环境 |

## 📡 Topics

| Topic | 消息类型 | 说明 |
|-------|----------|------|
| `/livox/lidar` | CustomMsg | LiDAR点云数据 |
| `/livox/imu` | Imu | IMU数据 |
| `/Odometry` | Odometry | SLAM里程计输出 |
| `/path` | Path | 轨迹路径 |
| `/cloud_registered` | PointCloud2 | 配准后的点云 |

## 🔧 手动配置（可选）

### 安装依赖
```bash
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 构建工作空间
```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## 🌐 网络配置

确保网络配置如下：
- **主机IP**: 192.168.1.50
- **LiDAR IP**: 192.168.1.1** (根据序列号)
- **子网**: 192.168.1.0/24

## 🚨 故障排除

### 常见问题
1. **构建失败**: 运行 `rlib` 清理缓存后重新构建
2. **网络连接**: 检查LiDAR IP配置和网络连接
3. **依赖缺失**: 运行 `rosdep install` 安装缺失依赖

### 调试命令
```bash
# 检查Topic连接
ros2 topic list | grep livox

# 检查LiDAR数据频率
ros2 topic hz /livox/lidar

# 查看IMU数据
ros2 topic echo /livox/imu
```

## 📚 相关资源

- [鱼香ROS一键安装](https://github.com/fishros/install): `source <(wget -qO- http://fishros.com/install)`
- [Livox ROS Driver2](https://github.com/Livox-SDK/livox_ros_driver2)
- [Point-LIO](https://github.com/hku-mars/Point-LIO)

---

*如有问题，请查看 `README_DEBUG.md` 获取详细调试信息。*

## ⚙️ 高级配置

### LiDAR频率调节

LiDAR的发布频率可以在驱动launch文件中调节：

```bash
nano livox_ros_driver2/launch/mid360_msg.launch.py
```

修改第8行的 `publish_freq` 参数：
```python
publish_freq  = 10.0 # 可选: 5.0, 10.0, 20.0, 50.0, 100.0 Hz
```

**频率选项说明：**
- `5.0 Hz`: 低频率，适合低功耗场景
- `10.0 Hz`: 标准频率，平衡性能和功耗
- `20.0 Hz`: 中高频率，适合一般应用
- `50.0 Hz`: 高频率，适合高精度SLAM（推荐）
- `100.0 Hz`: 最高频率，适合实时应用

**注意：** 修改频率后需要重新构建工作空间：`cb` 