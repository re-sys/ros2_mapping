# Point-LIO

> ROS2 Fork repo maintainer: [LihanChen2004](https://github.com/LihanChen2004)

## Point-LIO: Robust High-Bandwidth Lidar-Inertial Odometry

**Branch**: RM25_SMBU_auto_sentry (支持先验地图输入)

**Point-LIO** 是一种基于点云的高带宽激光雷达-惯性里程计算法，能够实现高频率（4k-8kHz）的里程计输出，并且在剧烈运动和IMU饱和的情况下仍能保持鲁棒性。

## 📖 工作原理

详细的技术原理和算法说明请参考：[**Point-LIO 工作原理详解**](doc/working_principle.md)

## 🚀 快速开始

### 系统要求

- **操作系统**: Ubuntu >= 20.04
- **ROS版本**: ROS2 >= Foxy (推荐使用 ROS2 Humble)
- **硬件**: Livox Mid360 激光雷达 + IMU

### 1. 环境准备

#### 安装依赖
```bash
# 安装ROS2依赖
sudo apt-get install ros-$ROS_DISTRO-pcl-conversions

# 安装Eigen
sudo apt-get install libeigen3-dev

# 安装livox_ros_driver2
# 请参考: https://github.com/Livox-SDK/livox_ros_driver2
```

#### 配置livox_ros_driver2
```bash
# 将以下行添加到 ~/.bashrc
source $Livox_ros_driver2_dir$/install/setup.bash
```

### 2. 编译安装

```bash
# 克隆代码
cd ~/$ROS_WORKSPACE$/src
git clone https://github.com/LihanChen2004/Point-LIO.git

# 安装依赖
cd ..
rosdepc install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -

# 编译
colcon build --symlink-install -DCMAKE_BUILD_TYPE=Release

# 配置环境
source install/setup.bash
```

### 3. 运行Point-LIO

#### 使用Mid360激光雷达
```bash
# 启动Point-LIO
ros2 launch point_lio point_lio.launch.py
```

#### 使用外部IMU
编辑 `config/mid360.yaml` 配置文件：

```yaml
common:
    lid_topic: "livox/lidar"      # 激光雷达话题名
    imu_topic: "livox/imu"        # IMU话题名

mapping:
    extrinsic_T: [x, y, z]        # LiDAR-IMU平移外参
    extrinsic_R: [                 # LiDAR-IMU旋转外参（旋转矩阵）
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0
    ]
    satu_acc: 3.0                 # 加速度计饱和值
    satu_gyro: 35.0               # 陀螺仪饱和值
    acc_norm: 1.0                 # 加速度单位（1.0表示g，9.81表示m/s²）
```

## ⚠️ 重要注意事项

### 1. 数据同步
- **IMU和LiDAR必须同步**，这是保证精度的关键
- 确保时间戳正确对齐

### 2. IMU参数配置
- 获取IMU的饱和值（加速度计和陀螺仪）
- 确认IMU加速度的单位
- 正确配置 `satu_acc`、`satu_gyro`、`acc_norm` 参数

### 3. 外参标定
- 如果已知外参，建议设置 `extrinsic_est_en: false`
- 外参标定请参考：[LiDAR-IMU Initialization](https://github.com/hku-mars/LiDAR_IMU_Init)

### 4. 时间戳问题
- 警告 "Failed to find match for field 'time'" 表示点云缺少时间戳
- Point-LIO需要每个LiDAR点的精确时间戳

### 5. 无IMU模式
- 设置 `imu_en: false`
- 在 `gravity_init` 中提供重力向量
- 保持 `use_imu_as_input: false`

## 🔧 配置参数

### 关键参数说明

| 参数 | 说明 | 推荐值 |
|------|------|--------|
| `point_filter_num` | 点云下采样率 | 4 |
| `filter_size_surf` | 平面特征滤波尺寸 | 0.5 |
| `ivox_grid_resolution` | iVox网格分辨率 | 2.0 |
| `match_s` | 特征匹配阈值 | 81.0 |
| `plane_thr` | 平面拟合阈值 | 0.1 |

### 性能调优
- **高精度模式**: 减小滤波尺寸，增加匹配点数
- **实时模式**: 增大滤波尺寸，减少匹配点数
- **剧烈运动**: 调整IMU噪声参数

## 📊 输出话题

| 话题名 | 消息类型 | 说明 |
|--------|----------|------|
| `/point_lio/odometry` | `nav_msgs/Odometry` | 里程计信息 |
| `/point_lio/path` | `nav_msgs/Path` | 轨迹路径 |
| `/point_lio/cloud_registered` | `sensor_msgs/PointCloud2` | 配准后的点云 |
| `/point_lio/cloud_scan` | `sensor_msgs/PointCloud2` | 当前扫描点云 |

## 🗺️ 地图保存

启用PCD保存功能：
```bash
# 在launch文件中设置
pcd_save_enable: true
```

地图将保存到：`Point-LIO/PCD/scans.pcd`

## 🐛 常见问题

### Q: 定位精度不高？
A: 检查IMU-LiDAR外参标定、数据同步、噪声参数配置

### Q: 计算负载过高？
A: 增大滤波尺寸、减少匹配点数、调整下采样率

### Q: 剧烈运动时漂移？
A: 检查IMU饱和值配置、调整噪声参数、确保外参准确

### Q: 启动失败？
A: 检查话题名配置、确保livox_ros_driver2已正确安装

## 📚 参考资料

- **论文**: [Point-LIO: Robust High-Bandwidth Lidar-Inertial Odometry](https://onlinelibrary.wiley.com/doi/epdf/10.1002/aisy.202200459)
- **演示视频**: [YouTube](https://youtu.be/oS83xUs42Uw)
- **外参标定**: [LiDAR-IMU Initialization](https://github.com/hku-mars/LiDAR_IMU_Init)
- **驱动安装**: [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)

## 👥 开发者

- **原论文作者**: [Dongjiao He](https://github.com/Joanna-HE), [Wei Xu](https://github.com/XW-HKU)
- **ROS2维护者**: [LihanChen2004](https://github.com/LihanChen2004)

## 📞 联系方式

如有问题，请联系：
- 贺东娇: hdj65822@connect.hku.hk
- 张富: fuzhang@hku.hk

---

**注意**: 本文档专注于部署和使用说明，详细的技术原理请参考 [工作原理文档](doc/working_principle.md)。