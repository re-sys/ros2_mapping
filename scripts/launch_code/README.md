# 点云处理器使用说明

## 功能概述

这个点云处理器可以对FAST_LIO输出的点云数据进行处理，包括：

1. **体素下采样**：减少点云密度，提高处理效率
2. **空间范围过滤**：只保留指定空间范围内的点云
3. **2D投影**：将3D点云投影到2D平面上，便于2D导航和路径规划

## 文件说明

- `pointcloud_processor.py`：主要的点云处理节点
- `pointcloud_processor.launch.py`：启动文件
- `pointcloud_processor_config.yaml`：配置文件
- `README.md`：本说明文档

## 使用方法

### 1. 直接运行节点

```bash
# 进入工作目录
cd ~/mapping_ws

# 构建项目
colcon build --packages-select ros2_mapping

# 设置环境
source install/setup.bash

# 运行节点
ros2 run ros2_mapping pointcloud_processor.py
```

### 2. 使用launch文件启动

```bash
# 使用默认配置启动
ros2 launch ros2_mapping pointcloud_processor.launch.py

# 使用自定义参数启动
ros2 launch ros2_mapping pointcloud_processor.launch.py \
    input_topic:=/Laser_map \
    x_min:=-10.0 \
    x_max:=10.0 \
    y_min:=-10.0 \
    y_max:=10.0 \
    voxel_size:=0.1 \
    height_threshold:=0.2
```

### 3. 使用配置文件

```bash
# 使用配置文件启动
ros2 launch ros2_mapping pointcloud_processor.launch.py \
    --ros-args --params-file src/ros2_mapping/scripts/launch_code/pointcloud_processor_config.yaml
```

## 参数说明

### 话题参数
- `input_topic`：输入点云话题（默认：`/Laser_map`）
- `output_3d_topic`：输出3D点云话题（默认：`/filtered_pointcloud`）
- `output_2d_topic`：输出2D投影点云话题（默认：`/projected_2d_pointcloud`）

### 空间范围参数
- `x_range`：X轴范围 `[最小值, 最大值]`（默认：`[-5.0, 5.0]`）
- `y_range`：Y轴范围 `[最小值, 最大值]`（默认：`[-5.0, 5.0]`）
- `z_range`：Z轴范围 `[最小值, 最大值]`（默认：`[-1.0, 2.0]`）

### 处理参数
- `voxel_size`：体素下采样大小（单位：米，默认：`0.05`）
- `projection_height`：2D投影高度（单位：米，默认：`0.0`）
- `height_threshold`：高度阈值，用于过滤地面点（单位：米，默认：`0.1`）

## 输出说明

### 3D点云输出
- 话题：`/filtered_pointcloud`
- 内容：经过体素下采样和空间范围过滤的3D点云
- 用途：用于3D可视化、障碍物检测等

### 2D投影点云输出
- 话题：`/projected_2d_pointcloud`
- 内容：将3D点云投影到指定高度的2D平面
- 用途：用于2D导航、路径规划、占用栅格地图生成等

## 可视化

### 使用RViz查看点云

```bash
# 启动RViz
ros2 run rviz2 rviz2

# 添加PointCloud2显示
# 1. 点击 "Add" 按钮
# 2. 选择 "PointCloud2"
# 3. 设置话题为 `/filtered_pointcloud` 或 `/projected_2d_pointcloud`
```

### 使用rqt_plot查看点云统计

```bash
# 启动rqt_plot
ros2 run rqt_plot rqt_plot
```

## 配置示例

### 室内导航配置
```yaml
pointcloud_processor:
  ros__parameters:
    x_range: [-10.0, 10.0]
    y_range: [-10.0, 10.0]
    z_range: [-0.5, 3.0]
    voxel_size: 0.1
    height_threshold: 0.1
    projection_height: 0.0
```

### 室外导航配置
```yaml
pointcloud_processor:
  ros__parameters:
    x_range: [-50.0, 50.0]
    y_range: [-50.0, 50.0]
    z_range: [-2.0, 5.0]
    voxel_size: 0.2
    height_threshold: 0.3
    projection_height: 0.0
```

## 故障排除

### 常见问题

1. **没有点云输出**
   - 检查输入话题是否正确
   - 确认FAST_LIO正在运行并发布点云
   - 检查空间范围设置是否合理

2. **点云密度过高**
   - 增加 `voxel_size` 参数值
   - 缩小空间范围

3. **2D投影点云为空**
   - 检查 `height_threshold` 设置
   - 确认点云中有高于阈值高度的点

### 调试命令

```bash
# 查看话题列表
ros2 topic list

# 查看话题信息
ros2 topic info /Laser_map
ros2 topic info /filtered_pointcloud
ros2 topic info /projected_2d_pointcloud

# 查看话题数据
ros2 topic echo /Laser_map --once
ros2 topic echo /filtered_pointcloud --once

# 查看节点参数
ros2 param list /pointcloud_processor
ros2 param get /pointcloud_processor voxel_size
```

## 性能优化

1. **调整体素大小**：根据应用需求调整 `voxel_size`
2. **缩小空间范围**：只处理感兴趣的区域
3. **调整高度阈值**：根据环境特点设置合适的高度过滤
4. **使用合适的发布频率**：避免过度频繁的点云发布

## 扩展功能

可以根据需要添加以下功能：
- 点云聚类分析
- 障碍物检测
- 地面检测
- 点云配准
- 多传感器融合 