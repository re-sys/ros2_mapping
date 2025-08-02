# 位置控制系统使用说明

## 概述

这个位置控制系统包含三个主要组件：
1. `rotate_shoot.py` - 坐标转换和旋转控制节点
2. `position_controller.py` - 位置列表管理和目标位置控制节点
3. `rpm_controller.py` - RPM手动调节控制器

## 功能特性

- 实时坐标转换和发布
- 支持YAML配置文件定义目标位置列表
- 支持通过ROS话题动态添加目标位置
- 基于shoot_btn状态控制位置切换
- 位置到达检测和状态反馈
- 支持手动和自动RPM模式切换
- 实时RPM值调节和监控

## 话题说明

### 订阅的话题
- `/Odometry` - 原始里程计数据
- `/initialpose` - 初始位姿设置
- `/shoot_btn` - 射击按钮状态（Bool）
- `/goalpose` - 新的目标位姿（PoseStamped）
- `/set_rpm` - RPM控制命令（Vector3）
- `/shoot_rpm` - 当前RPM值（Vector3）

### 发布的话题
- `/transformedxyyaw` - 转换后的坐标和角度（Vector3）
- `/target_position` - 当前目标位置（Vector3）
- `/position_status` - 位置控制状态（Vector3）
- `/cmd_vel` - 速度控制命令（Twist）
- `/shoot_rpm` - 射击转速（Vector3）
- `/set_rpm` - RPM控制命令（Vector3）

## 使用方法

### 1. 快速启动
```bash
# 在scripts目录下运行
python3 launch_position_control.py
```

### 2. 单独启动节点
```bash
# 终端1：启动坐标转换节点
python3 rotate_shoot.py

# 终端2：启动位置控制器
python3 position_controller.py

# 终端3：启动RPM控制器（可选）
python3 rpm_controller.py
```

### 3. 配置目标位置

#### 方法1：使用YAML配置文件
编辑 `target_positions.yaml` 文件：
```yaml
target_positions:
  - x: 2.0
    y: 1.5
    yaw: 0.0  # 弧度制
  - x: 3.0
    y: 2.0
    yaw: 1.5708  # 90度
```

#### 方法2：通过ROS话题动态添加
```bash
# 发布新的目标位置
ros2 topic pub /goalpose geometry_msgs/msg/PoseStamped "{pose: {position: {x: 2.0, y: 1.5, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

### 4. 控制位置切换
- 系统会自动移动到目标位置列表中的第一个位置
- 当到达目标位置后，需要将 `/shoot_btn` 话题设置为 `true` 才能移动到下一个位置
- 可以通过发布 `false` 到 `/shoot_btn` 来阻止移动到下一个位置

```bash
# 允许移动到下一个位置
ros2 topic pub /shoot_btn std_msgs/msg/Bool "data: true"

# 阻止移动到下一个位置
ros2 topic pub /shoot_btn std_msgs/msg/Bool "data: false"
```

### 5. RPM控制
```bash
# 手动设置RPM值（第一个数字为1表示手动模式）
ros2 topic pub /set_rpm geometry_msgs/msg/Vector3 "{x: 1.0, y: 1500.0, z: 0.0}"

# 恢复自动RPM模式（第一个数字为0）
ros2 topic pub /set_rpm geometry_msgs/msg/Vector3 "{x: 0.0, y: 0.0, z: 0.0}"
```

## 状态监控

### 查看当前位置
```bash
ros2 topic echo /transformedxyyaw
```

### 查看目标位置
```bash
ros2 topic echo /target_position
```

### 查看控制状态
```bash
ros2 topic echo /position_status
```

### 查看RPM状态
```bash
ros2 topic echo /shoot_rpm
```

## 参数配置

在 `position_controller.py` 中可以调整以下参数：
- `position_tolerance` - 位置到达容差（默认0.1米）
- `yaw_tolerance` - 角度到达容差（默认0.1弧度）

在 `rpm_controller.py` 中可以调整以下参数：
- `manual_rpm` - 初始手动RPM值（默认1000.0）
- `rpm_step` - RPM调节步长（默认50.0）

## RPM控制器使用说明

### 控制按键
- **上下箭头键**：调节RPM值（步长50）
- **Enter键**：发送当前RPM值到系统
- **空格键**：切换手动/自动模式
- **数字键**：直接输入RPM值
- **q键**：退出程序

### 显示信息
程序会实时显示：
- 当前RPM：系统当前使用的RPM值
- 手动RPM：用户设置的RPM值
- 模式：MANUAL（手动）或AUTO（自动）

## 工作流程

1. 系统启动后，从YAML文件加载目标位置列表
2. 自动移动到第一个目标位置
3. 到达目标位置后，等待shoot_btn为true
4. 收到shoot_btn=true后，移动到下一个目标位置
5. 重复步骤3-4，直到完成所有目标位置
6. 可以通过goalpose话题动态添加新的目标位置
7. 支持手动和自动RPM模式切换
8. 实时监控和调节RPM值

## 注意事项

- 确保ROS2环境已正确配置
- 确保所有依赖的消息类型可用
- 位置坐标使用米为单位，角度使用弧度制
- 系统会自动处理角度环绕问题
- 如果YAML配置文件不存在，系统会从空列表开始 