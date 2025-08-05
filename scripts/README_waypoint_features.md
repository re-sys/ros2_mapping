# 路径点管理功能说明

## 新增功能

### 1. 顺序导航
- 当序列完成后添加多个新目标点时，系统会按顺序依次访问所有目标点
- 不再只访问最后一个添加的目标点

### 2. 路径点保存和加载
- 可以将当前标记的所有路径点保存到文件
- 可以从文件加载之前保存的路径点
- 支持清空所有路径点

## 使用方法

### 路径点管理命令

#### 方法1：使用路径点管理器脚本
```bash
# 保存当前路径点
python3 scripts/waypoint_manager.py save

# 加载已保存的路径点
python3 scripts/waypoint_manager.py load

# 清空所有路径点
python3 scripts/waypoint_manager.py clear

# 显示帮助信息
python3 scripts/waypoint_manager.py help
```

#### 方法2：使用ROS话题
```bash
# 保存路径点
ros2 topic pub /save_waypoints std_msgs/msg/Bool "data: true"

# 加载路径点
ros2 topic pub /load_waypoints std_msgs/msg/Bool "data: true"

# 清空路径点
ros2 topic pub /clear_waypoints std_msgs/msg/Bool "data: true"
```

### 工作流程示例

#### 1. 标记路径点
1. 启动position_controller节点
2. 设置initialpose
3. 在RViz中标记多个goal_pose
4. 按position_btn开始导航

#### 2. 保存路径点
```bash
python3 scripts/waypoint_manager.py save
```
路径点将保存到`saved_waypoints.yaml`文件

#### 3. 重新运行相同路径
1. 重启position_controller节点（会自动加载保存的路径点）
2. 或者手动加载：
```bash
python3 scripts/waypoint_manager.py load
```
3. 设置initialpose
4. 按position_btn开始导航

## 文件格式

保存的路径点文件格式（YAML）：
```yaml
waypoints:
  - id: 0
    x: 2.0
    y: 0.0
    yaw: 0.0
  - id: 1
    x: 4.0
    y: 2.0
    yaw: 0.0
  - id: 2
    x: 6.0
    y: 0.0
    yaw: 0.0
```

## 功能特点

### 1. 自动加载
- 节点启动时会自动尝试加载已保存的路径点
- 如果文件不存在，会使用预定义的waypoints

### 2. 坐标系统
- 保存的是map坐标系下的坐标
- 加载时会根据当前的initialpose进行坐标转换

### 3. 状态管理
- 保存和加载操作会更新marker显示
- 清空操作会停止当前导航并清除所有marker

### 4. 错误处理
- 文件操作失败时会输出错误信息
- 不会影响正常的导航功能

## 注意事项

1. **文件位置**：路径点文件保存在`position_controller.py`文件所在目录下的`saved_waypoints.yaml`
2. **坐标系统**：确保使用相同的initialpose来保证坐标一致性
3. **文件权限**：确保脚本有读写文件的权限
4. **备份**：重要的路径点文件建议备份

## 故障排除

### 常见问题

1. **无法保存路径点**
   - 检查文件权限
   - 检查磁盘空间
   - 查看错误日志

2. **加载的路径点位置不对**
   - 确保使用相同的initialpose
   - 检查坐标系统设置

3. **marker不显示**
   - 确保map_origin已设置
   - 检查marker话题是否正常发布

### 调试命令
```bash
# 查看保存的路径点文件
cat saved_waypoints.yaml

# 查看话题消息
ros2 topic echo /waypoint_markers

# 查看节点日志
ros2 node info /multi_point_position_controller
``` 