#!/bin/bash

# ROS2 Mapping 一键配置脚本
# 作者: ROS2 Mapping Team
# 功能: 自动配置alias、安装依赖、构建工作空间

set -e  # 遇到错误立即退出

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}================================${NC}"
echo -e "${BLUE}  ROS2 Mapping 一键配置脚本${NC}"
echo -e "${BLUE}================================${NC}"

# 检查是否在正确的工作空间
if [ ! -f "package.xml" ] && [ ! -d "livox_ros_driver2" ]; then
    echo -e "${RED}错误: 请在ros2_mapping工作空间根目录下运行此脚本${NC}"
    echo "当前目录: $(pwd)"
    echo "请切换到 ~/ros2_ws/src/ros2_mapping 目录"
    exit 1
fi

echo -e "${YELLOW}1. 配置ROS2 Mapping Aliases...${NC}"

# 检查alias是否已存在
if grep -q "alias lidar=" ~/.bashrc; then
    echo -e "${YELLOW}Aliases已存在，跳过配置${NC}"
else
    # 添加aliases到bashrc
    echo "" >> ~/.bashrc
    echo "# ROS2 Mapping Aliases" >> ~/.bashrc
    echo 'alias lidar="ros2 launch livox_ros_driver2 mid360_msg.launch.py"' >> ~/.bashrc
    echo 'alias lidar-rviz="ros2 launch livox_ros_driver2 mid360_rviz.launch.py"' >> ~/.bashrc
    echo 'alias pointlio="ros2 launch point_lio point_lio.launch.py"' >> ~/.bashrc
    echo 'alias cb="colcon build --symlink-install --parallel-workers 8"' >> ~/.bashrc
    echo 'alias rlib="rm -rf build log install"' >> ~/.bashrc
    echo 'alias source-ros="source ~/ros2_ws/install/setup.bash"' >> ~/.bashrc
    echo -e "${GREEN}✓ Aliases配置完成${NC}"
fi

echo -e "${YELLOW}2. 安装ROS依赖...${NC}"

# 检查rosdep是否已安装
if ! command -v rosdep &> /dev/null; then
    echo -e "${YELLOW}安装rosdep...${NC}"
    sudo apt update
    sudo apt install -y python3-rosdep
    sudo rosdep init || true  # 如果已初始化则忽略错误
fi

# 更新rosdep并安装依赖
echo -e "${YELLOW}更新rosdep并安装依赖...${NC}"
rosdep update
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
echo -e "${GREEN}✓ 依赖安装完成${NC}"

echo -e "${YELLOW}3. 构建工作空间...${NC}"
colcon build --symlink-install --parallel-workers 8
echo -e "${GREEN}✓ 工作空间构建完成${NC}"

echo -e "${YELLOW}4. 配置环境...${NC}"
source ~/ros2_ws/install/setup.bash
echo -e "${GREEN}✓ 环境配置完成${NC}"

echo ""
echo -e "${GREEN}================================${NC}"
echo -e "${GREEN}  配置完成！${NC}"
echo -e "${GREEN}================================${NC}"
echo ""
echo -e "${BLUE}可用的快捷命令:${NC}"
echo "  lidar      - 启动LiDAR驱动"
echo "  lidar-rviz - 启动LiDAR驱动并打开RViz"
echo "  pointlio   - 启动Point-LIO建图"
echo "  cb         - 快速构建工作空间"
echo "  rlib       - 清理构建缓存"
echo "  source-ros - 加载ROS2环境"
echo ""
echo -e "${YELLOW}下一步:${NC}"
echo "1. 重新打开终端或运行: source ~/.bashrc"
echo "2. 配置LiDAR IP地址"
echo "3. 启动LiDAR和Point-LIO"
echo ""
echo -e "${BLUE}配置LiDAR IP:${NC}"
echo "编辑文件: livox_ros_driver2/config/MID360_config.json"
echo "将IP地址最后两位改为雷达序列号的最后两位"
echo ""
echo -e "${BLUE}启动命令:${NC}"
echo "lidar    # 启动LiDAR"
echo "pointlio # 启动Point-LIO" 