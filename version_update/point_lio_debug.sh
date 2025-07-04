#!/bin/bash

echo "=== Point-LIO Debug Testing Script ==="

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${YELLOW}1. 检查Topic连接状态${NC}"
echo "检查LiDAR和IMU topic是否正常发布..."
ros2 topic list | grep -E "(livox|imu)"
echo ""

echo -e "${YELLOW}2. 检查Topic频率${NC}"
echo "检查LiDAR频率 (应该约10Hz):"
timeout 5s ros2 topic hz /livox/lidar
echo ""
echo "检查IMU频率 (应该约200Hz):"
timeout 5s ros2 topic hz /livox/imu
echo ""

echo -e "${YELLOW}3. 检查IMU数据质量${NC}"
echo "检查IMU静止时的噪声水平..."
echo "请保持设备完全静止5秒钟!"
sleep 2
ros2 topic echo /livox/imu --once | grep -A 3 linear_acceleration
ros2 topic echo /livox/imu --once | grep -A 3 angular_velocity
echo ""

echo -e "${YELLOW}4. 启动Point-LIO (Debug模式)${NC}"
echo "将在新终端启动Point-LIO，请观察日志输出..."
echo "启动命令: ros2 launch point_lio mapping.launch.py"
echo ""

echo -e "${YELLOW}5. 实时监控关键Topic${NC}"
echo "监控里程计输出:"
echo "ros2 topic echo /Odometry --field pose.pose.position"
echo ""
echo "监控TF变换:"
echo "ros2 run tf2_tools view_frames.py"
echo ""

echo -e "${YELLOW}6. 记录数据用于离线分析${NC}"
echo "记录关键topic的rosbag:"
echo "ros2 bag record /livox/lidar /livox/imu /Odometry /cloud_registered /path"
echo ""

echo -e "${YELLOW}Debug检查清单:${NC}"
echo -e "${RED}□${NC} IMU是否在启动时保持静止至少10秒?"
echo -e "${RED}□${NC} LiDAR数据频率是否稳定?"
echo -e "${RED}□${NC} 环境中是否有足够的几何特征?"
echo -e "${RED}□${NC} IMU和LiDAR的时间戳是否同步?"
echo -e "${RED}□${NC} 外参标定是否准确?"
echo ""

echo -e "${GREEN}使用方法:${NC}"
echo "1. 先运行此脚本进行基础检查"
echo "2. 启动Point-LIO并观察日志"
echo "3. 在另一个终端运行: rviz2 -d point_lio/config/rviz.rviz"
echo "4. 缓慢移动雷达，观察轨迹是否平滑" 