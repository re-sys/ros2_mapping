#!/bin/bash

echo "=== Livox ROS Driver 2 README 更新工具 ==="
echo

# 颜色定义
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${YELLOW}📋 README 更新内容概览:${NC}"
echo "✅ 详细配置说明 (网络、参数、launch文件)"
echo "✅ 启动方式 (ROS & ROS2)"
echo "✅ 消息类型定义 (PointCloud2, CustomMsg, IMU)"
echo "✅ 坐标系定义 (X-前, Y-左, Z-上)"
echo "✅ 四线雷达技术原理"
echo "✅ 点云获取原理 (ToF测距)"
echo "✅ 扫描模式说明"
echo "✅ 频率特性 (旋转10Hz, 发布可调)"
echo "✅ 参数调优指南"
echo "✅ 故障排除方法"
echo

echo -e "${BLUE}📂 文件位置:${NC}"
echo "主文档: livox_ros_driver2/README.md (ROS2 + MID360)"
echo "技术原理: livox_ros_driver2/resources/technical_principles.md"
echo "备份文档: livox_ros_driver2/README_OLD.md"
echo

echo -e "${GREEN}✅ README文档已更新完成!${NC}"
echo -e "${YELLOW}📋 文档结构:${NC}"
echo "  ├── README.md (主文档，ROS2 + MID360)"
echo "  ├── resources/technical_principles.md (技术原理)"
echo "  └── README_OLD.md (原文档备份)"

echo
echo -e "${BLUE}📋 关键信息速查:${NC}"
echo
echo -e "${YELLOW}🔧 频率配置说明:${NC}"
echo "  publish_freq: 控制ROS topic发布频率 (5-100Hz)"
echo "  ⚠️  不影响雷达硬件旋转频率 (硬件固定5-20Hz)"
echo "  建议: SLAM用10-20Hz, 实时应用20-50Hz"
echo
echo -e "${YELLOW}📡 消息类型:${NC}"
echo "  /livox/lidar - sensor_msgs/PointCloud2 (10Hz)"
echo "  /livox/imu   - sensor_msgs/Imu (200Hz)"
echo
echo -e "${YELLOW}🗂️ 坐标系 (MID360):${NC}"
echo "  X轴: 向前 (雷达正面)"
echo "  Y轴: 向左"
echo "  Z轴: 向上"
echo "  原点: 雷达几何中心"
echo
echo -e "${YELLOW}⚙️ 四线雷达特性:${NC}"
echo "  4条激光束垂直排列"
echo "  360°机械旋转扫描"
echo "  垂直视场: 38.4° (-7°到+31.4°)"
echo "  点云密度: ~240,000 pts/s"
echo
echo -e "${YELLOW}🔍 工作原理:${NC}"
echo "  测距: 激光ToF (Time-of-Flight)"
echo "  角度: 编码器 + 固定俯仰角"
echo "  顺序: 按水平角递增，线束从下到上"
echo "  循环: 360°为一个完整周期"
echo
echo -e "${YELLOW}🚀 快速启动:${NC}"
echo "  ROS2: ros2 launch livox_ros_driver2 rviz_MID360_launch.py"
echo "  配置: config/MID360_config.json"
echo "  雷达IP: 192.168.1.100 (默认)"
echo "  主机IP: 192.168.1.5 (需配置)"

echo
echo -e "${GREEN}🎯 文档更新完成！现在您可以:${NC}"
echo "1. 查看主文档: cat livox_ros_driver2/README.md"
echo "2. 查看技术原理: cat livox_ros_driver2/resources/technical_principles.md"
echo "3. 配置网络和参数"
echo "4. 启动雷达驱动测试"
echo "5. 与Point-LIO等SLAM算法集成" 