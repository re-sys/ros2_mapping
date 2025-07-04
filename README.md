# ROS2 Mapping System with MID360 LiDAR

This is a configured ROS2 Humble workspace for mapping using Livox MID360 LiDAR and Point-LIO.

## Quick Start with Aliases

### 1. Setup Aliases (One-time setup)
```bash
# Add these aliases to your ~/.bashrc for permanent use
echo 'alias lidar="ros2 launch livox_ros_driver2 mid360_msg.launch.py"' >> ~/.bashrc
echo 'alias lidar-rviz="ros2 launch livox_ros_driver2 mid360_rviz.launch.py"' >> ~/.bashrc
echo 'alias pointlio="ros2 launch point_lio point_lio.launch.py"' >> ~/.bashrc
echo 'alias cb="colcon build --symlink-install --parallel-workers 8"' >> ~/.bashrc
echo 'alias rlib="rm -rf build log install"' >> ~/.bashrc
echo 'alias source-ros="source ~/ros2_ws/install/setup.bash"' >> ~/.bashrc

# Reload bashrc to apply changes
source ~/.bashrc
```

### 2. Quick Commands
```bash
# Start LiDAR driver
lidar

# Start LiDAR with RViz visualization
lidar-rviz

# Start Point-LIO mapping
pointlio

# Build workspace
cb

# Clean build cache
rlib

# Source ROS2 workspace
source-ros
```

## Manual Setup (Alternative)

### Hardware Configuration
- **LiDAR**: Livox MID360
- **Host IP**: 192.168.1.50
- **LiDAR IP**: 192.168.1.197
- **Frequency**: 50.0 Hz (maximum)
- **Data Format**: CustomMsg (optimized for Point-LIO)

### Software Configuration
- **ROS2 Version**: Humble
- **Driver**: livox_ros_driver2 (ROS2 only)
- **SLAM**: Point-LIO
- **Message Type**: `livox_ros_driver2::msg::CustomMsg`

## Build Instructions

1. Make sure you have ROS2 Humble installed
2. Install dependencies:
   ```bash
   sudo apt install ros-humble-pcl-ros ros-humble-pcl-conversions
   ```
3. Build the workspace:
   ```bash
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```

## Usage

### 1. Start LiDAR Driver
```bash
ros2 launch livox_ros_driver2 mid360_msg.launch.py
```

### 2. Start Point-LIO Mapping
```bash
ros2 launch point_lio point_lio.launch.py
```

### 3. Start with RViz Visualization
```bash
ros2 launch livox_ros_driver2 mid360_rviz.launch.py
```

## Topics

- `/livox/lidar` - LiDAR point cloud data (CustomMsg format)
- `/livox/imu` - IMU data
- `/Odometry` - SLAM odometry output
- `/path` - Trajectory path
- `/cloud_registered` - Registered point cloud

## Network Configuration

Make sure your network is configured as follows:
- Host PC: 192.168.1.50
- LiDAR: 192.168.1.197
- Subnet: 192.168.1.0/24

## Optimizations

The system is optimized for maximum performance:
- 50 Hz LiDAR frequency
- CustomMsg format for efficient Point-LIO processing
- Cleaned up build system for ROS2 only
- Optimized network configuration

## Troubleshooting

If you encounter build issues:
1. Clean the build cache: `rlib` (or `rm -rf build/ install/ log/`)
2. Make sure Livox SDK is installed: `sudo apt install livox-sdk`
3. Check network connectivity to LiDAR
4. Verify IP configuration matches the settings above

## Alias Management

### Adding Aliases Permanently
To make aliases available in every new terminal session:

1. **Edit bashrc file:**
   ```bash
   nano ~/.bashrc
   ```

2. **Add aliases at the end of the file:**
   ```bash
   # ROS2 Mapping Aliases
   alias lidar="ros2 launch livox_ros_driver2 mid360_msg.launch.py"
   alias lidar-rviz="ros2 launch livox_ros_driver2 mid360_rviz.launch.py"
   alias pointlio="ros2 launch point_lio point_lio.launch.py"
   alias cb="colcon build --symlink-install --parallel-workers 8"
   alias rlib="rm -rf build log install"
   alias source-ros="source ~/ros2_ws/install/setup.bash"
   ```

3. **Save and reload:**
   ```bash
   source ~/.bashrc
   ```

### Temporary Aliases
For current session only:
```bash
alias lidar="ros2 launch livox_ros_driver2 mid360_msg.launch.py"
```

### List All Aliases
```bash
alias
```

### Remove an Alias
```bash
unalias lidar
``` 