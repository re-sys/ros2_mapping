import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 只保留map_script_path参数
    map_script_path = LaunchConfiguration('map_script_path')

    # 只声明map_script_path参数
    declare_map_script_path_cmd = DeclareLaunchArgument(
        'map_script_path', default_value='~/shoot_ws/map.py',
        description='Path to map.py script'
    )

    # 四个程序进程 - 直接启动，无条件判断
    livox_driver_process = ExecuteProcess(
        cmd=['ros2', 'launch', 'livox_ros_driver2', 'rviz_MID360_launch.py'],
        name='livox_driver',
        output='screen'
    )
    
    fast_lio_process = ExecuteProcess(
        cmd=['ros2', 'launch', 'fast_lio', 'mapping.launch.py'],
        name='fast_lio',
        output='screen'
    )
    
    map_script_process = ExecuteProcess(
        cmd=['python3', map_script_path],
        name='map_script',
        output='screen'
    )
    
    usb_bulk_process = ExecuteProcess(
        cmd=['ros2', 'run', 'usb_bulk_node', 'usb_bulk_node'],
        name='usb_bulk_node',
        output='screen'
    )
    
    # 日志信息
    log_info_cmd = LogInfo(
        msg='Starting complete system with: Livox driver, FAST-LIO, map script, and USB bulk node'
    )

    # 创建LaunchDescription
    ld = LaunchDescription()
    
    # 添加参数声明
    ld.add_action(declare_map_script_path_cmd)
    
    # 添加日志
    ld.add_action(log_info_cmd)
    
    # 添加四个进程（按启动顺序）
    ld.add_action(livox_driver_process)
    ld.add_action(fast_lio_process)
    ld.add_action(map_script_process)
    ld.add_action(usb_bulk_process)

    return ld 