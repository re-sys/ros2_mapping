# alias com='ros2 launch fast_lio complete_system_simple.launch.py'
import os.path
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 展开用户主目录路径
    home_dir = os.path.expanduser('~')
    default_map_script_path = os.path.join(home_dir, 'shoot_ws', 'map.py')
    
    # 验证map.py文件是否存在
    if not os.path.exists(default_map_script_path):
        raise FileNotFoundError(f"Map script not found: {default_map_script_path}")
    
    print(f"✅ Map script found: {default_map_script_path}")
    
    # 只保留map_script_path参数
    map_script_path = LaunchConfiguration('map_script_path')

    # 只声明map_script_path参数，使用绝对路径
    declare_map_script_path_cmd = DeclareLaunchArgument(
        'map_script_path', default_value=default_map_script_path,
        description='Path to map.py script (absolute path)'
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
    
    # 方法1：只显示错误信息（推荐）
    usb_bulk_process = ExecuteProcess(
        cmd=['ros2', 'run', 'usb_bulk_node', 'usb_bulk_node'],
        name='usb_bulk_node',
        output='log'  # 只显示错误信息到日志
    )
    
    # 方法2：如果需要更精确的过滤，可以使用shell命令
    # usb_bulk_process = ExecuteProcess(
    #     cmd=['bash', '-c', 'ros2 run usb_bulk_node usb_bulk_node 2>&1 | grep -i "error\|exception\|failed"'],
    #     name='usb_bulk_node',
    #     output='screen'
    # )
    
    # 方法3：完全静默，只记录到日志文件
    # usb_bulk_process = ExecuteProcess(
    #     cmd=['ros2', 'run', 'usb_bulk_node', 'usb_bulk_node'],
    #     name='usb_bulk_node',
    #     output='log',
    #     log_cmd=True  # 启用命令日志记录
    # )
    
    # 日志信息
    log_info_cmd = LogInfo(
        msg=f'Starting complete system with: Livox driver, FAST-LIO, map script ({default_map_script_path}), and USB bulk node'
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