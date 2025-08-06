import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    package_path = get_package_share_directory('fast_lio')
    default_config_path = os.path.join(package_path, 'config')
    default_rviz_config_path = os.path.join(
        package_path, 'rviz', 'fastlio.rviz')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')
    
    # 新增的参数
    enable_livox_driver = LaunchConfiguration('enable_livox_driver')
    enable_map_script = LaunchConfiguration('enable_map_script')
    enable_usb_bulk_node = LaunchConfiguration('enable_usb_bulk_node')
    map_script_path = LaunchConfiguration('map_script_path')

    # 声明launch参数
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value='mid360.yaml',
        description='Config file'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )
    
    # 新增的参数声明
    declare_enable_livox_driver_cmd = DeclareLaunchArgument(
        'enable_livox_driver', default_value='true',
        description='Enable Livox driver'
    )
    declare_enable_map_script_cmd = DeclareLaunchArgument(
        'enable_map_script', default_value='true',
        description='Enable map.py script'
    )
    declare_enable_usb_bulk_node_cmd = DeclareLaunchArgument(
        'enable_usb_bulk_node', default_value='true',
        description='Enable USB bulk node'
    )
    declare_map_script_path_cmd = DeclareLaunchArgument(
        'map_script_path', default_value='/home/ares/shoot_ws/map.py',
        description='Path to map.py script'
    )

    # 使用ExecuteProcess运行原始命令
    livox_driver_process = ExecuteProcess(
        cmd=['ros2', 'launch', 'livox_ros_driver2', 'rviz_MID360_launch.py'],
        name='livox_driver',
        condition=IfCondition(enable_livox_driver),
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
        condition=IfCondition(enable_map_script),
        output='screen'
    )
    
    usb_bulk_process = ExecuteProcess(
        cmd=['ros2', 'run', 'usb_bulk_node', 'usb_bulk_node'],
        name='usb_bulk_node',
        condition=IfCondition(enable_usb_bulk_node),
        output='screen'
    )
    
    # 日志信息
    log_info_cmd = LogInfo(
        msg='Starting complete system with: Livox driver, FAST-LIO, map script, and USB bulk node'
    )

    # 创建LaunchDescription
    ld = LaunchDescription()
    
    # 添加参数声明
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)
    ld.add_action(declare_enable_livox_driver_cmd)
    ld.add_action(declare_enable_map_script_cmd)
    ld.add_action(declare_enable_usb_bulk_node_cmd)
    ld.add_action(declare_map_script_path_cmd)
    
    # 添加日志
    ld.add_action(log_info_cmd)
    
    # 添加进程（按启动顺序）
    ld.add_action(livox_driver_process)
    ld.add_action(fast_lio_process)
    ld.add_action(map_script_process)
    ld.add_action(usb_bulk_process)

    return ld 