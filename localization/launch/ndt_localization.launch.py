import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # ==========================================
    # 1. 定义 Launch 参数 (Arguments)
    # ==========================================
    
    # 地图路径参数
    # 如果命令行没有指定 map_path，则默认使用 default_value
    map_path_arg = DeclareLaunchArgument(
        'map_path',
        default_value='/home/user/akm_ws2/map/3floor.pcd',
        description='点云地图(.pcd)的绝对路径'
    )

    # 雷达话题参数
    lidar_topic_arg = DeclareLaunchArgument(
        'lidar_topic',
        default_value='/livox/center/lidar',
        description='雷达点云话题名称'
    )

    # NDT 分辨率参数
    ndt_resolution_arg = DeclareLaunchArgument(
        'ndt_resolution',
        default_value='1.0',
        description='NDT网格分辨率'
    )

    # NDT 步长参数
    ndt_step_size_arg = DeclareLaunchArgument(
        'ndt_step_size',
        default_value='0.1',
        description='NDT搜索步长'
    )

    # ==========================================
    # 2. 获取配置值 (LaunchConfiguration)
    # ==========================================
    map_path = LaunchConfiguration('map_path')
    lidar_topic = LaunchConfiguration('lidar_topic')
    ndt_resolution = LaunchConfiguration('ndt_resolution')
    ndt_step_size = LaunchConfiguration('ndt_step_size')

    # ==========================================
    # 3. 定义节点 (Node)
    # ==========================================
    ndt_node = Node(
        package='localization',          
        executable='ndt_localization_node', 
        name='ndt_localization_node',    
        output='screen',                 
        parameters=[{
            'map_path': map_path,        
            'lidar_topic': lidar_topic,
            'ndt_resolution': ndt_resolution,
            'ndt_step_size': ndt_step_size
        }]
    )

    # ==========================================
    # 4. 返回 Launch 描述
    # ==========================================
    return LaunchDescription([
        map_path_arg,
        lidar_topic_arg,
        ndt_resolution_arg,
        ndt_step_size_arg,
        ndt_node
    ])