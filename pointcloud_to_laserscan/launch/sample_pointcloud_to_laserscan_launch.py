from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            remappings=[
                ('cloud_in', '/livox/front/lidar'),   # 输入为前livox原始点云
                ('scan', '/scan_front')               # 输出scan
            ],
            parameters=[{
                'target_frame': 'base_link',    # 转换到base_link坐标系
                'transform_tolerance': 0.05,

                'min_height': 0.0,             # 只要2m以下的点
                'max_height': 1.1,

                'angle_min': -3.14,             # 全360度
                'angle_max': 3.14,

                # --- 核心修改 2: 角度分辨率 ---
                # 值越小，线越平滑、越连续！
                'angle_increment': 0.004,

                'scan_time': 0.1,
                'range_min': 0.5,
                'range_max': 100.0,
                'use_inf': False,
                'inf_epsilon': 1.0,
                'publisher_qos': 'default',  # 兼容rviz2
                'queue_size': 500,               # 增大输入队列
                'transform_tolerance': 0.2      # 稍微宽容一点的 TF 等待时间
            }],
            output='screen'
        ),
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            remappings=[
                ('cloud_in', '/livox/rear/lidar'),   # 输入为后livox原始点云
                ('scan', '/scan_rear')               # 输出scan
            ],
            parameters=[{
                'target_frame': 'base_link',    # 转换到base_link坐标系
                'transform_tolerance': 0.05,

                'min_height': 0.0,             # 只要2m以下的点
                'max_height': 1.1,

                'angle_min': -3.14,             # 全360度
                'angle_max': 3.14,

                # --- 核心修改 2: 角度分辨率 ---
                # 值越小，线越平滑、越连续！
                'angle_increment': 0.004,

                'scan_time': 0.1,
                'range_min': 0.5,
                'range_max': 100.0,
                'use_inf': False,
                'inf_epsilon': 1.0,
                'publisher_qos': 'default',  # 兼容rviz2
                'queue_size': 500,               # 增大输入队列
                'transform_tolerance': 0.2      # 稍微宽容一点的 TF 等待时间
            }],
            output='screen'
        ),
    ])
