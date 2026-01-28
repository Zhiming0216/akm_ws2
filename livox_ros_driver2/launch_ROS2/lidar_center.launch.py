import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    driver_pkg = get_package_share_directory('livox_ros_driver2')
    config_path = os.path.join(driver_pkg, 'config', 'center.json')

    return LaunchDescription([
        Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            name='livox_center',
            output='screen',
            respawn=True,           # 掉线自动重启
            respawn_delay=2.0,
            parameters=[
                {'xfer_format': 1},           # 1 = CustomMsg (FastLIO专用)
                {'multi_topic': 0},
                {'data_src': 0},
                {'publish_freq': 10.0},
                {'frame_id': 'livox_frame_center'},
                {'broadcast_discovery': False},
                {'user_config_path': config_path}
            ],
            remappings=[
                ('/livox/lidar', '/livox/center/lidar'),
                ('/livox/imu',   '/livox/center/imu')
            ]
        )
    ])