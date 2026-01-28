import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    driver_pkg = get_package_share_directory('livox_ros_driver2')
    config_path = os.path.join(driver_pkg, 'config', 'front.json')

    return LaunchDescription([
        Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            name='livox_front',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[
                {'xfer_format': 0},           # 0 = PointCloud2
                {'multi_topic': 0},
                {'data_src': 0},
                {'publish_freq': 10.0},
                {'frame_id': 'livox_frame_front'},
                {'broadcast_discovery': False},
                {'user_config_path': config_path}
            ],
            remappings=[
                ('/livox/lidar', '/livox/front/lidar'),
                ('/livox/imu',   '/livox/front/imu')
            ]
        )
    ])