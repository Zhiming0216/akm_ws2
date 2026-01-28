import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 定位到 livox_ros_driver2 包
    driver_pkg = get_package_share_directory('livox_ros_driver2')
    
    # 2. 定位到你的“三合一”配置文件
    # 请确保文件名和你保存的一致
    config_path = os.path.join(driver_pkg, 'config', 'MID360_config.json')

    # ==========================================================
    # 核心节点: 统一管理 3 台雷达
    # ==========================================================
    livox_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            # --------------------------------------------------
            # 关键设置
            # --------------------------------------------------
            {'xfer_format': 0},     # 0 = 全部输出 PointCloud2 格式 (满足你的要求)
            {'multi_topic': 1},     # 1 = 开启多话题 (自动把3台雷达分开)
            {'data_src': 0},        # 0 = 真实雷达
            {'publish_freq': 10.0}, # 10Hz
            {'output_data_type': 0},
            
            # 这里的 frame_id 只是个默认值。
            # 实际运行中，驱动会优先使用你 JSON 文件里给每个雷达单独定义的 
            # "livox_frame_front", "livox_frame_center", "livox_frame_rear"
            {'frame_id': 'livox_frame'}, 
            
            # 加载你的三合一配置
            {'user_config_path': config_path}
        ],
        # --------------------------------------------------
        # 话题重映射 (Remapping)
        # --------------------------------------------------
        # 开启 multi_topic=1 后，驱动默认发出的名字是 /livox/lidar_192_168_99_xxx
        # 我们在这里把它们改名为你想习惯的 /livox/xxx/lidar
        # --------------------------------------------------
        remappings=[
            # 前雷达 (110)
            ('/livox/lidar_192_168_99_110', '/livox/front/lidar'),
            ('/livox/imu_192_168_99_110',   '/livox/front/imu'),
            
            # 中雷达 (111)
            ('/livox/lidar_192_168_99_111', '/livox/center/lidar'),
            ('/livox/imu_192_168_99_111',   '/livox/center/imu'),
            
            # 后雷达 (112)
            ('/livox/lidar_192_168_99_112', '/livox/rear/lidar'),
            ('/livox/imu_192_168_99_112',   '/livox/rear/imu')
        ]
    )

    return LaunchDescription([
        livox_node
    ])