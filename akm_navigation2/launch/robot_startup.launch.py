import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    # ==============================
    # 0. 声明参数 (Argument)
    # ==============================
    # 默认地图路径 (YAML格式)
    default_map_path = '/home/user/akm_ws2/map/3floor.yaml'

    # 声明 'map' 参数
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='地图的绝对路径 (传入 .yaml 文件，代码会自动推导 .pcd)'
    )

    # 获取参数值 (这是 .yaml 路径)
    map_yaml_path = LaunchConfiguration('map')

    # 使用 PythonExpression 动态把字符串里的 .yaml 替换为 .pcd
    # 这样 NDT 就能拿到正确的点云地图路径
    map_pcd_path = PythonExpression(["'", map_yaml_path, "'.replace('.yaml', '.pcd')"])

    # ==============================
    # 1. 定义各个包的路径
    # ==============================
    chassis_pkg_dir = get_package_share_directory('ackerman_chassis2')
    livox_pkg_dir = get_package_share_directory('livox_ros_driver2')
    imu_pkg_dir = get_package_share_directory('imu_ros2')
    fastlio_pkg_dir = get_package_share_directory('fast_lio')
    nav2_pkg_dir = get_package_share_directory('akm_navigation2')

    # ==============================
    # 2. 定义启动动作 (Actions)
    # ==============================

    # A. 启动底盘驱动 (立即启动)
    launch_chassis = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(chassis_pkg_dir, 'launch', 'ackerman_chassis_node2.launch.py')
        )
    )

    # B. 启动 Livox 雷达驱动 (立即启动)
    launch_livox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(livox_pkg_dir, 'launch_ROS2', 'msg_MID360_all.launch.py')
        )
    )

    # C. 启动 IMU 驱动 (立即启动)
    launch_imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(imu_pkg_dir, 'launch', 'lpms_nav3_launch.py')
        )
    )

    # D. 启动 Fast-LIO 里程计 (延时 3秒)
    launch_fastlio = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(fastlio_pkg_dir, 'launch', 'odometry.launch.py')
                )
            )
        ]
    )

    # E. 启动 NDT 定位节点 (延时 6秒)
    node_ndt = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='localization',
                executable='ndt_localization_node',
                name='ndt_localization_node',
                output='screen',
                parameters=[{
                    'map_path': map_pcd_path, # 这里传入的是 .pcd 路径
                    'lidar_topic': '/livox/center/lidar',
                    'ndt_resolution': 1.0,
                    'ndt_step_size': 0.1
                }]
            )
        ]
    )

    # F. 启动 Nav2 导航 (延时 10秒)
    launch_nav2 = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_pkg_dir, 'launch', 'akm_navigation.launch.py')
                ),
                launch_arguments={
                    'map': map_yaml_path
                }.items()
            )
        ]
    )

    # ==============================
    # 3. 返回 LaunchDescription
    # ==============================
    return LaunchDescription([
        map_arg,
        
        # 硬件层
        launch_chassis,
        launch_livox,
        launch_imu,
        
        # 算法层
        launch_fastlio,
        node_ndt,
        
        # 应用层
        launch_nav2
    ])