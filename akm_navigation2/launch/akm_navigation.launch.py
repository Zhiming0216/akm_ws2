import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # ================= 1. 定位到包的地址 =================
    akm_navigation2_dir = get_package_share_directory('akm_navigation2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # ================= 2. 定义默认值变量 =================
    # 这里定义死默认路径，方便后面使用
    default_map_path = '/home/user/akm_ws2/map/3floor.yaml'
    default_param_path = os.path.join(akm_navigation2_dir, 'params', 'nav2_params.yaml')

    # ================= 3. 声明参数 (DeclareLaunchArgument) =================
    # 🔥 关键：有了这一步，你才能在命令行用 map:=... 来传参
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Full path to map yaml file to load')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_param_path,
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    # ================= 4. 获取配置值 (LaunchConfiguration) =================
    # 这里只负责“读取”上面声明的值
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_path = LaunchConfiguration('map')
    nav2_param_path = LaunchConfiguration('params_file')
    
    rviz_config_dir = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')

    # ================= 5. 定义节点 =================

    # 🌟 变化 A: 单独启动地图服务器
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, 
                    {'yaml_filename': map_yaml_path}] # 这里会自动读取传入的地图路径
    )

    # 🌟 变化 B: 启动生命周期管理器
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )

    # 🌟 变化 C: 只调用 navigation_launch
    nav2_navigation_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_bringup_dir, '/launch', '/navigation_launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path,
                'autostart': 'true',
                'use_composition': 'False' 
            }.items(),
    )

    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen')
    
    # ================= 6. 返回描述 =================
    return LaunchDescription([
        # 必须把声明的参数加进来
        use_sim_time_arg,
        map_arg,
        params_file_arg,
        
        # 节点
        map_server_node,
        lifecycle_manager_node,
        nav2_navigation_launch,
        rviz_node
    ])