from launch import LaunchDescription
from launch_ros.actions import Node
# from launch.actions import ExecuteProcess, RegisterEventHandler
# from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 获取参数文件的绝对路径
    pkg_share = get_package_share_directory('ackerman_chassis2')
    params_file = os.path.join(pkg_share, 'config', 'ackerman_params.yaml')

    # 获取tf关系
    tf_launch_path = os.path.join(pkg_share, 'launch', 'tf_static.launch.py')


    return LaunchDescription([
        Node(
            package='ackerman_chassis2',           # 包名，对应你的CMakeLists.txt里的project()
            executable='ackerman_chassis_node2_odom',   # 可执行文件名，对应你的add_executable生成的目标
            name='chassis_driver_node2',           # 节点名，等价于命令行的 -r __node:=chassis_driver_node2
            output='screen',                       # 输出到终端（screen），方便调试
            parameters=[
                params_file,
                {'pub_odom_tf': True}
                ],              # 加载参数文件（可以是yaml文件路径或Python字典）
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tf_launch_path)
        ),
    ])