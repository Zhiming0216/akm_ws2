from launch import LaunchDescription
from launch_ros.actions import Node
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
            package='ackerman_chassis2',          
            executable='ackerman_chassis_node2',   
            name='chassis_driver_node2',           
            output='screen',                       
            parameters=[params_file],              
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tf_launch_path)
        ),
      
    ])