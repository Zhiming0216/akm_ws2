import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            namespace = 'imu',
            package = 'imu_ros2',
            executable = 'lpms_nav3_node',
            output = 'screen',
            parameters=[{
                "port": "/dev/ttyUSB0",
                "baudrate": 115200
            }]
        ),
        Node(
            namespace = 'imu',
            package = 'imu_ros2',
            executable = 'quat_to_euler_node',
            parameters=[{  # 修改: 加param，确保frame_id/imu_flip
                "frame_id": "imu",
                "imu_flip": True  # 翻转倒装IMU (acc/gyro y/z负)
            }]
        )
    ])
