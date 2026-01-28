import math
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # ==========================================
    # 角度转弧度计算
    # ==========================================
    # 1. 中心雷达 (1.124) [新变化!]
    center_yaw   = 2.0 * (math.pi / 180.0)   # ≈ 0.0349
    center_pitch = 0.0
    center_roll  = 0.0

    # 2. 前雷达 (1.3)
    front_roll   = -183.0 * (math.pi / 180.0)
    front_pitch  = 0.0
    front_yaw    = 0.0
    
    # 3. 后雷达 (1.145)
    rear_roll    = 178.4  * (math.pi / 180.0)
    rear_pitch   = -0.1   * (math.pi / 180.0)
    rear_yaw     = -182.0 * (math.pi / 180.0)

    return LaunchDescription([
        # ----------------------------------------------------------
        # 1. 地面投影 -> 车身
        # ----------------------------------------------------------
        Node(
            package='tf2_ros', executable='static_transform_publisher', name='base_to_footprint',
            arguments=['0', '0', '-0.17', '0', '0', '0', 'base_link', 'base_footprint']
        ),

        # ----------------------------------------------------------
        # 2. 中心建图雷达 (Center 1.124)
        # ----------------------------------------------------------
        # 位置: X=0.52, Y=0.39 (39cm), Z=1.215
        # 旋转: Yaw=2度
        Node(
            package='tf2_ros', executable='static_transform_publisher', name='center_lidar_tf',
            arguments=['0.52', '0.39', '1.215', str(center_yaw), '0', '0', 'base_link', 'livox_frame_center']
        ),

        # ----------------------------------------------------------
        # 3. 前避障雷达 (Front 1.3)
        # ----------------------------------------------------------
        Node(
            package='tf2_ros', executable='static_transform_publisher', name='front_lidar_tf',
            arguments=['1.30', '0.455', '1.0', str(front_yaw), str(front_pitch), str(front_roll), 'base_link', 'livox_frame_front']
        ),

        # ----------------------------------------------------------
        # 4. 后避障雷达 (Rear 1.145)
        # ----------------------------------------------------------
        Node(
            package='tf2_ros', executable='static_transform_publisher', name='rear_lidar_tf',
            arguments=['-0.26', '-0.455', '1.0', str(rear_yaw), str(rear_pitch), str(rear_roll), 'base_link', 'livox_frame_rear']
        ),
    ])