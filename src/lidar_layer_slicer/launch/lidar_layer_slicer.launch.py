# -*- coding: utf-8 -*-
"""水平スライス層ノードの単体起動.

実機は livox driver の CustomMsg (`/mid360/livox/lidar`)、模擬構成は Gazebo bridge の
PointCloud2 (`/mid360/livox/lidar/points`) を入力にする。`cloud_message_type` と
`points_topic` を合わせて切り替える。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    params_file = PathJoinSubstitution(
        [FindPackageShare('lidar_layer_slicer'), 'params', 'default.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=params_file),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('livox_topic', default_value='/mid360/livox/lidar'),
        DeclareLaunchArgument('points_topic', default_value='/mid360/livox/lidar/points'),
        Node(
            package='lidar_layer_slicer',
            executable='lidar_layer_slicer',
            name='lidar_layer_slicer',
            parameters=[LaunchConfiguration('params_file')],
            remappings=[
                ('scan', LaunchConfiguration('scan_topic')),
                ('livox_lidar', LaunchConfiguration('livox_topic')),
                ('points', LaunchConfiguration('points_topic')),
            ],
            output='screen',
        ),
    ])
