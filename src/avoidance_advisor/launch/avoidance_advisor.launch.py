# -*- coding: utf-8 -*-
"""AI 助言ノードの単体起動.

入力は lidar_layer_slicer の層画像・層観測値と、カメラ画像である。
CLI を実行できない環境では `enabled:=false` で起動し、要求を拒否させる。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    params_file = PathJoinSubstitution(
        [FindPackageShare('avoidance_advisor'), 'params', 'default.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=params_file),
        DeclareLaunchArgument('enabled', default_value='true'),
        DeclareLaunchArgument('camera_topic', default_value='/usb_cam/image_raw'),
        Node(
            package='avoidance_advisor',
            executable='avoidance_advisor',
            name='avoidance_advisor',
            parameters=[LaunchConfiguration('params_file'),
                        {'enabled': LaunchConfiguration('enabled')}],
            remappings=[('camera/image_raw', LaunchConfiguration('camera_topic'))],
            output='screen',
        ),
    ])
