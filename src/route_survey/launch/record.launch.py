"""実機・模擬で共通の記録ノード。Joy操縦と位置推定は既存起動を使う."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('output'), DeclareLaunchArgument('projection'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('localization_mode', default_value='gnss', choices=['gnss', 'icp']),
        Node(package='route_survey', executable='recorder', output='screen', parameters=[
            str(Path(get_package_share_directory('route_survey'))/'params/default.yaml'),
            {'output_directory': LaunchConfiguration('output'),
             'projection_config': LaunchConfiguration('projection'),
             'localization_mode': LaunchConfiguration('localization_mode'),
             'use_sim_time': LaunchConfiguration('use_sim_time')}])])
