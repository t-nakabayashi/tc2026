"""GUIで選んだ場所から実機の手動採取設定を生成して起動する。"""
from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from icart_bringup.hardware_core import prepare_real, read_yaml
from icart_bringup.gnss_config import resolve_selection
from icart_bringup.session_core import validate_domain
import os


def setup(context):
    selected = LaunchConfiguration('site').perform(context)
    sites = {'稲城': 'inagi', 'つくば': 'tsukuba'}
    if selected not in sites:
        raise ValueError('起動・設定で場所（稲城／つくば）を選択してください')
    validate_domain({}, 'real', int(os.environ.get('ROS_DOMAIN_ID', '0')))
    share = Path(get_package_share_directory('icart_bringup'))
    site, station = resolve_selection(read_yaml(share/'params/rtk_stations.yaml'), selected,
                                     LaunchConfiguration('station').perform(context))
    custom_path = context.launch_configurations.get('ntrip_config', '')
    custom = read_yaml(Path(custom_path).expanduser()) if station == 'custom' and custom_path else None
    root = LaunchConfiguration('output_root').perform(context).strip()
    if not root:
        raise ValueError('軌跡の保存先を指定してください')
    output = Path(root).expanduser().resolve() / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    shares = [Path(get_package_share_directory(p)) for p in
              ['icart_bringup', 'route_planner', 'fast_lio', 'livox_ros_driver2']]
    prepare_real(*shares, output / 'session', site, station=station, custom=custom)
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(shares[0] / 'launch/survey.launch.py')),
        launch_arguments={'environment': 'real', 'session': str(output / 'session/session.yaml'),
                          'output': str(output / 'survey'), 'start_ui': 'false',
                          'joy_input': 'joy_node'}.items())]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('site', default_value='場所を選択'),
        DeclareLaunchArgument('station', default_value='地域の既定局'),
        DeclareLaunchArgument('ntrip_config', default_value=''),
        DeclareLaunchArgument('output_root', default_value='~/route_surveys'),
        OpaqueFunction(function=setup)])
