"""GUIで選んだ場所から実機の手動採取設定を生成して起動する。"""
from datetime import datetime
import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from icart_bringup.hardware_core import prepare_real, read_yaml, write_yaml
from icart_bringup.gnss_config import resolve_selection
from geo_pose_converter.geo_core import ProjectionConfig
from icp_localization.registration import load_manifest
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
    mode = LaunchConfiguration('localization_mode').perform(context)
    if mode not in ('gnss', 'icp'):
        raise ValueError('localization_modeはicpまたはgnssが必要')
    settings = dict(localization_mode=mode)
    projection = None
    if mode == 'icp':
        manifest = Path(LaunchConfiguration('icp_map_manifest').perform(context)).expanduser().resolve()
        projection = ProjectionConfig(**json.loads(manifest.read_text())['projection'])
        load_manifest(manifest, vars(projection))
        backend = LaunchConfiguration('icp_backend_python_path').perform(context).strip()
        if backend and not Path(backend).expanduser().is_dir():
            raise ValueError('ICPバックエンドのディレクトリがありません')
        settings.update(icp_map_manifest=str(manifest),
                        icp_backend_python_path=str(Path(backend).expanduser().resolve()) if backend else '')
    prepare_real(*shares, output / 'session', site, station=station, custom=custom,
                 survey_projection=projection)
    session_file = output/'session/session.yaml'
    write_yaml(session_file, dict(read_yaml(session_file), **settings))
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
        DeclareLaunchArgument('localization_mode', default_value='icp', choices=['icp', 'gnss']),
        DeclareLaunchArgument('icp_map_manifest', default_value='/media/nkb/TEST/icp_runtime_20261007/map_manifest.json'),
        DeclareLaunchArgument('icp_backend_python_path', default_value='/home/nkb/colcon_ws_experiments/glim_runtime/python'),
        DeclareLaunchArgument('output_root', default_value='/media/nkb/TEST/route_surveys'),
        OpaqueFunction(function=setup)])
