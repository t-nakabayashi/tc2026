"""GUIで選択した記録経路を実機自律走行へ渡す。起動後は開始操作を待つ。"""
from datetime import datetime
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from icart_bringup.recorded_route import inspect_recorded_route
from icart_bringup.hardware_core import prepare_real, read_yaml, write_yaml
from icart_bringup.session_core import validate_domain
from icart_bringup.localization_config import localization_settings


def setup(context):
    directory = LaunchConfiguration('route_directory').perform(context).strip()
    if not directory:
        raise ValueError('起動・設定で記録ルートを選択してください')
    route = inspect_recorded_route(directory)
    validate_domain({}, 'real', int(os.environ.get('ROS_DOMAIN_ID', '0')))
    root = LaunchConfiguration('output_root').perform(context).strip()
    if not root:
        raise ValueError('走行設定の保存先を指定してください')
    output = Path(root).expanduser().resolve() / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    shares = [Path(get_package_share_directory(p)) for p in
              ['icart_bringup', 'route_planner', 'fast_lio', 'livox_ros_driver2']]
    source = Path(route['session']).parent
    custom = None
    if route['station'] == 'custom':
        parameters = read_yaml(source/'um982.yaml')['/rtk_gps/rtk_gps_um982_node']['ros__parameters']
        custom = {key: parameters['ntrip.'+key] for key in ('host', 'port', 'mountpoint', 'user', 'password')}
    hardware_path = source/'hardware.yaml'
    baseline = float(context.launch_configurations.get('antenna_baseline_m', '0'))
    master_forward = context.launch_configurations.get('master_forward_m', '').strip()
    if baseline != 0 or master_forward:
        import math
        if baseline != 0 and (not math.isfinite(baseline) or not .1 <= baseline <= 2.):
            raise ValueError('アンテナ間隔は0.1〜2.0 mで指定してください')
        hardware = read_yaml(hardware_path)
        if baseline != 0:
            hardware['gnss']['slave_behind_master_m'] = baseline
        if master_forward:
            value = float(master_forward)
            if not math.isfinite(value):
                raise ValueError('主アンテナ前後位置は有限値で指定してください')
            hardware['gnss']['master_forward_m'] = value
        # 採取時の原本は保持し、今回の走行用コピーに取付変更を反映する。
        output.parent.mkdir(parents=True, exist_ok=True)
        hardware_path = output.parent/(output.name+'_hardware.yaml')
        write_yaml(hardware_path, hardware)
    prepare_real(*shares, output, route['site'], hardware=hardware_path,
                 station=route['station'], custom=custom, route_directory=Path(route['directory']))
    mode = context.launch_configurations.get('localization_mode', 'gnss')
    manifest = context.launch_configurations.get('icp_map_manifest', '').strip()
    backend = context.launch_configurations.get('icp_backend_python_path', '').strip()
    from icart_bringup.session_core import load_session
    localization_settings(load_session(output/'session.yaml', 'real'), mode, manifest, backend)
    session = read_yaml(output/'session.yaml')
    session['localization_mode'] = mode
    if mode == 'icp':
        session['icp_map_manifest'] = str(Path(manifest).expanduser().resolve())
        if backend:
            session['icp_backend_python_path'] = str(Path(backend).expanduser().resolve())
    write_yaml(output/'session.yaml', session)
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(shares[0]/'launch/bringup.launch.py')),
        launch_arguments={'environment': 'real', 'session': str(output/'session.yaml'),
                          'start_ui': 'false', 'initial_drive_mode': 'autonomous',
                          'start_teleop': 'true', 'joy_input': 'joy_node'}.items())]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('route_directory', default_value=''),
        DeclareLaunchArgument('localization_mode', default_value='gnss', choices=['gnss', 'icp']),
        DeclareLaunchArgument('icp_map_manifest', default_value=''),
        DeclareLaunchArgument('icp_backend_python_path', default_value=''),
        DeclareLaunchArgument('antenna_baseline_m', default_value='0', description='0は採取時の間隔を継承。変更時は実測mを指定'),
        DeclareLaunchArgument('master_forward_m', default_value='', description='車輪中心からの主アンテナ前後位置m。後方は負。空欄は採取時の位置を継承'),
        DeclareLaunchArgument('output_root', default_value='~/route_runs'),
        OpaqueFunction(function=setup)])
