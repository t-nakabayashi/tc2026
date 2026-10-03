"""場所・RTK補正局を選んでGNSSだけを起動する。"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from icart_bringup.hardware_core import read_yaml
from icart_bringup.gnss_config import resolve_selection, gnss_parameters
from icart_bringup.session_core import validate_domain
import os


def setup(context):
    validate_domain({}, 'real', int(os.environ.get('ROS_DOMAIN_ID', '0')))
    share = Path(get_package_share_directory('icart_bringup'))
    catalog = read_yaml(share/'params/rtk_stations.yaml')
    site, station = resolve_selection(catalog,
        LaunchConfiguration('site').perform(context), LaunchConfiguration('station').perform(context))
    hardware_path = context.launch_configurations.get('hardware', '')
    hardware = read_yaml(Path(hardware_path).expanduser() if hardware_path else share/'params/hardware.yaml')
    custom_path = context.launch_configurations.get('ntrip_config', '')
    custom = read_yaml(Path(custom_path).expanduser()) if station == 'custom' and custom_path else None
    params = gnss_parameters(catalog, hardware, site, station, custom)
    config = LaunchConfiguration('config').perform(context)
    # Optional receiver YAML controls serial/publish settings. Station selection
    # and measurement-clock settings are always applied after this file.
    if config:
        configured = read_yaml(Path(config).expanduser())
        receiver = configured.get('/rtk_gps/rtk_gps_um982_node', {}).get('ros__parameters', {})
        serial = receiver.get('serial', {})
        for key in ('port', 'baud'):
            params['serial.'+key] = receiver.get('serial.'+key, serial.get(key, params['serial.'+key]))
    return [Node(package='rtk_gps_um982', executable='rtk_gps_um982_node',
                 name='rtk_gps_um982_node', namespace='rtk_gps',
                 parameters=([config] if config else [])+[params], output='screen')]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('site', default_value='場所を選択'),
        DeclareLaunchArgument('station', default_value='地域の既定局'),
        DeclareLaunchArgument('config', default_value=''),
        DeclareLaunchArgument('hardware', default_value=''),
        DeclareLaunchArgument('ntrip_config', default_value=''),
        OpaqueFunction(function=setup)])
