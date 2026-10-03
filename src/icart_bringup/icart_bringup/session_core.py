"""地形生成物と共通走行設定を結ぶ、ROS非依存のセッション設定."""
import argparse
import json
import math
import os
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

from geo_pose_converter.geo_core import ProjectionConfig
from icart_bringup.route_storage_core import canonical_rows, write_route


def load_session(filename: Path, environment: str) -> dict:
    """全入力を起動前に検査し、環境ごとのセンサ構成を明示する."""
    if environment not in ['simulation', 'real']:
        raise ValueError('environmentはsimulationまたはrealが必要')
    data = yaml.safe_load(filename.read_text())
    if not isinstance(data, dict):
        raise ValueError('sessionはYAML mappingが必要')
    if data.get('gnss_namespace') in (None, '/rtk_gps/rtk_gps_um982_node'):
        data['gnss_namespace'] = '/rtk_gps'
    for key in ['projection_params', 'route_config', 'csv_base_dir']:
        if not data.get(key):
            raise ValueError(key+'が未設定')
        data[key] = str((filename.parent/data[key]).resolve())
        if not Path(data[key]).exists():
            raise ValueError(key+'が存在しない')
    optional = ['fusion_params']
    optional += (['trial_directory', 'trial_sdf'] if environment == 'simulation'
                 else ['fastlio_config', 'hardware_launch', 'hardware_config', 'recorder_params'])
    for key in optional:
        if data.get(key):
            data[key] = str((filename.parent/data[key]).resolve())
            if not Path(data[key]).exists():
                raise ValueError(key+'が存在しない')
    if environment == 'simulation':
        density = float(data.get('pedestrian_density', .1))
        if not math.isfinite(density) or density < 0:
            raise ValueError('pedestrian_density は有限の非負値が必要')
        fix_radius = float(data.get('gnss_start_fix_radius_m', 10.))
        if not math.isfinite(fix_radius) or fix_radius < 0:
            raise ValueError('gnss_start_fix_radius_m は有限の非負値が必要')
        for name in ['trial.sdf', 'world.json', 'trial.json']:
            if not data.get('trial_directory') or not (Path(data['trial_directory'])/name).is_file():
                raise ValueError('IMU追加済みtrial_directoryに'+name+'が必要')
    return data


def prepare(trial: Path, output: Path) -> None:
    """地図を複製せず、原点・経路と編集用sessionを新規ディレクトリへ生成する."""
    metadata = json.loads((trial/'trial.json').read_text())
    if not metadata.get('projection'):
        raise ValueError('実環境と共有する投影原点がtrial.jsonに必要')
    if not (trial/'route.csv').is_file():
        raise ValueError('route.csvが必要')
    route_rows = canonical_rows(trial/'route.csv', ProjectionConfig(**metadata['projection']))
    world = ET.parse(trial/'trial.sdf')
    physics = world.getroot().find('.//physics/max_step_size')
    if physics is not None:
        physics.text = '0.001'
    for joint in world.getroot().findall(".//model[@name='icart_mini']/joint[@type='revolute']"):
        axis = joint.find('axis')
        xyz = axis.find('xyz')
        # terrain生成器の既知の車輪だけ、等価な局所軸へ変換する。
        if (xyz is not None and xyz.get('expressed_in') == '__model__'
                and xyz.text.strip() == '0 1 0'
                and joint.get('name') in ['left_wheel_joint', 'right_wheel_joint']):
            xyz.attrib.clear()
            xyz.text = '0 0 -1'
        limit = axis.find('limit')
        if limit is None:
            limit = ET.SubElement(axis, 'limit')
        for key, value in [('effort', '2.0'), ('velocity', '20.0')]:
            if limit.find(key) is None:
                ET.SubElement(limit, key).text = value
    drive = world.getroot().find(
        ".//model[@name='icart_mini']/plugin[@name='gz::sim::systems::DiffDrive']")
    if drive is not None:
        for key, value in [('max_angular_velocity', '1.2'), ('min_angular_velocity', '-1.2'),
                           ('max_angular_acceleration', '1.0'), ('min_angular_acceleration', '-1.0')]:
            if drive.find(key) is None:
                ET.SubElement(drive, key).text = value
    output.mkdir(parents=True, exist_ok=False)
    world.write(output/'session_world.sdf', encoding='utf-8', xml_declaration=True)
    (output/'projection.yaml').write_text(yaml.safe_dump(
        {'/**': {'ros__parameters': dict(metadata['projection'], projection_id='terrain_trial')}}))
    routes = output/'routes'
    (routes/'fixed').mkdir(parents=True)
    write_route(routes/'fixed'/'waypoints.csv', route_rows)
    (routes/'route_config.yaml').write_text(yaml.safe_dump(
        {'blocks': [{'type': 'fixed', 'name': 'terrain',
                     'segment_id': 'fixed/waypoints.csv'}]}))
    (output/'session.yaml').write_text(yaml.safe_dump(dict(
        projection_params='projection.yaml', route_config='routes/route_config.yaml',
        trial_sdf='session_world.sdf',
        csv_base_dir='routes',
        trial_directory=os.path.relpath(trial.resolve(), output.resolve()),
        start_label=route_rows[0]['label'], goal_label=route_rows[-1]['label'],
        noise_profile='conservative', noise_seed=1, building_gnss=True,
        gnss_start_fix_radius_m=10.0,
        hardware_launch='', fastlio_config=''), allow_unicode=True))


def main() -> None:
    """地図生成後に呼び出す設定準備CLI."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    prepare(args.trial, args.output)


def validate_domain(data: dict, environment: str, actual: int) -> None:
    """同じトピックを持つ実機と模擬環境のDDS混在を起動前に防ぐ."""
    simulation = int(data.get('simulation_domain_id', 86))
    real = int(data.get('real_domain_id', 0))
    if simulation == real or not 0 <= real <= 232 or not 1 <= simulation <= 232:
        raise ValueError('実機と模擬のdomainは異なる有効値が必要')
    expected = simulation if environment == 'simulation' else real
    if actual != expected:
        raise ValueError(f'UIとlaunchをROS_DOMAIN_ID={expected}で起動してください')


def run_main() -> None:
    """環境選択からDDS domainを設定し、共通launchへプロセスを引き渡す."""
    parser = argparse.ArgumentParser(description='i-Cart実機・デジタルツイン共通起動')
    parser.add_argument('--session', type=Path, required=True)
    parser.add_argument('--environment', choices=['real', 'simulation'], required=True)
    parser.add_argument('--start-ui', action='store_true')
    args = parser.parse_args()
    launch_session(args.session, args.environment, start_ui=args.start_ui)


def launch_session(session: Path, environment: str, *, start_ui: bool = False) -> None:
    """DDS設定を検査し、余分なros2 runプロセスを挟まずlaunchへ引き渡す."""
    data = load_session(session, environment)
    domain = int(data.get('simulation_domain_id', 86) if environment == 'simulation'
                 else data.get('real_domain_id', 0))
    validate_domain(data, environment, domain)
    os.environ['ROS_DOMAIN_ID'] = str(domain)
    if environment == 'simulation':
        os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    os.execvp('ros2', ['ros2', 'launch', 'icart_bringup', 'bringup.launch.py',
                       'session:='+str(session.resolve()),
                       'environment:='+environment,
                       'start_ui:='+str(start_ui).lower()])
