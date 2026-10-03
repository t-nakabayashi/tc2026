"""昨年度の車体構成から、実機専用の設定一式を生成する。起動・通信はしない。"""
import argparse
import ipaddress
import math
from pathlib import Path

import numpy as np
import yaml

from gnss_lio_fusion.mount_core import rotation
from geo_pose_converter.geo_core import ProjectionConfig
from route_planner.route_builder import RouteBuilder
from icart_bringup.route_storage_core import write_route, canonical_rows


def read_yaml(path: Path) -> dict:
    value = yaml.safe_load(path.read_text())
    if not isinstance(value, dict):
        raise ValueError(f'YAML mappingが必要: {path}')
    return value


def coordinator_parameter(config: dict) -> Path:
    """インストール済みpackage URIまたは従来のファイルパスを解決する。"""
    value = config['wheel']['coordinator_param']
    if value.startswith('package://'):
        from ament_index_python.packages import get_package_share_directory
        package, relative = value[len('package://'):].split('/', 1)
        return Path(get_package_share_directory(package)) / relative
    return Path(value).expanduser()


def geometry(config: dict) -> dict:
    """LiDAR原点からIMU、前主アンテナ、後副アンテナを車体座標で求める。"""
    mid, gps = config['mid360'], config['gnss']
    for value in [mid['xyz'], mid['rpy_deg'], mid['lidar_in_imu_xyz'],
                  config['urg']['xyz'], config['urg']['rpy_deg']]:
        if len(value) != 3 or not all(math.isfinite(float(v)) for v in value):
            raise ValueError('センサxyz/rpyは3個の有限値が必要')
    above, behind = float(gps['master_above_lidar_m']), float(gps['slave_behind_master_m'])
    if not all(math.isfinite(v) and v > 0 for v in [above, behind]):
        raise ValueError('アンテナ間隔は正の有限値が必要')
    angles = [math.radians(float(v)) for v in mid['rpy_deg']]
    lidar = np.array(mid['xyz'], dtype=float)
    imu = lidar - rotation(*angles) @ np.array(mid['lidar_in_imu_xyz'])
    master = lidar + [0., 0., above]
    master[0] = float(gps.get('master_forward_m', master[0]))
    if not math.isfinite(master[0]):
        raise ValueError('主アンテナ前後位置は有限値が必要')
    slave = master + [-behind, 0., 0.]
    return dict(lidar=lidar.tolist(), imu=imu.tolist(), master=master.tolist(),
                slave=slave.tolist(), rpy_deg=list(mid['rpy_deg']), baseline_m=behind)


def station_config(catalog: dict, site: str, station: str | None,
                   custom: dict | None = None) -> tuple[str, dict]:
    """地域内の選択肢を検証。認証情報は利用者の別ファイルからのみ受け取る。"""
    if site not in catalog['sites']:
        raise ValueError('未知のsite')
    region = catalog['sites'][site]
    station = station or region['default_station']
    if station not in region['stations']:
        raise ValueError('このsiteで選択できないstation')
    if station == 'none':
        return station, {'enabled': False}
    value = custom if station == 'custom' else catalog['stations'][station]
    if not value:
        raise ValueError('custom局は--ntrip-configが必要')
    result = dict(enabled=True, host=value.get('host', ''), port=int(value.get('port', 2101)),
                  mountpoint=value.get('mountpoint', ''), user=value.get('user', ''),
                  password=value.get('password', ''))
    if (not result['host'] or not result['mountpoint'] or not 1 <= result['port'] <= 65535
            or any(c in str(v) for v in result.values() for c in ['\r', '\n'])
            or any(c in result['mountpoint'] for c in [' ', '/', '?', '#'])):
        raise ValueError('NTRIP接続設定が不正')
    return station, result


def write_yaml(path: Path, value: dict, private: bool = False) -> None:
    path.write_text(yaml.safe_dump(value, allow_unicode=True, sort_keys=False))
    if private:
        path.chmod(0o600)


def prepare_real(share: Path, planner_share: Path, fastlio_share: Path, livox_share: Path,
                 output: Path, site: str, hardware: Path | None = None,
                 station: str | None = None, custom: dict | None = None,
                 route_directory: Path | None = None) -> None:
    """再現可能な実機sessionを新規出力する。既存設定や採取原本は上書きしない。"""
    config = read_yaml(hardware or share/'params/hardware.yaml')
    geo = geometry(config)
    for key in ['host_ip', 'lidar_ip']:
        ipaddress.IPv4Address(config['mid360'][key])
    ports = [config[k]['serial_port'] for k in ['gnss', 'urg', 'wheel']]
    if len(set(ports)) != len(ports):
        raise ValueError('GNSS/URG/車輪のシリアルポートが重複')
    catalog = read_yaml(share/'params/rtk_stations.yaml')
    station, ntrip = station_config(catalog, site, station, custom)
    route_dir = (route_directory or planner_share/'routes'/site).resolve()
    route_name = 'route_config.yaml' if route_directory else (
        'route_config_fixed.yaml' if site == 'inagi' else 'route_config_all_fixed.yaml')
    route_config = read_yaml(route_dir/route_name)
    first_file = route_config['blocks'][0]['segment_id']
    import csv
    with (route_dir/first_file).open(newline='') as stream:
        first = next(csv.DictReader(stream))
    if not first.get('latitude') or not first.get('longitude'):
        raise ValueError('実機初期経路はLLH正本が必要')
    # 採取経路では元の投影原点を必ず引き継ぐ。
    projection_file = route_dir/'projection.yaml'
    if projection_file.exists():
        from geo_pose_converter.geo_core import load_projection_config_from_yaml
        projection = load_projection_config_from_yaml(str(projection_file))
    else:
        projection = ProjectionConfig(float(first['latitude']), float(first['longitude']), 0.)
    builder = RouteBuilder(str(route_dir/route_name), csv_base_dir=str(route_dir), projection=projection)
    builder.load()
    blocks = builder.blocks
    if any(b['type'] != 'fixed' for b in blocks):
        raise ValueError('実機準備では確認済みfixed経路を指定する')
    start = builder.segments[blocks[0]['segment_id']].waypoints[0].label
    goal = builder.segments[blocks[-1]['segment_id']].waypoints[-1].label
    route = builder.build_route(start, goal, [])
    if len(route.waypoints) < 2:
        raise ValueError('経路は2点以上が必要')
    output.mkdir(parents=True, exist_ok=False)
    output.chmod(0o700)
    (output/'routes/fixed').mkdir(parents=True)
    # 正本のCSVとblock構成を保持し、元ディレクトリに依存しないsessionにする。
    for block in blocks:
        relative = Path(block['segment_id'])
        if relative.is_absolute() or '..' in relative.parts:
            raise ValueError('segment_idは経路内の相対パスが必要')
        target = output/'routes'/relative
        target.parent.mkdir(parents=True, exist_ok=True)
        write_route(target, canonical_rows(route_dir/relative, projection))
    write_yaml(output/'routes/route_config.yaml', route_config)
    write_yaml(output/'projection.yaml', {'/**': {'ros__parameters': vars(projection)}})
    write_yaml(output/'hardware.yaml', config)
    write_yaml(output/'geometry.yaml', geo)
    write_yaml(output/'station.yaml', dict(site=site, station=station,
               checked_date=catalog['checked_date'], details=catalog['stations'][station]))
    from .gnss_config import gnss_parameters
    gnss = gnss_parameters(catalog, config, site, station, custom)
    write_yaml(output/'um982.yaml', {'/rtk_gps/rtk_gps_um982_node': {'ros__parameters': gnss}}, True)
    fastlio = read_yaml(fastlio_share/'config/mid360.yaml')
    params = fastlio['/**']['ros__parameters']
    params['common']['lid_topic'] = '/mid360/livox/lidar'
    params['common']['imu_topic'] = '/mid360/livox/imu'
    params['mapping']['extrinsic_T'] = config['mid360']['lidar_in_imu_xyz']
    params['mapping']['extrinsic_est_en'] = False
    params['publish']['scan_bodyframe_pub_en'] = True
    params['pcd_save']['pcd_save_en'] = False
    write_yaml(output/'fastlio.yaml', fastlio)
    import json
    livox = json.loads((livox_share/'config/MID360_config.json').read_text())
    host = livox['MID360']['host_net_info']
    for key in ['cmd_data_ip', 'push_msg_ip', 'point_data_ip', 'imu_data_ip']:
        host[key] = config['mid360']['host_ip']
    sensor = livox['lidar_configs'][0]
    sensor['ip'] = config['mid360']['lidar_ip']
    sensor['frame_id'] = 'mid360_frame'
    sensor['extrinsic_parameter'] = dict(roll=0., pitch=0., yaw=0., x=0, y=0, z=0)
    (output/'livox.json').write_text(json.dumps(livox, indent=2))
    fusion = {'publish_base_tf': True, 'gnss_heading_offset_deg': config['gnss']['heading_offset_deg'],
              'baseline.initial_m': geo['baseline_m'],
              'baseline.minimum_m': geo['baseline_m'] * .7,
              'baseline.maximum_m': geo['baseline_m'] * 1.3}
    recorder = {}
    for prefix, position in [('lio', geo['imu']), ('master', geo['master'])]:
        for axis, value in zip(['forward_m', 'left_m', 'height_m'], position):
            fusion[prefix+'_'+axis] = float(value)
    for axis, value in zip(['roll', 'pitch', 'yaw'], geo['rpy_deg']):
        fusion['lio_mount_'+axis+'_deg'] = float(value)
        recorder['lidar_mount_'+axis+'_deg'] = float(value)
    recorder.update(dict(zip(['lidar_forward_m', 'lidar_left_m', 'lidar_height_m'], geo['imu'])))
    write_yaml(output/'fusion.yaml', {'gnss_lio_fusion': {'ros__parameters': fusion}})
    write_yaml(output/'recorder.yaml', {'route_survey': {'ros__parameters': recorder}})
    write_yaml(output/'session.yaml', dict(
        projection_params='projection.yaml', route_config='routes/route_config.yaml', csv_base_dir='routes',
        start_label=str(start), goal_label=str(goal), hardware_config='hardware.yaml',
        hardware_launch=str((share/'launch/hardware.launch.py').resolve()),
        fastlio_config='fastlio.yaml', fusion_params='fusion.yaml', recorder_params='recorder.yaml',
        gnss_namespace='/rtk_gps', real_domain_id=0, simulation_domain_id=86,
        site=site, rtk_station=station, route_source=str(route_dir),
        route_review_required=route_directory is None))


def validate_runtime(config: dict) -> None:
    """不足時はプロセスを起動する前に具体的な設定不足を報告する。"""
    from ament_index_python.packages import get_package_prefix
    geometry(config)
    parameter = coordinator_parameter(config)
    if not parameter.is_file():
        raise ValueError(f'YP-Spurの実車パラメータがありません: {parameter}')
    for package in ['urg_node', 'livox_ros_driver2', 'rtk_gps_um982', 'ypspur_ros2', 'joy', 'tf2_ros'] + (
            ['usb_cam'] if config['camera']['enabled'] else []):
        get_package_prefix(package)


def main() -> None:
    from ament_index_python.packages import get_package_share_directory
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--site', required=True, choices=['inagi', 'tsukuba'])
    parser.add_argument('--station', help='未指定なら地域の既定局。none/customも指定可能')
    parser.add_argument('--hardware', type=Path)
    parser.add_argument('--ntrip-config', type=Path)
    parser.add_argument('--route-directory', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    shares = [Path(get_package_share_directory(p)) for p in
              ['icart_bringup', 'route_planner', 'fast_lio', 'livox_ros_driver2']]
    prepare_real(*shares, args.output, args.site, args.hardware, args.station,
                 read_yaml(args.ntrip_config) if args.ntrip_config else None, args.route_directory)
    print(f'実機設定を生成しました: {args.output}/session.yaml（未起動）')
