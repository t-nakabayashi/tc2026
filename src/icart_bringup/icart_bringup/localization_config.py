"""同一経路・投影を保ったGNSS/ICP起動設定。起動前に地図座標系を検査する。"""
from pathlib import Path
from geo_pose_converter.geo_core import load_projection_config_from_yaml
from .hardware_core import geometry, read_yaml


def localization_settings(data, mode='', manifest='', backend=''):
    mode = mode or data.get('localization_mode', 'gnss')
    if mode not in ('gnss', 'icp'):
        raise ValueError('localization_mode must be gnss or icp')
    if mode == 'gnss':
        return mode, {}
    manifest = manifest or data.get('icp_map_manifest', '')
    if not manifest:
        raise ValueError('ICP走行にはicp_map_manifestが必要です')
    from icp_localization.registration import load_manifest
    projection = load_projection_config_from_yaml(data['projection_params'])
    manifest = str(Path(manifest).expanduser().resolve())
    load_manifest(manifest, vars(projection))
    if data.get('hardware_config'):
        hardware = read_yaml(Path(data['hardware_config']))
        geo = geometry(hardware)
        mount = dict(base_imu_xyz=geo['imu'], base_imu_rpy_deg=geo['rpy_deg'],
                     antenna_xyz=geo['master'], gnss_heading_offset_deg=float(hardware['gnss']['heading_offset_deg']))
    else:
        raise ValueError('ICP走行にはセンサ取付位置のhardware_configが必要です')
    return mode, dict(mount, map_manifest=manifest,
                      backend_python_path=backend or data.get('icp_backend_python_path', ''))
