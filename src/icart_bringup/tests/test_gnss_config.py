import importlib.util
from pathlib import Path

import pytest
from icart_bringup.gnss_config import resolve_selection, gnss_parameters
from icart_bringup.hardware_core import read_yaml, prepare_real

ROOT = Path(__file__).resolve().parents[3]
SHARE = ROOT/'src/icart_bringup'
CATALOG = read_yaml(SHARE/'params/rtk_stations.yaml')
HARDWARE = read_yaml(SHARE/'params/hardware.yaml')


@pytest.mark.parametrize('site,station', [('稲城', 'tokyo_c4cae992'), ('つくば', 'joso_yasuda'),
                                        ('稲城', 'NTRIPなし'), ('つくば', '地域の既定局')])
def test_standalone_and_shared_session_use_identical_receiver_and_station_settings(
        monkeypatch, tmp_path, site, station):
    from launch import LaunchContext
    spec = importlib.util.spec_from_file_location('gnss_launch', SHARE/'launch/gnss.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda _: str(SHARE))
    monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
    monkeypatch.setenv('ROS_DOMAIN_ID', '0')
    context = LaunchContext()
    context.launch_configurations.update(site=site, station=station, config='')
    node = module.setup(context)[0]
    site_id, station_id = resolve_selection(CATALOG, site, station)
    prepare_real(SHARE, ROOT/'src/route_planner', ROOT/'src/FAST_LIO',
                 ROOT/'src/livox_ros_driver2', tmp_path/'session', site_id, station=station_id)
    common = read_yaml(tmp_path/'session/um982.yaml')['/rtk_gps/rtk_gps_um982_node']['ros__parameters']
    assert node['parameters'][0] == common
    assert node['namespace'] == 'rtk_gps'
    assert common['time_sync.enabled'] and common['transport_delay_ms'] == 0


def test_station_selection_rejects_wrong_region_and_custom_without_credentials():
    with pytest.raises(ValueError, match='場所'):
        resolve_selection(CATALOG, '場所を選択', '地域の既定局')
    with pytest.raises(ValueError, match='場所'):
        resolve_selection(CATALOG, '稲城', 'joso_yasuda')
    with pytest.raises(ValueError, match='ntrip-config'):
        gnss_parameters(CATALOG, HARDWARE, 'inagi', 'custom')
    params = gnss_parameters(CATALOG, HARDWARE, 'inagi', 'none')
    assert not params['ntrip.enabled'] and params['ntrip.host'] == ''


def test_custom_station_uses_validated_private_configuration():
    params = gnss_parameters(CATALOG, HARDWARE, 'tsukuba', 'custom',
                             dict(host='example.invalid', mountpoint='TEST', user='user', password='secret'))
    assert params['ntrip.enabled'] and params['ntrip.station_id'] == 'custom'
    assert params['ntrip.password'] == 'secret'
