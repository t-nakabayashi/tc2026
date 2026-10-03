"""昨年度の構成・新しいアンテナ幾何・地域選択・実機起動の接続を検証する。"""
import importlib.util
from pathlib import Path

import numpy as np
import pytest
import yaml
from scipy.spatial.transform import Rotation
from icart_bringup.hardware_core import geometry, read_yaml, prepare_real, station_config
from icart_bringup.session_core import load_session
from route_planner.route_builder import RouteBuilder
from geo_pose_converter.geo_core import load_projection_config_from_yaml

ROOT = Path(__file__).resolve().parents[3]
SHARE = ROOT/'src/icart_bringup'


def test_geometry_uses_vertical_antenna_offset_not_tilted_sensor_z():
    config = read_yaml(SHARE/'params/hardware.yaml')
    geo = geometry(config)
    assert np.array(geo['master'])-geo['lidar'] == pytest.approx([-.3, 0., .15])
    assert np.array(geo['slave'])-geo['master'] == pytest.approx([-.15, 0., 0.])
    assert np.array(geo['imu']) + Rotation.from_euler('xyz', [-.6, 26.9, 0.], degrees=True).apply(
        config['mid360']['lidar_in_imu_xyz']) == pytest.approx(geo['lidar'])


@pytest.mark.parametrize('site', ['inagi', 'tsukuba'])
def test_generated_session_loads_route_and_matching_mount(tmp_path, site):
    out = tmp_path/site
    prepare_real(SHARE, ROOT/'src/route_planner', ROOT/'src/FAST_LIO', ROOT/'src/livox_ros_driver2', out, site)
    session = load_session(out/'session.yaml', 'real')
    projection = load_projection_config_from_yaml(session['projection_params'])
    builder = RouteBuilder(session['route_config'], session['csv_base_dir'], projection=projection)
    builder.load()
    assert len(builder.build_route(session['start_label'], session['goal_label'], []).waypoints) >= 2
    assert session['route_review_required']
    fusion = read_yaml(out/'fusion.yaml')['gnss_lio_fusion']['ros__parameters']
    survey = read_yaml(out/'recorder.yaml')['route_survey']['ros__parameters']
    assert fusion['lio_mount_pitch_deg'] == survey['lidar_mount_pitch_deg'] == 26.9
    assert fusion['lio_mount_roll_deg'] == survey['lidar_mount_roll_deg'] == -.6
    assert fusion['master_height_m'] == pytest.approx(.564)
    assert fusion['lio_height_m'] == survey['lidar_height_m']
    assert fusion['gnss_heading_offset_deg'] == 180.
    assert (out/'um982.yaml').stat().st_mode & 0o777 == 0o600
    with pytest.raises(FileExistsError):
        prepare_real(SHARE, ROOT/'src/route_planner', ROOT/'src/FAST_LIO', ROOT/'src/livox_ros_driver2', out, site)


def test_station_rejects_wrong_region_and_missing_custom():
    catalog = read_yaml(SHARE/'params/rtk_stations.yaml')
    with pytest.raises(ValueError):
        station_config(catalog, 'inagi', 'tsukuba_takashima')
    with pytest.raises(ValueError):
        station_config(catalog, 'inagi', 'custom')
    assert not station_config(catalog, 'inagi', 'none')[1]['enabled']


def test_survey_delegates_single_joy_and_manual_lock(monkeypatch, tmp_path):
    from launch import LaunchContext
    from launch_ros.actions import Node
    spec = importlib.util.spec_from_file_location('survey_launch', SHARE/'launch/survey.launch.py')
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'load_session', lambda *_: dict(projection_params='projection.yaml'))
    context = LaunchContext()
    context.launch_configurations.update(environment='real', session='session.yaml',
        output=str(tmp_path/'record'), joy_input='external', recorder_params='recorder.yaml')
    actions = module.setup(context)
    assert [a.node_package for a in actions if isinstance(a, Node)] == ['route_survey']
    args = dict(actions[0].launch_arguments)
    assert args['allow_auto_resume'] == 'false'
    assert args['start_teleop'] == 'true' and args['joy_input'] == 'external'


@pytest.mark.parametrize('use_symlink', [False, True])
def test_real_hardware_graph_has_expected_drivers_and_no_mux(monkeypatch, tmp_path, use_symlink):
    from launch import LaunchContext
    from launch_ros.actions import Node
    spec = importlib.util.spec_from_file_location('hardware_launch', SHARE/'launch/hardware.launch.py')
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    config = read_yaml(SHARE/'params/hardware.yaml')
    device = tmp_path/'dev/video4'
    device.parent.mkdir()
    device.touch()
    stable = tmp_path/'dev/v4l/by-id/camera'
    stable.parent.mkdir(parents=True)
    stable.symlink_to('../../video4')
    config['camera']['device'] = str(stable if use_symlink else device)
    monkeypatch.setattr(module, 'read_yaml', lambda _: config)
    monkeypatch.setattr(module, 'validate_runtime', lambda _: None)
    captured = []
    class Capture(Node):
        def __init__(self, **kwargs):
            captured.append(kwargs)
            super().__init__(**kwargs)
    monkeypatch.setattr(module, 'Node', Capture)
    context = LaunchContext(); context.launch_configurations['hardware_config'] = str(tmp_path/'hardware.yaml')
    module.setup(context)
    drivers = [k for k in captured if k['package'] != 'tf2_ros']
    assert {k['package'] for k in drivers} == {
        'ypspur_ros2', 'urg_node', 'rtk_gps_um982', 'livox_ros_driver2', 'usb_cam'}
    livox = next(k for k in drivers if k['package'] == 'livox_ros_driver2')
    assert livox['parameters'][0]['xfer_format'] == 1
    assert livox['namespace'] == 'mid360'
    gnss = next(k for k in drivers if k['package'] == 'rtk_gps_um982')
    assert gnss['namespace'] == 'rtk_gps'
    wheel = next(k for k in drivers if k['package'] == 'ypspur_ros2')
    assert ('odom', '/ypspur_ros/odom') in wheel['remappings']
    urg = next(k for k in drivers if k['package'] == 'urg_node')
    assert urg.get('namespace', '') == ''
    assert urg['parameters'][0]['laser_frame_id'] == 'laser'
    camera = next(k for k in drivers if k['package'] == 'usb_cam')
    assert camera['remappings'] == [('image_raw', '/usb_cam/image_raw')]
    assert camera['parameters'][0]['video_device'] == str(device)
    sensor_tfs = [k for k in captured if k['package'] == 'tf2_ros']
    assert len(sensor_tfs) == 5
    assert all('body' not in k['arguments'] for k in sensor_tfs)


@pytest.mark.parametrize('filename,xfer_format', [('msg_MID360_launch.py', 1), ('rviz_MID360_launch.py', 0)])
def test_mid360_standalone_and_lio_use_shared_topics(monkeypatch, filename, xfer_format):
    import yaml
    path = ROOT/'src/livox_ros_driver2/launch'/filename
    spec = importlib.util.spec_from_file_location('mid360_standalone', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    captured = []
    from launch_ros.actions import Node
    class Capture(Node):
        def __init__(self, **kwargs):
            captured.append(kwargs)
            super().__init__(**kwargs)
    monkeypatch.setattr(module, 'Node', Capture)
    module.generate_launch_description()
    assert captured[0]['namespace'] == 'mid360'
    params = {key: value for item in captured[0]['parameters'] for key, value in item.items()}
    assert params['frame_id'] == 'mid360_frame' and params['xfer_format'] == xfer_format
    if xfer_format == 0:
        display = Path(module.rviz_config_path).read_text()
        assert 'Topic: /mid360/livox/lidar' in display
        assert 'Fixed Frame: mid360_frame' in display
    lio = yaml.safe_load((ROOT/'src/FAST_LIO/config/mid360.yaml').read_text())
    assert lio['/**']['ros__parameters']['common']['lid_topic'] == '/mid360/livox/lidar'
    assert lio['/**']['ros__parameters']['common']['imu_topic'] == '/mid360/livox/imu'


def test_custom_survey_projection_is_preserved(tmp_path):
    from route_survey.storage_core import save
    from geo_pose_converter.geo_core import ProjectionConfig
    survey = tmp_path/'survey'; survey.mkdir()
    rows = [dict(x=float(x), y=0., yaw=.1, right_is_open=0., left_is_open=0.,
        line_is_stop=0, signal_is_stop=0, isnot_skipnum=1) for x in [0, 5]]
    save(survey, rows, ProjectionConfig(35.65, 139.50, 50., map_yaw_offset_rad=.2),
         np.empty((0, 3)), False)
    out = tmp_path/'session'
    prepare_real(SHARE, ROOT/'src/route_planner', ROOT/'src/FAST_LIO', ROOT/'src/livox_ros_driver2',
                 out, 'inagi', route_directory=survey)
    restored = load_projection_config_from_yaml(str(out/'projection.yaml'))
    assert restored.origin_altitude == 50.
    assert restored.map_yaw_offset_rad == .2


def test_coordinator_parameter_resolves_package_and_legacy_paths(monkeypatch, tmp_path):
    from ament_index_python import packages
    from icart_bringup.hardware_core import coordinator_parameter
    monkeypatch.setattr(packages, 'get_package_share_directory', lambda name: str(tmp_path/name))
    config = read_yaml(SHARE/'params/hardware.yaml')
    assert coordinator_parameter(config) == tmp_path/'ypspur_ros2/config/icart-middle.param'
    config['wheel']['coordinator_param'] = str(tmp_path/'custom.param')
    assert coordinator_parameter(config) == tmp_path/'custom.param'


def test_mid360_local_map_covers_reference_2026_course_without_sliding():
    import csv
    from scipy.spatial.distance import pdist
    params = read_yaml(ROOT/'src/FAST_LIO/config/mid360.yaml')['/**']['ros__parameters']
    with (ROOT/'src/route_planner/routes/tsukuba2026_digital_twin/fixed/waypoints.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    ll = np.array([[float(row['longitude']), float(row['latitude'])] for row in rows])
    xy = np.deg2rad(ll-ll[0])*6378137.*[np.cos(np.deg2rad(ll[:, 1].mean())), 1.]
    # Diameter bounds displacement from any course starting point, at any yaw.
    diameter = pdist(xy).max()
    slide_margin = 1.5*params['mapping']['det_range']
    assert params['cube_side_length']/2 > diameter+slide_margin
    assert params['preprocess']['blind'] == .7


def test_main_antenna_forward_offset_is_absolute_and_legacy_is_preserved():
    config = read_yaml(SHARE/'params/hardware.yaml')
    config['mid360']['xyz'][0] = .1
    geo = geometry(config)
    assert geo['master'][0] == pytest.approx(-.3)
    assert geo['slave'][0] == pytest.approx(-.45)
    assert geo['baseline_m'] == pytest.approx(.15)
    del config['gnss']['master_forward_m']
    assert geometry(config)['master'][0] == pytest.approx(.1)
    config['gnss']['master_forward_m'] = float('nan')
    with pytest.raises(ValueError, match='主アンテナ'):
        geometry(config)
