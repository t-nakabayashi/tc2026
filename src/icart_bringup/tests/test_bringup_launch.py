"""launch記述を展開し、実機と模擬の配信元を分離できることを確認する."""
import importlib.util
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
from icart_bringup.session_core import validate_domain


def test_domain_mismatch_and_shared_domain_are_rejected() -> None:
    validate_domain({}, 'simulation', 86)
    validate_domain({}, 'real', 0)
    with pytest.raises(ValueError, match='ROS_DOMAIN_ID'):
        validate_domain({}, 'simulation', 0)
    with pytest.raises(ValueError, match='異なる'):
        validate_domain({'simulation_domain_id': 86, 'real_domain_id': 86}, 'real', 86)


def test_real_launch_has_no_simulator_and_sim_has_one_fusion(monkeypatch, tmp_path: Path) -> None:
    pytest.importorskip('launch_ros')
    from launch import LaunchContext
    from launch_ros.actions import Node
    spec = importlib.util.spec_from_file_location(
        'icart_launch', Path(__file__).parents[1]/'launch/bringup.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    captured = []

    class CapturingNode(Node):
        def __init__(self, **kwargs) -> None:
            captured.append(kwargs)
            super().__init__(**kwargs)

    monkeypatch.setattr(module, 'Node', CapturingNode)
    data = dict(projection_params='projection.yaml', route_config='route.yaml',
                csv_base_dir=str(tmp_path), trial_directory=str(tmp_path), goal_label='10')
    monkeypatch.setattr(module, 'load_session', lambda *_: data)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda _: str(tmp_path))
    for mode, domain in [('real', '0'), ('simulation', '86')]:
        captured.clear()
        monkeypatch.setenv('ROS_DOMAIN_ID', domain)
        context = LaunchContext()
        context.launch_configurations.update(environment=mode, session='session.yaml',
                                              start_ui='true', fusion_log='', initial_drive_mode='autonomous')
        actions = module.setup(context)
        packages = [a.node_package
                    for a in actions if isinstance(a, Node)]
        assert packages.count('gnss_lio_fusion') == 2
        align = next(item for item in captured if item['executable'] == 'gravity_alignment_node')
        fusion = next(item for item in captured if item['executable'] == 'fusion_node')
        assert any(p.get('require_gravity_alignment') for p in fusion['parameters'] if isinstance(p, dict))
        lio = next(item for item in captured if item['executable'] == 'fastlio_mapping')
        assert ('/Odometry', '/lio/odometry_raw') in lio['remappings']
        assert packages.count('drive_mode_manager') == 1
        assert packages.count('fast_lio') == 1
        assert ('obstacle_route_sim' in packages) == (mode == 'simulation')
        assert ('ros_gz_bridge' in packages) == (mode == 'simulation')
        pedestrians = [item for item in captured
                       if item['executable'] == 'pedestrian_simulator_node.py']
        assert len(pedestrians) == int(mode == 'simulation')
        if pedestrians:
            assert pedestrians[0]['parameters'][0]['density'] == .1
            gnss = next(item for item in captured if item['executable'] == 'gnss_simulator_node.py')
            assert gnss['parameters'][1]['start_fix_radius_m'] == 10.
            assert pedestrians[0]['parameters'][0]['world_json'] == str(tmp_path/'world.json')
        follower = next(item for item in captured if item['package'] == 'route_follower')
        assert follower['parameters'][0]['start_immediately'] is False
        navigator = next(item for item in captured if item['package'] == 'robot_navigator')
        assert navigator['parameters'][0]['require_motion_limits'] == (mode == 'real')
        params = {key: value for entry in navigator['parameters'] if isinstance(entry, dict)
                  for key, value in entry.items()}
        assert params['pose_timeout_sec'] == 1.
        assert params['odom_timeout_sec'] == 1.
        ui = next(item for item in captured if item['package'] == 'robot_console')
        assert ui['arguments'] == ['--business-environment',
                                   'デジタルツイン' if mode == 'simulation' else '実機（融合）']
        # UIのGNSS診断購読も、融合・座標変換と同じgps_baseへ向ける。
        base = '/rtk_gps'
        remaps = dict(ui['remappings'])
        assert remaps['rtk_gps/rtk_status'] == base+'/rtk_status'
        assert remaps['rtk_gps/ntrip_status'] == base+'/ntrip_status'


@pytest.mark.parametrize('site,expected', [('稲城', 'inagi'), ('つくば', 'tsukuba')])
def test_gui_survey_uses_selected_site_and_disables_duplicate_ui(monkeypatch, tmp_path, site, expected):
    from launch import LaunchContext
    spec = importlib.util.spec_from_file_location(
        'real_survey_launch', Path(__file__).parents[1]/'launch/real_survey.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    calls = []
    monkeypatch.setattr(module, 'prepare_real', lambda *args, **kwargs: calls.append((args, kwargs)))
    monkeypatch.setattr(module, 'write_yaml', lambda *_: None)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda _: str(tmp_path))
    monkeypatch.setattr(module, 'read_yaml', lambda _: {
        'sites': {'inagi': {'default_station': 'none', 'stations': ['none']},
                  'tsukuba': {'default_station': 'none', 'stations': ['none']}},
        'stations': {'none': {'label': 'NTRIPなし'}}})
    monkeypatch.setenv('ROS_DOMAIN_ID', '0')
    context = LaunchContext()
    context.launch_configurations.update(site=site, station='NTRIPなし', output_root=str(tmp_path),
                                         localization_mode='gnss')
    actions = module.setup(context)
    assert calls[0][0][5] == expected
    assert calls[0][1]['station'] == 'none'
    arguments = dict(actions[0].launch_arguments)
    assert arguments['start_ui'] == 'false'
    assert arguments['environment'] == 'real'
    assert arguments['output'].endswith('/survey')
    assert calls[0][0][4].name == 'session'
    previous = arguments['output']
    assert dict(module.setup(context)[0].launch_arguments)['output'] != previous
    context.launch_configurations['site'] = '場所を選択'
    with pytest.raises(ValueError, match='場所'):
        module.setup(context)


def test_clock_trial_starts_only_two_sensors(monkeypatch, tmp_path):
    from launch import LaunchContext
    spec=importlib.util.spec_from_file_location('clock_sensors',Path(__file__).parents[1]/'launch/clock_sensors.launch.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    monkeypatch.setattr(module,'Node',lambda **kw:kw)
    for name in ['um982.yaml','livox.json']:(tmp_path/name).write_text('{}')
    context=LaunchContext();context.launch_configurations['session_directory']=str(tmp_path)
    actions=module.setup(context)
    assert [a['package']for a in actions]==['rtk_gps_um982','livox_ros_driver2']
    assert actions[0]['parameters'][-1]['time_sync.enabled'] is True
    assert actions[0]['parameters'][-1]['transport_delay_ms']==0
