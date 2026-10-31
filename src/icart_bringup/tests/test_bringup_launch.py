"""launch記述を展開し、実機と模擬の配信元を分離できることを確認する."""
import importlib.util
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
from icart_bringup.session_core import validate_domain


def _all_actions(actions) -> list:
    """RegisterEventHandler の内側も含めて action を平坦化する。

    実機構成では、時刻同期の待ち合わせが成立した後に走行系の action を返す。
    launch を実行せずに中身を見るため、成立時のコールバックをここで呼び出す。
    """
    import types
    from launch.actions import RegisterEventHandler
    result = []
    for action in actions:
        result.append(action)
        if not isinstance(action, RegisterEventHandler):
            continue
        # launch は on_exit のコールバックを非公開属性に持ち、handler_description は
        # 文字列を返す。launch を実行せずに中身を見るため、ここだけ属性名を直接使う。
        callback = getattr(action.event_handler, '_OnActionEventBase__on_event', None)
        if not callable(callback):
            continue
        produced = callback(types.SimpleNamespace(returncode=0), None)
        if isinstance(produced, list):
            result.extend(produced)
    return result


def _parameter(captured_node: dict, name: str):
    """Node へ渡したパラメータ辞書から値を取り出す."""
    for entry in captured_node['parameters']:
        if isinstance(entry, dict) and name in entry:
            return entry[name]
    return None


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


def _real_hardware_setup(monkeypatch, tmp_path: Path):
    """launch を展開する補助。生成された Node の引数を記録する."""
    pytest.importorskip('launch_ros')
    from launch import LaunchContext
    from launch_ros.actions import Node
    spec = importlib.util.spec_from_file_location(
        'icart_launch_hw', Path(__file__).parents[1]/'launch/bringup.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    captured = []

    class CapturingNode(Node):
        def __init__(self, **kwargs) -> None:
            captured.append(kwargs)
            super().__init__(**kwargs)

    monkeypatch.setattr(module, 'Node', CapturingNode)
    data = dict(projection_params='projection.yaml', route_config='route.yaml',
                csv_base_dir=str(tmp_path), goal_label='10',
                trial_directory=str(tmp_path),
                hardware_config=str(tmp_path/'hardware.yaml'),
                hardware_launch=str(tmp_path/'hardware.launch.py'))
    monkeypatch.setattr(module, 'load_session', lambda *_: data)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda _: str(tmp_path))
    monkeypatch.setattr(module, 'read_yaml',
                        lambda _: {'joy': {'enable_button': 4, 'ps_button': 10}})
    monkeypatch.setattr(module, 'validate_runtime', lambda _: None)
    monkeypatch.setenv('ROS_DOMAIN_ID', '0')
    return module, LaunchContext(), captured


def _command_text(action) -> str:
    """ExecuteProcess の cmd を文字列へ戻す。cmd の各要素は置換の列である."""
    words = []
    for item in action.cmd:
        parts = item if isinstance(item, (list, tuple)) else [item]
        words.append(''.join(str(getattr(part, 'text', part)) for part in parts))
    return ' '.join(words)


def _bag_recorders(actions) -> list:
    """走行記録を開始する action を取り出す."""
    from launch.actions import ExecuteProcess
    return [a for a in actions if isinstance(a, ExecuteProcess)
            and 'icart_bringup.bag_record' in _command_text(a)]


def test_real_run_records_a_bag_and_keeps_ai_advice_off(monkeypatch, tmp_path: Path) -> None:
    """実機の自律走行では bag を自動記録し、AI 助言は使わない.

    助言なしで回避が作動する記録を残し、後から bag 再生で助言の有効性を比べるため、
    実機では助言を無効のまま走る。記録は操作者の開始操作に依存させない。
    """
    module, context, captured = _real_hardware_setup(monkeypatch, tmp_path)
    context.launch_configurations.update(environment='real', session='session.yaml',
                                         start_ui='false', fusion_log='',
                                         initial_drive_mode='autonomous')

    actions = module.setup(context)

    follower = next(item for item in captured if item['package'] == 'route_follower')
    assert _parameter(follower, 'use_ai_advice') is False
    assert len(_bag_recorders(_all_actions(actions))) == 1


def test_recording_can_be_turned_off_and_advice_cannot_be_turned_on(
        monkeypatch, tmp_path: Path) -> None:
    """記録は引数で止められる。実機では助言を引数で有効にできない."""
    module, context, captured = _real_hardware_setup(monkeypatch, tmp_path)
    context.launch_configurations.update(environment='real', session='session.yaml',
                                         start_ui='false', fusion_log='',
                                         initial_drive_mode='autonomous',
                                         record_bag='false', use_ai_advice='true')

    actions = module.setup(context)

    follower = next(item for item in captured if item['package'] == 'route_follower')
    assert _parameter(follower, 'use_ai_advice') is False
    assert _bag_recorders(_all_actions(actions)) == []


def test_simulation_does_not_record_and_can_enable_advice(monkeypatch, tmp_path: Path) -> None:
    """模擬環境は自動記録せず、助言を引数で有効にできる。

    助言を有効にしたときだけ、層スライスと助言のノードを起動する。層の観測値は回避幅の
    上限に効くため、助言なしの基準走行では起動しない。
    """
    module, context, captured = _real_hardware_setup(monkeypatch, tmp_path)
    monkeypatch.setenv('ROS_DOMAIN_ID', '86')
    context.launch_configurations.update(environment='simulation', session='session.yaml',
                                         start_ui='false', fusion_log='',
                                         initial_drive_mode='autonomous',
                                         use_ai_advice='true')

    actions = module.setup(context)

    follower = next(item for item in captured if item['package'] == 'route_follower')
    assert _parameter(follower, 'use_ai_advice') is True
    assert _bag_recorders(_all_actions(actions)) == []
    packages = [item['package'] for item in captured]
    assert packages.count('lidar_layer_slicer') == 1
    assert packages.count('avoidance_advisor') == 1


def test_advice_nodes_are_absent_when_advice_is_disabled(monkeypatch, tmp_path: Path) -> None:
    """助言が無効なら、層スライスと助言のノードを起動しない。

    層の観測値は回避幅の上限に効く。助言なしの走行を従来の挙動のまま記録するため、
    基準の走行ではこれらを動かさない。層は記録したbagから再現できる。
    """
    module, context, captured = _real_hardware_setup(monkeypatch, tmp_path)
    monkeypatch.setenv('ROS_DOMAIN_ID', '86')
    context.launch_configurations.update(environment='simulation', session='session.yaml',
                                         start_ui='false', fusion_log='',
                                         initial_drive_mode='autonomous')

    module.setup(context)

    packages = [item['package'] for item in captured]
    assert 'lidar_layer_slicer' not in packages
    assert 'avoidance_advisor' not in packages


def test_advice_nodes_do_not_stop_the_stack_when_they_exit(monkeypatch, tmp_path: Path) -> None:
    """助言ノードの終了で走行全体を止めない。

    共通起動は主要子プロセスの終了で全体を停止する。助言は走行の付帯機能であり、
    停止の理由にしない。launchのNodeもこの登録の対象になるため、登録の後に追加する。
    """
    from launch.actions import RegisterEventHandler
    from launch_ros.actions import Node
    module, context, _ = _real_hardware_setup(monkeypatch, tmp_path)
    monkeypatch.setenv('ROS_DOMAIN_ID', '86')
    context.launch_configurations.update(environment='simulation', session='session.yaml',
                                         start_ui='false', fusion_log='',
                                         initial_drive_mode='autonomous',
                                         use_ai_advice='true')

    actions = module.setup(context)

    watched = {id(getattr(action.event_handler, '_OnActionEventBase__action_matcher', None))
               for action in actions if isinstance(action, RegisterEventHandler)}
    advice = [action for action in actions if isinstance(action, Node)
              and action.node_package in ('lidar_layer_slicer', 'avoidance_advisor')]
    assert len(advice) == 2
    assert all(id(action) not in watched for action in advice)


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
