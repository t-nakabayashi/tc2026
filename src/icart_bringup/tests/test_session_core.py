"""セッションの相対パス解決と実機・模擬入力の分離を確認する."""
import json
from pathlib import Path
import sys

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).parents[1]))
from icart_bringup.session_core import load_session, prepare


def test_prepare_and_mode_specific_assets(tmp_path: Path) -> None:
    trial = tmp_path/'trial'
    trial.mkdir()
    (trial/'trial.json').write_text(json.dumps({'projection': {'origin_latitude': 36., 'origin_longitude': 140.},
                                              'points': [[0, 0], [1, 0]]}))
    (trial/'route.csv').write_text('label,x,y,z,q1,q2,q3,q4\n0,0,0,0,0,0,0,1\n1,1,0,0,0,0,0,1\n')
    (trial/'trial.sdf').write_text('<sdf><world><physics><max_step_size>0.002</max_step_size></physics></world></sdf>')
    output = tmp_path/'session'
    prepare(trial, output)
    config = output/'session.yaml'
    assert '../trial' in config.read_text()
    assert load_session(config, 'real')['goal_label'] == '1'
    with pytest.raises(ValueError, match='world.json'):
        load_session(config, 'simulation')
    (trial/'trial.sdf').write_text('<sdf/>')
    (trial/'world.json').write_text('{}')
    assert load_session(config, 'simulation')['csv_base_dir'] == str(output/'routes')
    with pytest.raises(ValueError, match='environment'):
        load_session(config, 'reel')
    with pytest.raises(FileExistsError):
        prepare(trial, output)


def test_missing_projection_is_not_silently_replaced(tmp_path: Path) -> None:
    (tmp_path/'trial.json').write_text('{"points": [[0,0]]}')
    with pytest.raises(ValueError, match='投影原点'):
        prepare(tmp_path, tmp_path/'session')


def test_receiver_node_namespace_in_saved_session_resolves_to_public_topic_base(tmp_path):
    for name in ('projection.yaml', 'route.yaml'):
        (tmp_path/name).write_text('{}')
    config = tmp_path/'session.yaml'
    config.write_text(yaml.safe_dump(dict(projection_params='projection.yaml', route_config='route.yaml',
        csv_base_dir='.', gnss_namespace='/rtk_gps/rtk_gps_um982_node')))
    assert load_session(config, 'real')['gnss_namespace'] == '/rtk_gps'


def test_simulation_does_not_require_real_driver_files(tmp_path: Path) -> None:
    """共有sessionに実車専用ファイル名があっても模擬環境では開かない."""
    trial = tmp_path/'trial'
    trial.mkdir()
    for name in ['trial.sdf', 'world.json', 'trial.json', 'projection.yaml', 'route.yaml']:
        (trial/name).write_text('{}')
    config = tmp_path/'session.yaml'
    config.write_text(yaml.safe_dump(dict(
        projection_params='trial/projection.yaml', route_config='trial/route.yaml',
        csv_base_dir='trial', trial_directory='trial', hardware_launch='real_driver.launch.py')))
    assert load_session(config, 'simulation')['trial_directory'] == str(trial)
    with pytest.raises(ValueError, match='hardware_launch'):
        load_session(config, 'real')


@pytest.mark.parametrize('environment,domain', [('simulation', '86'), ('real', '0')])
def test_run_entry_selects_domain_before_exec(monkeypatch, tmp_path: Path,
                                             environment: str, domain: str) -> None:
    """入口が実機と模擬を別domainで起動し、同じsessionを渡すことを確認する."""
    import os
    from icart_bringup import session_core

    monkeypatch.setenv('ROS_DOMAIN_ID', '99')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'SUBNET')
    monkeypatch.setattr(session_core, 'load_session', lambda *_: {})
    monkeypatch.setattr(sys, 'argv', ['run_session', '--session', str(tmp_path/'session.yaml'),
                                    '--environment', environment, '--start-ui'])
    captured = []
    monkeypatch.setattr(os, 'execvp', lambda executable, args: captured.append((
        executable, args, os.environ['ROS_DOMAIN_ID'],
        os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'])))
    session_core.run_main()
    executable, args, actual_domain, discovery = captured[0]
    assert executable == 'ros2' and actual_domain == domain
    assert 'environment:='+environment in args and 'start_ui:=true' in args
    assert discovery == ('LOCALHOST' if environment == 'simulation' else 'SUBNET')
