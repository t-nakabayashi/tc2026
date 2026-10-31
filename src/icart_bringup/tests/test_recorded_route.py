import importlib.util
import json
from pathlib import Path
import numpy as np
import pytest
import yaml
from route_survey.storage_core import save
from geo_pose_converter.geo_core import ProjectionConfig
from icart_bringup.recorded_route import inspect_recorded_route
from icart_bringup.session_core import load_session
from route_planner.route_builder import RouteBuilder

ROOT = Path(__file__).resolve().parents[3]

@pytest.fixture
def recorded(tmp_path):
    directory = tmp_path/'record/survey'
    directory.mkdir(parents=True)
    session = directory.parent/'session'
    session.mkdir()
    (session/'hardware.yaml').write_text((ROOT/'src/icart_bringup/params/hardware.yaml').read_text())
    (session/'session.yaml').write_text(yaml.safe_dump(dict(site='inagi', rtk_station='none')))
    rows = [dict(x=x, y=y, yaw=0., right_is_open=0., left_is_open=0., line_is_stop=False,
                 signal_is_stop=False, isnot_skipnum=True) for x,y in [(0,0),(5,0),(5,5)]]
    save(directory, rows, ProjectionConfig(35.65,139.50,0.), np.empty((0,3)), False)
    return directory


def test_reject_active_and_one_point_route(recorded):
    info = inspect_recorded_route(recorded)
    assert info['count'] == 3 and info['start'] == '0' and info['goal'] == '2'
    assert info['unknown_width'] == 3
    metadata = recorded/'survey.json'
    data = json.loads(metadata.read_text());data['active']=True;metadata.write_text(json.dumps(data))
    with pytest.raises(ValueError, match='終了・保存'):
        inspect_recorded_route(recorded)
    data['active']=False;metadata.write_text(json.dumps(data))
    csv = recorded/'fixed/waypoints.csv'
    csv.write_text('\n'.join(csv.read_text().splitlines()[:2])+'\n')
    with pytest.raises(ValueError, match='2点'):
        inspect_recorded_route(recorded)


def test_launch_preserves_selected_route_and_waits_for_start(recorded, tmp_path, monkeypatch):
    from launch import LaunchContext
    spec=importlib.util.spec_from_file_location('recorded_launch',ROOT/'src/icart_bringup/launch/recorded_route.launch.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    packages={'icart_bringup':'icart_bringup','route_planner':'route_planner','fast_lio':'FAST_LIO','livox_ros_driver2':'livox_ros_driver2'}
    monkeypatch.setattr(module,'get_package_share_directory',lambda p:str(ROOT/'src'/packages[p]))
    monkeypatch.setenv('ROS_DOMAIN_ID','0')
    context=LaunchContext();context.launch_configurations.update(route_directory=str(recorded),output_root=str(tmp_path/'runs'), antenna_baseline_m='0.15', master_forward_m='-0.30')
    action=module.setup(context)[0]
    args=dict(action.launch_arguments)
    assert args['start_ui']=='false' and args['initial_drive_mode']=='autonomous'
    # 自律走行は既定でrosbagを記録する。保存先は媒体を指定できるよう素通しする。
    assert args['record_bag']=='true'
    assert args['bag_directory']==''
    session=load_session(Path(args['session']),'real')
    from geo_pose_converter.geo_core import load_projection_config_from_yaml
    builder=RouteBuilder(session['route_config'],session['csv_base_dir'],
                         projection=load_projection_config_from_yaml(session['projection_params']))
    builder.load()
    route=builder.build_route(session['start_label'],session['goal_label'],[])
    assert len(route.waypoints)==3
    assert session['route_source']==str(recorded)
    params=yaml.safe_load(Path(session['fusion_params']).read_text())['gnss_lio_fusion']['ros__parameters']
    assert params['baseline.initial_m']==pytest.approx(.15)
    assert params['baseline.minimum_m']==pytest.approx(.105)
    assert params['baseline.maximum_m']==pytest.approx(.195)
    assert params['master_forward_m']==pytest.approx(-.30)
    hardware=yaml.safe_load(Path(session['hardware_config']).read_text())
    from icart_bringup.hardware_core import geometry
    assert geometry(hardware)['slave'][0]==pytest.approx(-.45)
    from gnss_lio_fusion.baseline_core import BaselineConfig
    from gnss_lio_fusion.fusion_core import FusionFilter
    config=BaselineConfig(initial_m=params['baseline.initial_m'], minimum_m=params['baseline.minimum_m'], maximum_m=params['baseline.maximum_m'])
    fusion=FusionFilter(baseline=config)
    fusion.advance(0.,np.zeros(3))
    assert fusion.observe_gps(0.,np.zeros(3),4,28,.1553,.02**2,.5)
    assert fusion.x is not None
    assert Path(session['projection_params']).read_text()==(recorded/'projection.yaml').read_text() or yaml.safe_load(Path(session['projection_params']).read_text())==yaml.safe_load((recorded/'projection.yaml').read_text())


def test_launch_passes_through_the_bag_destination(recorded, tmp_path, monkeypatch):
    """保存先と記録有無の指定が共通起動へそのまま渡る。

    内蔵ディスクに長時間の走行を書くと容量が尽きるため、運用では外部媒体を指定する。
    """
    from launch import LaunchContext
    spec=importlib.util.spec_from_file_location('recorded_launch2',ROOT/'src/icart_bringup/launch/recorded_route.launch.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    packages={'icart_bringup':'icart_bringup','route_planner':'route_planner','fast_lio':'FAST_LIO','livox_ros_driver2':'livox_ros_driver2'}
    monkeypatch.setattr(module,'get_package_share_directory',lambda p:str(ROOT/'src'/packages[p]))
    monkeypatch.setenv('ROS_DOMAIN_ID','0')
    context=LaunchContext()
    context.launch_configurations.update(route_directory=str(recorded),
                                         output_root=str(tmp_path/'runs'),
                                         antenna_baseline_m='0', master_forward_m='',
                                         record_bag='false',
                                         bag_directory=str(tmp_path/'media'))

    args=dict(module.setup(context)[0].launch_arguments)

    assert args['record_bag']=='false'
    assert args['bag_directory']==str(tmp_path/'media')
