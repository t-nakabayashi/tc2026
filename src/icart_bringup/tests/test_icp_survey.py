"""採取の既定ICP、座標原点、GNSS選択を起動せずに検証する。"""
import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions
from geo_pose_converter.geo_core import ProjectionConfig, load_projection_config_from_yaml
from icart_bringup.session_core import load_session
from icart_bringup.hardware_core import prepare_real
from icp_localization.registration import sha256
from robot_console.core.business_mode import get_preset
from robot_console.core.launch_profile import LaunchProfileStore, build_initial_states, resolve_effective_overrides

ROOT=Path(__file__).resolve().parents[3]
SHARE=ROOT/'src/icart_bringup'


def launch_module():
    spec=importlib.util.spec_from_file_location('real_survey_test',SHARE/'launch/real_survey.launch.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


def defaults(module):
    return {a.name:perform_substitutions(LaunchContext(),a.default_value)
            for a in module.generate_launch_description().entities if isinstance(a,DeclareLaunchArgument)}


def test_gui_matches_cli_defaults_and_explicit_modes():
    values=defaults(launch_module())
    store=LaunchProfileStore(ROOT/'src/robot_console/config/node_launch_profiles.yaml');store.load()
    profile=store.get('icart_real_survey')
    effective=resolve_effective_overrides(profile,build_initial_states([profile])[profile.profile_id])
    for key in ('localization_mode','icp_map_manifest','icp_backend_python_path','output_root'):
        assert values[key]==effective[key] and values[key]
    assert values['localization_mode']=='icp'
    for env,mode in [('実機','icp'),('実機（ICP）','icp'),('実機（融合）','gnss')]:
        assert get_preset(env,'手動走行')[0].overrides['localization_mode']==mode


@pytest.mark.parametrize('mode',['icp','gnss'])
def test_generated_survey_session_uses_selected_map_projection(tmp_path,monkeypatch,mode):
    m=launch_module()
    shares={'fast_lio':ROOT/'src/FAST_LIO'}
    monkeypatch.setattr(m,'get_package_share_directory',lambda p:str(shares.get(p,ROOT/'src'/p)))
    monkeypatch.setattr(m,'validate_domain',lambda *_:None)
    projection=ProjectionConfig(36.082911197,140.07683123183332,26.7597,
                                projection_id='test_constrained_map')
    np.save(tmp_path/'map.npy',np.zeros((100,4)));np.save(tmp_path/'anchors.npy',np.eye(4)[None])
    manifest=tmp_path/'map_manifest.json'
    manifest.write_text(json.dumps(dict(schema=1,projection=vars(projection),
        points=dict(path='map.npy',sha256=sha256(tmp_path/'map.npy')),
        base_anchors=dict(path='anchors.npy',sha256=sha256(tmp_path/'anchors.npy')))))
    context=LaunchContext()
    context.launch_configurations.update(defaults(m))
    context.launch_configurations.update(site='つくば',localization_mode=mode,
        icp_map_manifest=str(manifest) if mode=='icp' else '/missing/unused.json',
        icp_backend_python_path='',output_root=str(tmp_path/'sessions'))
    actions=m.setup(context)
    args=dict(actions[0].launch_arguments)
    session=load_session(Path(args['session']),'real')
    assert session['localization_mode']==mode
    assert args['joy_input']=='joy_node'
    actual=load_projection_config_from_yaml(session['projection_params'])
    if mode=='icp':
        assert actual==projection
        assert session['icp_map_manifest']==str(manifest)
    else:
        assert 'icp_map_manifest' not in session
    assert (Path(args['output']).parent/'session/recorder.yaml').is_file()


def test_wrong_site_is_rejected_before_creating_survey(tmp_path):
    with pytest.raises(ValueError,match='場所とICP地図'):
        prepare_real(SHARE,ROOT/'src/route_planner',ROOT/'src/FAST_LIO',ROOT/'src/livox_ros_driver2',
                     tmp_path/'session','inagi',survey_projection=ProjectionConfig(36.08,140.08,20.))
    assert not (tmp_path/'session').exists()
