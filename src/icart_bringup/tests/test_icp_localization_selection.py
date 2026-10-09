"""Compare launch wiring only; never execute launch actions or hardware."""
import importlib.util
from pathlib import Path
import pytest
import yaml

from icart_bringup.localization_config import localization_settings


def module():
    filename = Path(__file__).parents[1]/'launch/bringup.launch.py'
    spec = importlib.util.spec_from_file_location('icp_selection_bringup', filename)
    m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
    return m


def test_only_localizer_is_swapped_same_route_and_control_parameters(monkeypatch, tmp_path):
    from launch import LaunchContext
    m = module()
    data = dict(projection_params='projection.yaml', route_config='route.yaml',
                csv_base_dir=str(tmp_path), goal_label='10')
    monkeypatch.setattr(m, 'load_session', lambda *_: data)
    monkeypatch.setattr(m, 'validate_domain', lambda *_: None)
    monkeypatch.setattr(m, 'get_package_share_directory', lambda _: str(tmp_path))
    monkeypatch.setattr(m, 'localization_settings', lambda _, mode, *args: (mode, {'map_manifest': 'map.json'}))
    original = m.Node
    captured = []
    def capture(**kwargs):
        captured.append(kwargs)
        return original(**kwargs)
    monkeypatch.setattr(m, 'Node', capture)
    graphs = {}
    for mode in ('gnss', 'icp'):
        c = LaunchContext()
        c.launch_configurations.update(environment='real', session='session.yaml', localization_mode=mode,
            initial_drive_mode='autonomous', start_ui='false', fusion_log='')
        captured.clear(); m.setup(c)
        graphs[mode] = {x['package']+'/'+x['executable']: x for x in captured}
    for package, executable in [('route_planner', 'route_planner'), ('route_manager', 'route_manager'),
                                ('route_follower', 'route_follower'), ('robot_navigator', 'robot_navigator')]:
        assert graphs['gnss'][package+'/'+executable] == graphs['icp'][package+'/'+executable]
    assert 'gnss_lio_fusion/fusion_node' not in graphs['icp']
    assert 'icp_localization/localization_node' not in graphs['gnss']
    assert [k for k in graphs['icp'] if k.startswith('icp_localization/')] == ['icp_localization/localization_node']
    mux = 'drive_mode_manager/drive_cmd_mux_node'
    assert graphs['icp'][mux]['parameters'] == graphs['gnss'][mux]['parameters']
    assert graphs['icp'][mux]['remappings'] == [('cmd_vel/autonomous', '/cmd_vel/autonomous')]
    assert graphs['gnss'][mux]['remappings'] == [('cmd_vel/autonomous', '/cmd_vel/fusion_limited')]


def test_mode_validation():
    assert localization_settings({}) == ('gnss', {})
    with pytest.raises(ValueError, match='localization_mode'):
        localization_settings({}, 'other')
    with pytest.raises(ValueError, match='icp_map_manifest'):
        localization_settings({}, 'icp')


def test_icp_gui_profile_uses_dedicated_launch_and_business_mode():
    from robot_console.core.business_mode import LAUNCH_PRESETS
    path = Path(__file__).parents[2]/'robot_console/config/node_launch_profiles.yaml'
    profiles = yaml.safe_load(path.read_text())['profiles']
    profile = next(p for p in profiles if p['profile_id'] == 'icart_icp_route')
    assert profile['launch_file'] == 'icp_route.launch.py'
    assert 'route_directory' in profile['user_arguments']
    assert [p.profile_id for p in LAUNCH_PRESETS[('実機（ICP）', '自律走行')]] == ['icart_icp_route']


def test_gui_and_launch_defaults_select_the_same_map_route_and_backend():
    from launch.actions import DeclareLaunchArgument
    from launch.utilities import perform_substitutions
    from launch import LaunchContext
    from robot_console.core.launch_profile import LaunchProfileStore, build_initial_states, resolve_effective_overrides
    root = Path(__file__).parents[2]
    store = LaunchProfileStore(root/'robot_console/config/node_launch_profiles.yaml')
    store.load()
    profile = store.get('icart_icp_route')
    state = build_initial_states([profile])[profile.profile_id]
    values = resolve_effective_overrides(profile, state)
    filename = Path(__file__).parents[1]/'launch/icp_route.launch.py'
    spec = importlib.util.spec_from_file_location('icp_defaults', filename)
    m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
    description = m.generate_launch_description()
    defaults = {a.name: perform_substitutions(LaunchContext(), a.default_value)
                for a in description.entities if isinstance(a, DeclareLaunchArgument)}
    for key in ('route_directory', 'icp_map_manifest', 'icp_backend_python_path', 'output_root'):
        assert values[key] == defaults[key] and values[key]
