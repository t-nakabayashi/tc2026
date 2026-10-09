import json
from pathlib import Path

import numpy as np
import pytest
import yaml
from PyQt5 import QtGui, QtWidgets
from geo_pose_converter.geo_core import ProjectionConfig
from icp_localization.registration import sha256
from robot_console.ui_qt.map_creation_tab import MapCreationTab
from robot_console.ui_qt.launch_settings_tab import LaunchSettingsTab


@pytest.fixture(scope='module')
def qt_app():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


def completed_map(directory):
    projection = vars(ProjectionConfig(36., 140., 20.))
    np.save(directory/'map.npy', np.zeros((100, 4)))
    np.save(directory/'anchors.npy', np.eye(4)[None])
    manifest = dict(schema=1, projection=projection,
        points=dict(path='map.npy', sha256=sha256(directory/'map.npy')),
        base_anchors=dict(path='anchors.npy', sha256=sha256(directory/'anchors.npy')))
    (directory/'map_manifest.json').write_text(json.dumps(manifest))
    (directory/'job.json').write_text(json.dumps(dict(projection=projection, backend='/backend')))
    (directory/'projection.yaml').write_text(yaml.safe_dump({'/**': {'ros__parameters':projection}}))
    summary = dict(points=100, nodes=1, GNSS_factors=3, join_factors=0, loop_factors=1,
                   fit_residual_m=dict(mean=.2, max=.4))
    (directory/'status.json').write_text(json.dumps(dict(state='complete', phase='完了', percent=100, summary=summary)))
    image = QtGui.QPixmap(100, 100)
    image.fill(QtGui.QColor('white'))
    image.save(str(directory/'overview.png'))
    return directory/'map_manifest.json'


def test_only_completed_intact_map_can_be_selected(qt_app, tmp_path):
    manifest = completed_map(tmp_path)
    tab = MapCreationTab()
    selected = []
    tab.map_selected.connect(lambda *args: selected.append(args))
    tab.load_result(tmp_path)
    assert tab.apply_button.isEnabled()
    assert '独立した精度評価ではありません' in tab.summary.text()
    tab.apply_result()
    assert selected == [(str(manifest), '/backend')]
    (tmp_path/'map.npy').write_bytes(b'damaged')
    tab.apply_result()
    assert len(selected) == 1
    assert not tab.apply_button.isEnabled()


@pytest.mark.parametrize('state', ['queued', 'running', 'failed', 'cancelled'])
def test_partial_results_cannot_be_applied(qt_app, tmp_path, state):
    completed_map(tmp_path)
    (tmp_path/'status.json').write_text(json.dumps(dict(state=state, phase=state, percent=20)))
    tab = MapCreationTab()
    tab.load_result(tmp_path)
    assert tab.result is None
    assert not tab.apply_button.isEnabled()


def test_map_setting_preserves_route_and_plan_and_rejects_wrong_projection(qt_app, tmp_path):
    manifest = completed_map(tmp_path)
    tab = LaunchSettingsTab()
    profile = 'icart_icp_route'
    state = tab.state_for(profile)
    state.override_inputs['route_directory'] = str(tmp_path)
    before = tab.plan.ordered_profile_ids.copy()
    changes = []
    tab.argument_changed.connect(lambda *args: changes.append(args))
    tab.configure_icp_map(str(manifest), '/backend')
    assert state.override_inputs['icp_map_manifest'] == str(manifest)
    assert state.override_inputs['route_directory'] == str(tmp_path)
    assert tab.plan.ordered_profile_ids == before
    assert changes == [(pid, key, value) for pid in (profile, 'icart_real_survey')
                       for key,value in [('icp_map_manifest', str(manifest)), ('icp_backend_python_path', '/backend')]]
    assert tab.state_for('icart_real_survey').override_inputs['icp_map_manifest'] == str(manifest)
    wrong = json.loads(manifest.read_text())
    wrong['projection']['origin_latitude'] += 1
    manifest.write_text(json.dumps(wrong))
    with pytest.raises(ValueError, match='projection mismatch'):
        tab.configure_icp_map(str(manifest), '/new-backend')
    assert state.override_inputs['icp_backend_python_path'] == '/backend'
