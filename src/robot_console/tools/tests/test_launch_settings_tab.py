"""LaunchSettingsTabの単体テスト（`QT_QPA_PLATFORM=offscreen` 前提）。"""

from pathlib import Path

import pytest
from PyQt5 import QtCore, QtWidgets

from robot_console.core.launch_profile import LaunchProfileStore
from robot_console.ui_qt.launch_settings_tab import LaunchSettingsTab

REPO_PROFILE_PATH = Path(__file__).resolve().parents[2] / 'config' / 'node_launch_profiles.yaml'


@pytest.fixture(scope='module')
def qt_app():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    yield app


def _make_tab() -> LaunchSettingsTab:
    return LaunchSettingsTab(LaunchProfileStore(REPO_PROFILE_PATH))


def test_tab_loads_all_profiles_grouped_by_category(qt_app):
    tab = _make_tab()

    assert len(tab._profiles) == 15
    assert tab._tree.topLevelItemCount() == 8  # profile.category の種類数
    assert set(tab._tree_items.keys()) == {p.profile_id for p in tab._profiles}


def test_checking_tree_item_adds_profile_to_plan(qt_app):
    tab = _make_tab()
    item = tab._tree_items['route_manager']

    item.setCheckState(0, QtCore.Qt.Checked)

    assert tab.plan.contains('route_manager') is True
    assert tab._plan_table.rowCount() == 1


def test_unchecking_tree_item_removes_profile_from_plan(qt_app):
    tab = _make_tab()
    item = tab._tree_items['route_manager']
    item.setCheckState(0, QtCore.Qt.Checked)

    item.setCheckState(0, QtCore.Qt.Unchecked)

    assert tab.plan.contains('route_manager') is False
    assert tab._plan_table.rowCount() == 0


def test_apply_preset_without_existing_plan_needs_no_confirmation(qt_app):
    tab = _make_tab()
    tab._environment_combo.setCurrentText('実機')
    tab._drive_mode_combo.setCurrentText('自律走行')

    tab._on_apply_preset_clicked()

    assert tab.plan.ordered_profile_ids == [
        'rtk_gps_um982',
        'ypspur_ros2',
        'drive_mode_manager',
        'route_planner',
        'route_manager',
        'geo_pose_converter',
        'route_follower',
        'obstacle_monitor',
        'robot_navigator',
        'road_blockage_detector',
        'traffic_signal_recognizer',
    ]
    assert tab._plan_table.rowCount() == 11
    assert tab._tree_items['rtk_gps_um982'].checkState(0) == QtCore.Qt.Checked


def test_apply_preset_with_existing_plan_requires_confirmation(qt_app, monkeypatch):
    monkeypatch.setattr(
        QtWidgets.QMessageBox,
        'question',
        staticmethod(lambda *args, **kwargs: QtWidgets.QMessageBox.No),
    )
    tab = _make_tab()
    tab._tree_items['route_manager'].setCheckState(0, QtCore.Qt.Checked)

    tab._environment_combo.setCurrentText('実機')
    tab._drive_mode_combo.setCurrentText('自律走行')
    tab._on_apply_preset_clicked()

    # Noを選んだため既存プランのまま変わらない
    assert tab.plan.ordered_profile_ids == ['route_manager']


def test_apply_preset_public_method_sets_combos_and_applies(qt_app):
    tab = _make_tab()

    tab.apply_preset('シミュレーション', '手動走行')

    assert tab._environment_combo.currentText() == 'シミュレーション'
    assert tab._drive_mode_combo.currentText() == '手動走行'
    assert tab.plan.ordered_profile_ids == ['obstacle_route_sim', 'drive_mode_manager']


def test_desktop_check_preset_sets_simulator_enabled_and_joy_input(qt_app):
    tab = _make_tab()
    tab._environment_combo.setCurrentText('机上確認')
    tab._drive_mode_combo.setCurrentText('自律走行')

    tab._on_apply_preset_clicked()

    assert tab.state_for('robot_navigator').simulator_enabled is True
    assert tab.state_for('drive_mode_manager').override_inputs.get('joy_input') == 'ps3_joy_sim'


def test_selecting_plan_row_updates_preview_and_config_panel(qt_app):
    tab = _make_tab()
    tab._environment_combo.setCurrentText('実機')
    tab._drive_mode_combo.setCurrentText('自律走行')
    tab._on_apply_preset_clicked()

    tab._select_plan_row_for('rtk_gps_um982')

    assert tab._selected_profile_id == 'rtk_gps_um982'
    preview = tab._preview_text.toPlainText()
    assert 'ros2 launch rtk_gps_um982 rtk_gps_um982.launch.py config:=config/default.yaml' in preview
    assert '起動順: 1' in preview
    assert '/rtk_gps/fix' in preview


def test_preview_shows_not_in_plan_when_profile_not_selected_for_launch(qt_app):
    tab = _make_tab()
    tab._environment_combo.setCurrentText('実機')
    tab._drive_mode_combo.setCurrentText('自律走行')
    tab._on_apply_preset_clicked()
    tab.plan.remove('road_blockage_detector')
    tab._selected_profile_id = 'road_blockage_detector'

    tab._update_preview()

    assert '起動予定に未追加' in tab._preview_text.toPlainText()


def test_editing_enum_argument_updates_state_and_plan_table(qt_app):
    tab = _make_tab()
    tab._environment_combo.setCurrentText('実機')
    tab._drive_mode_combo.setCurrentText('自律走行')
    tab._on_apply_preset_clicked()
    tab._select_plan_row_for('drive_mode_manager')

    joy_input_combo = tab._argument_widgets['joy_input']
    assert isinstance(joy_input_combo, QtWidgets.QComboBox)
    joy_input_combo.setCurrentText('ps3_joy_sim')

    assert tab.state_for('drive_mode_manager').override_inputs['joy_input'] == 'ps3_joy_sim'
    override_cell = tab._plan_table.item(2, 4)  # drive_mode_manager は3番目
    assert 'joy_input=ps3_joy_sim' in override_cell.text()


def test_alternate_launch_checkbox_updates_state(qt_app):
    tab = _make_tab()
    tab._tree_items['road_blockage_detector'].setCheckState(0, QtCore.Qt.Checked)
    tab._select_plan_row_for('road_blockage_detector')

    checkbox = None
    for row in range(tab._config_form.rowCount()):
        field_widget = tab._config_form.itemAt(row, QtWidgets.QFormLayout.FieldRole)
        if field_widget is not None and isinstance(field_widget.widget(), QtWidgets.QCheckBox):
            checkbox = field_widget.widget()
            break
    assert checkbox is not None

    checkbox.setChecked(True)

    assert tab.state_for('road_blockage_detector').use_alternate_launch is True
    assert 'road_blockage_perception_yolo.launch.py' in tab._preview_text.toPlainText()


def test_remove_from_plan_button_unchecks_tree_item(qt_app):
    tab = _make_tab()
    tab._tree_items['route_manager'].setCheckState(0, QtCore.Qt.Checked)

    tab._on_remove_from_plan_clicked('route_manager')

    assert tab.plan.contains('route_manager') is False
    assert tab._tree_items['route_manager'].checkState(0) == QtCore.Qt.Unchecked


def test_move_up_and_down_reorders_plan(qt_app):
    tab = _make_tab()
    for profile_id in ('route_planner', 'route_manager', 'route_follower'):
        tab._tree_items[profile_id].setCheckState(0, QtCore.Qt.Checked)

    tab._select_plan_row_for('route_follower')
    tab._move_selected_plan_entry(-1)

    assert tab.plan.ordered_profile_ids == ['route_planner', 'route_follower', 'route_manager']


def test_tab_has_no_launch_or_stop_signals(qt_app):
    """起動・設定タブは純粋な設定タブであり、起動・停止操作は持たない。"""

    tab = _make_tab()
    for removed_signal in ('launch_requested', 'stop_requested', 'launch_all_requested', 'stop_all_requested'):
        assert not hasattr(tab, removed_signal)


def test_plan_changed_emitted_on_checkbox_toggle(qt_app):
    tab = _make_tab()
    received = []
    tab.plan_changed.connect(lambda: received.append(True))

    tab._tree_items['route_manager'].setCheckState(0, QtCore.Qt.Checked)

    assert received == [True]


def test_plan_changed_emitted_on_apply_preset(qt_app):
    tab = _make_tab()
    tab._environment_combo.setCurrentText('シミュレーション')
    tab._drive_mode_combo.setCurrentText('手動走行')

    received = []
    tab.plan_changed.connect(lambda: received.append(True))
    tab._on_apply_preset_clicked()

    assert received == [True]
    assert tab.plan.ordered_profile_ids == ['obstacle_route_sim', 'drive_mode_manager']


def test_environment_and_drive_mode_properties_reflect_combo_selection(qt_app):
    tab = _make_tab()
    tab._environment_combo.setCurrentText('シミュレーション')
    tab._drive_mode_combo.setCurrentText('手動走行')

    assert tab.environment == 'シミュレーション'
    assert tab.drive_mode == '手動走行'


def test_profiles_by_id_exposes_loaded_profiles(qt_app):
    tab = _make_tab()

    assert tab.profiles_by_id['rtk_gps_um982'].display_name == 'RTK GPS UM982'


def test_update_launch_states_reflects_status_in_tree_and_table(qt_app):
    from robot_console.core.launch_profile import LaunchProfileState
    from robot_console.utils import NodeLaunchStatus

    tab = _make_tab()
    tab._tree_items['route_manager'].setCheckState(0, QtCore.Qt.Checked)

    tab.update_launch_states(
        {'route_manager': LaunchProfileState(profile_id='route_manager', status=NodeLaunchStatus.RUNNING)}
    )

    assert tab._tree_items['route_manager'].text(1) == 'RUNNING'
    assert tab._plan_table.item(0, 5).text() == 'RUNNING'


def test_business_mode_changed_emitted_on_combo_selection(qt_app):
    """業務モード選択は、プリセット適用を待たずConsoleCore側へ通知される。"""

    tab = _make_tab()
    received = []
    tab.business_mode_changed.connect(
        lambda environment, drive_mode: received.append((environment, drive_mode))
    )

    tab._environment_combo.setCurrentText('シミュレーション')
    tab._drive_mode_combo.setCurrentText('自律走行')

    assert received == [('シミュレーション', '手動走行'), ('シミュレーション', '自律走行')]
