"""LaunchControlCard の単体テスト（`QT_QPA_PLATFORM=offscreen` 前提）。"""

import pytest
from PyQt5 import QtWidgets

from robot_console.core.launch_profile import LaunchProfile
from robot_console.ui_qt.widgets.launch_control_card import LaunchControlCard


@pytest.fixture(scope='module')
def qt_app():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    yield app


def _profile(profile_id: str, display_name: str) -> LaunchProfile:
    return LaunchProfile(
        profile_id=profile_id,
        category='route_stack',
        display_name=display_name,
        package=profile_id,
        launch_file=f'{profile_id}.launch.py',
    )


def test_update_plan_reflects_environment_and_drive_mode(qt_app):
    card = LaunchControlCard()

    card.update_plan(
        environment='実機',
        drive_mode='自律走行',
        ordered_profile_ids=[],
        profiles_by_id={},
    )

    assert card._environment_combo.currentText() == '実機'
    assert card._drive_mode_combo.currentText() == '自律走行'


def test_update_plan_populates_node_combo_with_display_names(qt_app):
    card = LaunchControlCard()

    card.update_plan(
        environment='実機',
        drive_mode='自律走行',
        ordered_profile_ids=['rtk_gps_um982', 'route_manager'],
        profiles_by_id={
            'rtk_gps_um982': _profile('rtk_gps_um982', 'RTK GPS UM982'),
            'route_manager': _profile('route_manager', 'Route Manager'),
        },
    )

    assert card._node_combo.count() == 2
    assert card._node_combo.itemText(0) == 'RTK GPS UM982'
    assert card._node_combo.itemData(0) == 'rtk_gps_um982'


def test_launch_all_requested_carries_ordered_profile_ids(qt_app):
    card = LaunchControlCard()
    card.update_plan(
        environment='実機',
        drive_mode='自律走行',
        ordered_profile_ids=['rtk_gps_um982', 'route_manager'],
        profiles_by_id={},
    )
    received = []
    card.launch_all_requested.connect(received.append)

    launch_all_button = [
        widget
        for widget in card.findChildren(QtWidgets.QPushButton)
        if widget.text() == '起動予定ノードを一斉起動'
    ][0]
    launch_all_button.click()

    assert received == [['rtk_gps_um982', 'route_manager']]


def test_stop_all_requested_carries_ordered_profile_ids(qt_app):
    card = LaunchControlCard()
    card.update_plan(
        environment='実機',
        drive_mode='自律走行',
        ordered_profile_ids=['rtk_gps_um982'],
        profiles_by_id={},
    )
    received = []
    card.stop_all_requested.connect(received.append)

    stop_all_button = [
        widget
        for widget in card.findChildren(QtWidgets.QPushButton)
        if widget.text() == '起動予定ノードを一斉停止'
    ][0]
    stop_all_button.click()

    assert received == [['rtk_gps_um982']]


def test_launch_requested_and_stop_requested_use_selected_node(qt_app):
    card = LaunchControlCard()
    card.update_plan(
        environment='実機',
        drive_mode='自律走行',
        ordered_profile_ids=['rtk_gps_um982', 'route_manager'],
        profiles_by_id={
            'rtk_gps_um982': _profile('rtk_gps_um982', 'RTK GPS UM982'),
            'route_manager': _profile('route_manager', 'Route Manager'),
        },
    )
    card._node_combo.setCurrentIndex(1)

    launched = []
    stopped = []
    card.launch_requested.connect(launched.append)
    card.stop_requested.connect(stopped.append)

    card._on_launch_selected_clicked()
    card._on_stop_selected_clicked()

    assert launched == ['route_manager']
    assert stopped == ['route_manager']


def test_update_plan_preserves_selection_when_still_present(qt_app):
    card = LaunchControlCard()
    profiles = {
        'rtk_gps_um982': _profile('rtk_gps_um982', 'RTK GPS UM982'),
        'route_manager': _profile('route_manager', 'Route Manager'),
    }
    card.update_plan(
        environment='実機',
        drive_mode='自律走行',
        ordered_profile_ids=['rtk_gps_um982', 'route_manager'],
        profiles_by_id=profiles,
    )
    card._node_combo.setCurrentIndex(1)

    # 順序が変わっても選択中のprofile_idは維持される。
    card.update_plan(
        environment='実機',
        drive_mode='自律走行',
        ordered_profile_ids=['route_manager', 'rtk_gps_um982'],
        profiles_by_id=profiles,
    )

    assert card._node_combo.currentData() == 'route_manager'


def test_apply_preset_requested_carries_selected_environment_and_drive_mode(qt_app):
    card = LaunchControlCard()
    card._environment_combo.setCurrentText('机上確認')
    card._drive_mode_combo.setCurrentText('自律走行')

    received = []
    card.apply_preset_requested.connect(lambda env, mode: received.append((env, mode)))

    apply_button = [
        widget
        for widget in card.findChildren(QtWidgets.QPushButton)
        if widget.text() == 'プリセット適用'
    ][0]
    apply_button.click()

    assert received == [('机上確認', '自律走行')]
