"""MainWindow骨格の自動テスト（`QT_QPA_PLATFORM=offscreen` 前提）。"""

import os

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

import pytest
from PyQt5 import QtWidgets

from robot_console.core.freshness import FreshnessLevel
from robot_console.core.launch_profile import LaunchProfileState
from robot_console.core.snapshot_model import ConsoleSnapshot, HealthSummaryView
from robot_console.utils import NodeLaunchStatus
from robot_console.ui_qt.main_window import (
    TAB_TITLE_CONSOLE_LOG,
    TAB_TITLE_DASHBOARD,
    TAB_TITLE_LAUNCH_SETTINGS,
    TAB_TITLE_LOCALIZATION_SENSOR,
    MainWindow,
)
from robot_console.ui_qt.widgets.scaled_canvas import LOGICAL_HEIGHT, LOGICAL_WIDTH, ScaledCanvas


@pytest.fixture(scope='module')
def qt_app():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    yield app


def test_main_window_has_four_tabs_in_expected_order(qt_app):
    window = MainWindow()
    tab_widget = window.tab_widget

    assert tab_widget.count() == 4
    assert tab_widget.tabText(0) == TAB_TITLE_DASHBOARD
    assert tab_widget.tabText(1) == TAB_TITLE_LOCALIZATION_SENSOR
    assert tab_widget.tabText(2) == TAB_TITLE_LAUNCH_SETTINGS
    assert tab_widget.tabText(3) == TAB_TITLE_CONSOLE_LOG
    assert tab_widget.currentWidget() is window.dashboard_tab


def test_main_window_has_no_status_bar(qt_app):
    window = MainWindow()
    # QMainWindow は statusBar() を一度も呼ばない限りステータスバーを生成しない。
    assert window.findChild(QtWidgets.QStatusBar) is None


def test_central_widget_is_scaled_canvas_with_logical_size(qt_app):
    window = MainWindow()
    canvas = window.centralWidget()

    assert isinstance(canvas, ScaledCanvas)
    assert canvas.content is window.tab_widget
    assert canvas.content.size().width() == LOGICAL_WIDTH
    assert canvas.content.size().height() == LOGICAL_HEIGHT


def test_scaled_canvas_keeps_aspect_ratio_scale_uniform_after_resize(qt_app):
    canvas = ScaledCanvas(QtWidgets.QWidget())
    canvas.resize(800, 600)
    canvas.show()

    transform = canvas.view_transform
    assert transform.m11() > 0.0
    # KeepAspectRatio のため水平・垂直の拡縮率は一致する。
    assert transform.m11() == pytest.approx(transform.m22())


def test_node_health_chip_click_navigates_to_console_log_tab(qt_app):
    window = MainWindow()
    window.dashboard_tab.update_snapshot(
        ConsoleSnapshot(
            health=[
                HealthSummaryView(
                    profile_id='obstacle_monitor', status='ERROR', health=FreshnessLevel.LOST
                )
            ]
        )
    )
    window.console_log_tab.update_snapshot(ConsoleSnapshot())

    chip = window.dashboard_tab.node_health_card._chip_layout.itemAt(0).widget()
    chip.click()

    assert window.tab_widget.currentWidget() is window.console_log_tab
    assert window.console_log_tab._selected_profile_id == 'obstacle_monitor'


def test_all_tabs_are_reachable_by_tab_selection(qt_app):
    """画面切り替えはタブ操作で行う（9章 画面間導線）。

    行き先を指定するだけの遷移ボタンは置かない方針のため、全タブがタブ選択で
    到達できることを保証する。
    """

    window = MainWindow()

    for tab in (
        window.localization_sensor_tab,
        window.launch_settings_tab,
        window.console_log_tab,
        window.dashboard_tab,
    ):
        window.tab_widget.setCurrentWidget(tab)
        assert window.tab_widget.currentWidget() is tab


def test_navigation_only_buttons_are_not_placed_on_screens(qt_app):
    """タブと重複する遷移ボタンを持たないことを確認する（9章）。

    ダッシュボードは走行中に常時見る画面であり、タブで代替できるボタンで
    表示面積を消費しない。
    """

    window = MainWindow()

    for tab in (
        window.dashboard_tab,
        window.localization_sensor_tab,
        window.console_log_tab,
    ):
        labels = [button.text() for button in tab.findChildren(QtWidgets.QPushButton)]
        assert 'ダッシュボードへ戻る' not in labels
        assert '自己位置・センサ情報を見る' not in labels


def test_launch_settings_plan_changes_propagate_to_dashboard_launch_control_card(qt_app):
    window = MainWindow()

    window.launch_settings_tab._environment_combo.setCurrentText('実機')
    window.launch_settings_tab._drive_mode_combo.setCurrentText('自律走行')
    window.launch_settings_tab._on_apply_preset_clicked()

    card = window.dashboard_tab.launch_control_card
    assert card._environment_combo.currentText() == '実機'
    assert card._drive_mode_combo.currentText() == '自律走行'
    assert card._ordered_profile_ids == window.launch_settings_tab.plan.ordered_profile_ids
    assert card._node_combo.count() == len(window.launch_settings_tab.plan.ordered_profile_ids)


def test_update_snapshot_fans_out_to_dashboard_and_localization_and_log_tabs(qt_app):
    window = MainWindow()
    snapshot = ConsoleSnapshot(
        health=[HealthSummaryView(profile_id='route_manager', status='RUNNING')]
    )

    window.update_snapshot(snapshot)

    assert window.dashboard_tab.node_health_card._chip_layout.count() == 1
    assert window.localization_sensor_tab._phase_label.text() == snapshot.operation_state.phase


def test_update_snapshot_reflects_launch_states_into_launch_settings_tab(qt_app):
    window = MainWindow()
    snapshot = ConsoleSnapshot(
        launch_profiles={
            'route_manager': LaunchProfileState(
                profile_id='route_manager',
                status=NodeLaunchStatus.RUNNING,
                process_id=4242,
            )
        }
    )

    window.update_snapshot(snapshot)

    state = window.launch_settings_tab.state_for('route_manager')
    assert state.status == NodeLaunchStatus.RUNNING
    assert state.process_id == 4242


class _FakeConsoleCore:
    """LaunchControlCard/ManualOpsCardのシグナル配線確認用の簡易ダブル。"""

    def __init__(self) -> None:
        self.launched: list = []
        self.stopped: list = []
        self.manual_start_calls: list = []
        self.sig_recog_calls: list = []
        self.road_blocked_calls: list = []
        self.obstacle_hint_override_calls: list = []
        self.obstacle_hint_stop_calls = 0
        self.frame_image_calls: list = []
        self.selected_param_calls: list = []
        self.use_alternate_launch_calls: list = []
        self.simulator_enabled_calls: list = []
        self.launch_override_calls: list = []
        self.business_mode_calls: list = []

    def request_launch(self, profile_id: str) -> None:
        self.launched.append(profile_id)

    def request_stop(self, profile_id: str) -> None:
        self.stopped.append(profile_id)

    def send_manual_start(self, value: bool) -> None:
        self.manual_start_calls.append(value)

    def send_sig_recog(self, value: int) -> None:
        self.sig_recog_calls.append(value)

    def send_road_blocked(self, value: bool) -> None:
        self.road_blocked_calls.append(value)

    def send_obstacle_hint_override(self, active, front_clearance_m, left_offset_m, right_offset_m) -> None:
        self.obstacle_hint_override_calls.append(
            (active, front_clearance_m, left_offset_m, right_offset_m)
        )

    def send_obstacle_hint_stop(self) -> None:
        self.obstacle_hint_stop_calls += 1

    def send_frame_image_request(self, path: str) -> None:
        self.frame_image_calls.append(path)

    def update_selected_param(self, profile_id: str, param_path) -> None:
        self.selected_param_calls.append((profile_id, param_path))

    def update_use_alternate_launch(self, profile_id: str, enabled: bool) -> None:
        self.use_alternate_launch_calls.append((profile_id, enabled))

    def update_simulator_enabled(self, profile_id: str, enabled: bool) -> None:
        self.simulator_enabled_calls.append((profile_id, enabled))

    def update_launch_override(self, profile_id: str, key: str, value: str) -> None:
        self.launch_override_calls.append((profile_id, key, value))

    def update_business_mode(self, environment: str, drive_mode: str) -> None:
        self.business_mode_calls.append((environment, drive_mode))


def test_launch_requested_signal_calls_core_request_launch(qt_app):
    core = _FakeConsoleCore()
    window = MainWindow(core=core)

    window.dashboard_tab.launch_control_card.launch_requested.emit('rtk_gps_um982')

    assert core.launched == ['rtk_gps_um982']


def test_stop_requested_signal_calls_core_request_stop(qt_app):
    core = _FakeConsoleCore()
    window = MainWindow(core=core)

    window.dashboard_tab.launch_control_card.stop_requested.emit('rtk_gps_um982')

    assert core.stopped == ['rtk_gps_um982']


def test_launch_all_requested_signal_calls_core_request_launch_for_each_id(qt_app):
    core = _FakeConsoleCore()
    window = MainWindow(core=core)

    window.dashboard_tab.launch_control_card.launch_all_requested.emit(['a', 'b'])

    assert core.launched == ['a', 'b']


def test_stop_all_requested_signal_calls_core_request_stop_for_each_id(qt_app):
    core = _FakeConsoleCore()
    window = MainWindow(core=core)

    window.dashboard_tab.launch_control_card.stop_all_requested.emit(['a', 'b'])

    assert core.stopped == ['a', 'b']


def test_launch_signals_are_not_connected_when_core_is_none(qt_app):
    window = MainWindow()

    # coreが無い場合でも例外を送出しないことのみ確認する。
    window.dashboard_tab.launch_control_card.launch_requested.emit('rtk_gps_um982')


def test_launch_settings_tab_param_changed_signal_calls_core_update_selected_param(qt_app):
    core = _FakeConsoleCore()
    window = MainWindow(core=core)

    window.launch_settings_tab.param_changed.emit('route_planner', 'params/tsukuba.yaml')

    assert core.selected_param_calls == [('route_planner', 'params/tsukuba.yaml')]


def test_launch_settings_tab_param_changed_signal_converts_empty_text_to_none(qt_app):
    core = _FakeConsoleCore()
    window = MainWindow(core=core)

    window.launch_settings_tab.param_changed.emit('route_planner', '')

    assert core.selected_param_calls == [('route_planner', None)]


def test_launch_settings_tab_alternate_and_simulator_toggles_call_core(qt_app):
    core = _FakeConsoleCore()
    window = MainWindow(core=core)

    window.launch_settings_tab.alternate_toggled.emit('robot_navigator', True)
    window.launch_settings_tab.simulator_toggled.emit('robot_navigator', True)

    assert core.use_alternate_launch_calls == [('robot_navigator', True)]
    assert core.simulator_enabled_calls == [('robot_navigator', True)]


def test_launch_settings_tab_argument_changed_signal_calls_core_update_launch_override(qt_app):
    core = _FakeConsoleCore()
    window = MainWindow(core=core)

    window.launch_settings_tab.argument_changed.emit('drive_mode_manager', 'joy_input', 'ps3_joy_sim')

    assert core.launch_override_calls == [('drive_mode_manager', 'joy_input', 'ps3_joy_sim')]


def test_manual_ops_card_signals_are_connected_to_core(qt_app):
    core = _FakeConsoleCore()
    window = MainWindow(core=core)
    manual_ops_card = window.dashboard_tab.manual_ops_card

    manual_ops_card.manual_start_requested.emit(True)
    manual_ops_card.sig_recog_requested.emit(2)
    manual_ops_card.road_blocked_requested.emit(True)
    manual_ops_card.obstacle_hint_override_requested.emit(True, 1.0, 0.1, 0.2)
    manual_ops_card.obstacle_hint_stop_requested.emit()
    manual_ops_card.frame_image_requested.emit('/tmp/frame.png')

    assert core.manual_start_calls == [True]
    assert core.sig_recog_calls == [2]
    assert core.road_blocked_calls == [True]
    assert core.obstacle_hint_override_calls == [(True, 1.0, 0.1, 0.2)]
    assert core.obstacle_hint_stop_calls == 1
    assert core.frame_image_calls == ['/tmp/frame.png']


def test_dashboard_apply_preset_updates_shared_launch_plan(qt_app):
    """ダッシュボードでの業務モード選択・プリセット適用が起動・設定タブの
    起動予定ノード一覧（唯一の実体）へ反映されることを確認する。"""

    window = MainWindow()
    card = window.dashboard_tab.launch_control_card
    card._environment_combo.setCurrentText('シミュレーション')
    card._drive_mode_combo.setCurrentText('手動走行')

    card.apply_preset_requested.emit(
        card._environment_combo.currentText(), card._drive_mode_combo.currentText()
    )

    assert window.launch_settings_tab.plan.ordered_profile_ids == [
        'obstacle_route_sim',
        'drive_mode_manager',
    ]
    # 適用結果が起動操作カードへも反映される（plan_changed経由）。
    assert card._ordered_profile_ids == ['obstacle_route_sim', 'drive_mode_manager']


def test_business_mode_selection_is_pushed_to_core(qt_app):
    """業務モードはROS topicから得られないため、選択値をConsoleCoreへ渡す必要がある。"""

    core = _FakeConsoleCore()
    window = MainWindow(core=core)

    assert core.business_mode_calls == [('実機', '手動走行')]

    window.launch_settings_tab._environment_combo.setCurrentText('シミュレーション')

    assert core.business_mode_calls[-1] == ('シミュレーション', '手動走行')
