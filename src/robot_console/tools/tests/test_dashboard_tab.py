"""DashboardTabおよび構成カードの単体テスト（`QT_QPA_PLATFORM=offscreen` 前提）。"""

from datetime import datetime, timezone

import pytest
from PyQt5 import QtWidgets

from robot_console.core.freshness import FreshnessLevel
from robot_console.core.snapshot_model import (
    ConsoleSnapshot,
    DriveModeStateView,
    EventBanner,
    FollowerView,
    GpsStateView,
    HealthSummaryView,
    LocalizationStateView,
    ManualControlsView,
    OperationStateView,
    RouteView,
    TargetView,
)
from robot_console.ui_qt.dashboard_tab import DashboardTab
from robot_console.ui_qt.widgets.event_banner_card import EventBannerCard, sort_by_priority
from robot_console.ui_qt.widgets.manual_ops_card import SIG_RECOG_GO, ManualOpsCard
from robot_console.ui_qt.widgets.node_health_card import NodeHealthCard, sort_health_summaries


@pytest.fixture(scope='module')
def qt_app():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    yield app


def _sample_snapshot() -> ConsoleSnapshot:
    return ConsoleSnapshot(
        operation_state=OperationStateView(
            environment='実機',
            drive_mode='自律',
            phase='走行中',
            route_progress=0.25,
            current_waypoint='A-10',
            next_waypoint='A-11',
            manual_start=True,
        ),
        gps_state=GpsStateView(
            rtk_state='RTK_FIX',
            num_satellites=18,
            hdop=0.8,
            correction_age_s=1.2,
            rtcm_bytes_received=125034,
            heading_deg=123.4,
            heading_stddev_deg=0.8,
            fix_freshness=FreshnessLevel.OK,
        ),
        localization_state=LocalizationStateView(source='pose_enu', freshness=FreshnessLevel.OK),
        route_state=RouteView(
            state='running',
            route_version=3,
            last_decision='replan',
            last_replan_reason='blocked_segment',
            current_index=10,
            total_waypoints=80,
            progress_ratio=0.125,
        ),
        follower_state=FollowerView(
            state='RUNNING',
            active_waypoint_index=10,
            active_waypoint_label='A-10',
            stagnation_reason='',
        ),
        target_state=TargetView(distance_m=1.5, within_arrival_threshold=False),
        drive_mode_state=DriveModeStateView(
            mode='autonomous',
            output_source='navigator',
            cmd_vel_linear_mps=0.5,
            cmd_vel_angular_dps=3.0,
            cmd_vel_freshness=FreshnessLevel.OK,
            odom_topic='/ypspur_ros/odom',
            odom_freshness=FreshnessLevel.OK,
        ),
        event_banners=[
            EventBanner(event_type='route_update', message='ROUTE UPDATED v3', severity='info'),
            EventBanner(
                event_type='road_blocked', message='ROAD BLOCKED external', severity='error'
            ),
        ],
        manual_controls=ManualControlsView(
            manual_start_value=True,
            manual_start_last_sent_at=datetime(2026, 1, 1, tzinfo=timezone.utc),
        ),
        health=[
            HealthSummaryView(
                profile_id='route_manager', status='RUNNING', health=FreshnessLevel.OK
            ),
            HealthSummaryView(
                profile_id='obstacle_monitor', status='ERROR', health=FreshnessLevel.LOST
            ),
        ],
    )


def test_dashboard_tab_phase_header_reflects_operation_state(qt_app):
    tab = DashboardTab()
    tab.update_snapshot(_sample_snapshot())

    assert tab._phase_label.text() == '走行中'
    assert tab._phase_label.styleSheet() == 'color: #2e7d32;'
    assert tab._progress_label.text() == '25.0%'
    assert tab._waypoint_label.text() == 'A-10 -> A-11'
    assert tab._manual_start_label.text() == 'True'
    # road_blocked(優先度上位)がroute_update(優先度下位)より優先表示される
    assert 'ROAD BLOCKED' in tab._top_event_label.text()


def test_dashboard_tab_route_follower_card_reflects_snapshot(qt_app):
    tab = DashboardTab()
    tab.update_snapshot(_sample_snapshot())

    assert tab._route_state_label.text() == 'running'
    assert tab._route_version_label.text() == '3'
    assert tab._follower_waypoint_label.text() == 'A-10 (#10)'
    assert tab._route_progress_label.text() == '10/80 (12.5%)'
    assert '未到達' in tab._target_distance_label.text()


def test_dashboard_tab_gps_card_reflects_snapshot(qt_app):
    tab = DashboardTab()
    tab.update_snapshot(_sample_snapshot())

    assert tab._gps_rtk_label.text() == 'RTK_FIX'
    assert tab._gps_satellites_label.text() == '18 sat'
    assert tab._localization_source_label.text() == 'pose_enu'


def test_event_banner_card_orders_by_priority(qt_app):
    banners = [
        EventBanner(event_type='route_update', message='route update'),
        EventBanner(event_type='road_blocked', message='road blocked'),
        EventBanner(event_type='signal_go', message='signal go'),
    ]
    ordered = sort_by_priority(banners)
    assert [banner.event_type for banner in ordered] == ['road_blocked', 'signal_go', 'route_update']

    card = EventBannerCard()
    card.update_snapshot(banners)
    assert card._primary_label.text() == 'road blocked'


def test_event_banner_card_shows_placeholder_when_empty(qt_app):
    card = EventBannerCard()
    card.update_snapshot([])
    assert card._primary_label.text() == 'イベントなし'


def test_node_health_card_summary_counts_and_sort_order(qt_app):
    items = [
        HealthSummaryView(profile_id='route_manager', status='RUNNING', health=FreshnessLevel.OK),
        HealthSummaryView(
            profile_id='obstacle_monitor', status='ERROR', health=FreshnessLevel.LOST
        ),
        HealthSummaryView(
            profile_id='route_follower', status='STOPPED', health=FreshnessLevel.UNKNOWN
        ),
    ]
    ordered = sort_health_summaries(items)
    assert ordered[0].profile_id == 'obstacle_monitor'

    card = NodeHealthCard()
    card.update_snapshot(items)
    assert card._summary_label.text() == 'RUNNING 1 / STOPPED 1 / WARN 0 / ERROR 1'


def test_node_health_card_summary_label_colors_red_when_error_present(qt_app):
    card = NodeHealthCard()
    card.update_snapshot(
        [HealthSummaryView(profile_id='obstacle_monitor', status='ERROR', health=FreshnessLevel.LOST)]
    )
    assert card._summary_label.styleSheet() == 'color: #c62828;'


def test_node_health_card_summary_label_colors_green_when_no_error(qt_app):
    card = NodeHealthCard()
    card.update_snapshot(
        [HealthSummaryView(profile_id='route_manager', status='RUNNING', health=FreshnessLevel.OK)]
    )
    assert card._summary_label.styleSheet() == 'color: #2e7d32;'


def test_node_health_card_emits_profile_selected_on_chip_click(qt_app):
    card = NodeHealthCard()
    card.update_snapshot(
        [HealthSummaryView(profile_id='route_manager', status='RUNNING', health=FreshnessLevel.OK)]
    )
    received = []
    card.profile_selected.connect(received.append)

    chip = card._chip_layout.itemAt(0).widget()
    chip.click()

    assert received == ['route_manager']


def test_manual_ops_card_manual_start_signal_needs_no_confirmation(qt_app):
    card = ManualOpsCard()
    received = []
    card.manual_start_requested.connect(received.append)

    manual_start_tab = card._tabs.widget(0)
    send_button = manual_start_tab.findChildren(QtWidgets.QPushButton)[0]
    send_button.click()

    assert received == [True]


def test_manual_ops_card_sig_recog_signal_defaults_to_go(qt_app):
    card = ManualOpsCard()
    received = []
    card.sig_recog_requested.connect(received.append)

    card._on_sig_recog_send_clicked()

    assert received == [SIG_RECOG_GO]


def test_manual_ops_card_road_blocked_false_needs_no_confirmation(qt_app):
    card = ManualOpsCard()
    received = []
    card.road_blocked_requested.connect(received.append)

    # ラジオボタンの既定選択(False)のまま送信する。
    card._on_road_blocked_send_clicked()

    assert received == [False]


def test_manual_ops_card_road_blocked_true_requires_confirmation(qt_app, monkeypatch):
    monkeypatch.setattr(
        QtWidgets.QMessageBox,
        'question',
        staticmethod(lambda *args, **kwargs: QtWidgets.QMessageBox.No),
    )
    card = ManualOpsCard()
    received = []
    card.road_blocked_requested.connect(received.append)

    card._road_blocked_true_radio.setChecked(True)
    card._on_road_blocked_send_clicked()

    assert received == []  # 確認ダイアログでNoを選んだため送信されない


def test_manual_ops_card_obstacle_hint_start_emits_after_confirmation(qt_app, monkeypatch):
    monkeypatch.setattr(
        QtWidgets.QMessageBox,
        'question',
        staticmethod(lambda *args, **kwargs: QtWidgets.QMessageBox.Yes),
    )
    card = ManualOpsCard()
    received = []
    card.obstacle_hint_override_requested.connect(lambda *args: received.append(args))

    card._obstacle_clearance_spin.setValue(2.5)
    card._obstacle_left_spin.setValue(0.3)
    card._obstacle_right_spin.setValue(-0.2)
    card._obstacle_front_blocked_check.setChecked(True)
    card._on_obstacle_hint_start_clicked()

    assert received == [(True, 2.5, 0.3, -0.2)]


def test_manual_ops_card_update_snapshot_reflects_manual_controls(qt_app):
    card = ManualOpsCard()
    card.update_snapshot(
        ManualControlsView(
            manual_start_value=True,
            manual_start_last_sent_at=datetime(2026, 1, 1, 12, 0, 0, tzinfo=timezone.utc),
            road_blocked_value=True,
            road_blocked_source='external',
        )
    )

    assert card._manual_start_value_label.text() == 'True'
    assert card._manual_start_time_label.text() == '12:00:00'
    assert card._road_blocked_source_label.text() == 'external'
