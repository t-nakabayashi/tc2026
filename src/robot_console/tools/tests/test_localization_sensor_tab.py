"""LocalizationSensorTab の単体テスト（`QT_QPA_PLATFORM=offscreen` 前提）。"""

import pytest
from PyQt5 import QtWidgets

from robot_console.core.freshness import FreshnessLevel
from robot_console.core.snapshot_model import (
    ConsoleSnapshot,
    GpsStateView,
    ImageReference,
    LocalizationStateView,
    OperationStateView,
    RouteView,
    RouteWaypointView,
    TargetView,
)
from robot_console.ui_qt.localization_sensor_tab import (
    DEFAULT_SENSOR_PANELS,
    LocalizationSensorTab,
)
from robot_console.ui_qt.widgets.map_view import MapView


@pytest.fixture(scope='module')
def qt_app():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    yield app


def test_default_construction_shows_default_sensor_panels(qt_app):
    tab = LocalizationSensorTab()

    assert tab._route_overlay_group.title() == '地図 / Route Overlay'
    assert isinstance(tab._map_view, MapView)
    assert tab._grid_layout.count() == len(DEFAULT_SENSOR_PANELS)


def test_update_snapshot_reflects_summary(qt_app):
    tab = LocalizationSensorTab()
    snapshot = ConsoleSnapshot(
        operation_state=OperationStateView(
            phase='走行中', current_waypoint='A-10', next_waypoint='A-11', route_progress=0.4
        ),
        gps_state=GpsStateView(rtk_state='RTK_FIX', num_satellites=15, hdop=0.9),
        localization_state=LocalizationStateView(
            source='pose_enu', x_m=1.5, y_m=2.5, yaw_deg=90.0, freshness=FreshnessLevel.OK
        ),
        target_state=TargetView(distance_m=3.2, within_arrival_threshold=True),
    )

    tab.update_snapshot(snapshot)

    assert tab._phase_label.text() == '走行中'
    assert tab._waypoint_label.text() == 'A-10 -> A-11'
    assert tab._progress_label.text() == '40.0%'
    assert tab._gps_state_label.text() == 'RTK_FIX'
    assert tab._localization_freshness_label.text() == 'OK'


def test_sensor_panels_from_snapshot_replace_defaults(qt_app):
    tab = LocalizationSensorTab()
    snapshot = ConsoleSnapshot(
        sensor_panels=[
            ImageReference(panel_id='route_map', title='Route Map', topic='/active_route'),
            ImageReference(panel_id='sensor_viewer', title='Sensor Viewer', topic='/sensor_viewer'),
        ]
    )

    tab.update_snapshot(snapshot)

    # route_mapはグリッドから除外され、専用パネルへ表示される
    assert tab._grid_layout.count() == 1


def test_update_snapshot_pushes_localization_and_target_to_map_view(qt_app):
    tab = LocalizationSensorTab()
    calls = []
    tab._map_view.update_map = lambda localization, target: calls.append((localization, target))
    localization_state = LocalizationStateView(latitude=36.083, longitude=140.113)
    target_state = TargetView(latitude=36.0832, longitude=140.1132)
    snapshot = ConsoleSnapshot(localization_state=localization_state, target_state=target_state)

    tab.update_snapshot(snapshot)

    assert calls == [(localization_state, target_state)]


def test_update_snapshot_pushes_route_waypoints_to_map_view(qt_app):
    tab = LocalizationSensorTab()
    calls = []
    tab._map_view.update_route = lambda route: calls.append(route)
    route_state = RouteView(
        current_index=1,
        waypoints=[
            RouteWaypointView(index=0, latitude=36.083, longitude=140.113),
            RouteWaypointView(index=1, latitude=36.0831, longitude=140.1131),
        ],
    )
    snapshot = ConsoleSnapshot(route_state=route_state)

    tab.update_snapshot(snapshot)

    assert calls == [route_state]


def test_tab_has_no_navigation_only_button(qt_app):
    """タブで代替できる遷移ボタンを持たないことを確認する（9章 画面間導線）。"""

    tab = LocalizationSensorTab()

    labels = [button.text() for button in tab.findChildren(QtWidgets.QPushButton)]
    assert 'ダッシュボードへ戻る' not in labels


def test_sensor_freshness_summary_reports_worst_level(qt_app):
    tab = LocalizationSensorTab()
    snapshot = ConsoleSnapshot(
        sensor_panels=[
            ImageReference(panel_id='sensor_viewer', freshness=FreshnessLevel.OK),
            ImageReference(panel_id='lidar_view', freshness=FreshnessLevel.LOST),
        ]
    )

    tab.update_snapshot(snapshot)

    assert tab._sensor_freshness_label.text() == 'LOST'
