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
    PerceptionDecisionView,
    RouteView,
    RouteWaypointView,
    TargetView,
)
from robot_console.ui_qt.localization_sensor_tab import (
    CAMERA_PANEL_ID,
    CAMERA_ROW_STRETCH,
    DEFAULT_SENSOR_PANELS,
    LocalizationSensorTab,
    SENSOR_ROW_STRETCH,
)
from robot_console.ui_qt.widgets.image_panel import ImagePanel
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


def test_sensor_panels_from_snapshot_merge_into_defaults(qt_app):
    """受信済みパネルは既定枠を置き換えず、既定枠の上に重ねて反映される。"""

    tab = LocalizationSensorTab()
    snapshot = ConsoleSnapshot(
        sensor_panels=[
            ImageReference(panel_id='route_map', title='Route Map', topic='/active_route'),
            ImageReference(
                panel_id='sensor_viewer',
                title='Sensor Viewer',
                topic='/sensor_viewer',
                freshness=FreshnessLevel.OK,
            ),
        ]
    )

    tab.update_snapshot(snapshot)

    # route_mapはグリッドから除外され、専用パネルへ表示される。残りの既定枠は
    # 未受信のまま残り、未起動ノードの枠が画面から消えないことを確認する。
    assert tab._grid_layout.count() == len(DEFAULT_SENSOR_PANELS)
    titles = [panel.title() for panel in tab._panels.values()]
    assert 'Sensor Viewer' in titles
    assert 'Front Camera' in titles


def test_camera_is_stacked_above_sensor_viewer_in_one_column(qt_app):
    """カメラを上、Sensor Viewerを下に縦積みする（左右分割にしない）。"""

    tab = LocalizationSensorTab()

    positions = {}
    for index in range(tab._grid_layout.count()):
        row, column, _, _ = tab._grid_layout.getItemPosition(index)
        positions[index] = (row, column)
    # 1列に縦積みするため、列は全て0で行だけが増える。
    assert [positions[index] for index in sorted(positions)] == [(0, 0), (1, 0)]

    assert tab._grid_layout.itemAtPosition(0, 0).widget() is tab._panels[CAMERA_PANEL_ID]
    assert tab._grid_layout.itemAtPosition(1, 0).widget() is tab._panels['sensor_viewer']


def test_camera_row_is_taller_than_other_rows(qt_app):
    """カメラ行へ他の行より大きい高さ比を割り当てる。

    等分にするとカメラ行だけが判定チップの高さを負担し、画像領域がSensor Viewer
    より小さくなる。2:1でカメラ枠の縦横比が16:9映像とほぼ一致する。
    """

    tab = LocalizationSensorTab()

    assert tab._grid_layout.rowStretch(0) == CAMERA_ROW_STRETCH
    assert tab._grid_layout.rowStretch(1) == SENSOR_ROW_STRETCH
    assert CAMERA_ROW_STRETCH > SENSOR_ROW_STRETCH


def test_perception_chips_are_placed_inside_camera_card(qt_app):
    """判定チップはタブ全幅の別カードではなくカメラカードの中へ収める。

    別カードにすると見出しと枠の分だけ縦を消費し、カメラ画像が小さくなる。
    """

    tab = LocalizationSensorTab()

    camera_panel = tab._panels[CAMERA_PANEL_ID]
    assert tab._perception_group.parent() is camera_panel
    # カード内は 見出し行（パネル名 + topic/鮮度）→ 画像 → 判定チップ の順。
    panel_layout = camera_panel.layout()
    assert panel_layout.itemAt(1).widget() is camera_panel._image_label
    assert panel_layout.itemAt(2).widget() is tab._perception_group
    # 見出し付きの独立カードを持たない（縦幅の節約）。
    assert not isinstance(tab._perception_group, QtWidgets.QGroupBox)


def test_perception_chips_survive_grid_rebuild(qt_app):
    """パネル構成が変わってグリッドを作り直しても判定チップが消えないことを確認する。

    チップはカメラセルの子Widgetとして載せているため、セル破棄時に親子関係を
    解除しないと一緒に破棄され、以降の判定が表示されなくなる。
    """

    tab = LocalizationSensorTab()
    snapshot = ConsoleSnapshot(
        perception_decisions=[
            PerceptionDecisionView(
                source='traffic_signal',
                title='信号',
                decision_text='GO',
                freshness=FreshnessLevel.OK,
            )
        ],
        sensor_panels=[ImageReference(panel_id='rear_camera', title='Rear Camera')],
    )

    tab.update_snapshot(snapshot)

    assert tab._perception_group.parent() is tab._panels[CAMERA_PANEL_ID]
    assert tab._perception_labels['traffic_signal'].text() == 'GO'


def test_unknown_sensor_panel_is_appended_to_defaults(qt_app):
    """既定枠に無いpanel_idを受信した場合も、既定枠を残したまま追加表示する。"""

    tab = LocalizationSensorTab()
    snapshot = ConsoleSnapshot(
        sensor_panels=[ImageReference(panel_id='rear_camera', title='Rear Camera', topic='/rear')]
    )

    tab.update_snapshot(snapshot)

    assert tab._grid_layout.count() == len(DEFAULT_SENSOR_PANELS) + 1


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
            ImageReference(panel_id='road_blockage', freshness=FreshnessLevel.LOST),
        ]
    )

    tab.update_snapshot(snapshot)

    assert tab._sensor_freshness_label.text() == 'LOST'


def test_sensor_panel_widgets_are_reused_while_panel_set_is_unchanged(qt_app):
    """構成が同じ間はWidgetを作り直さない（グリッド再構築による再描画の抑制）。

    snapshotは毎秒ポーリングされるため、毎回作り直すとタブ全体のレイアウト再計算が
    走り、同じ画面のMapView（QWebEngineView）のちらつきにつながる。
    """

    tab = LocalizationSensorTab()
    snapshot = ConsoleSnapshot(
        sensor_panels=[
            ImageReference(
                panel_id='sensor_viewer', title='Sensor Viewer', freshness=FreshnessLevel.OK
            )
        ]
    )

    tab.update_snapshot(snapshot)
    widgets_before = [
        tab._grid_layout.itemAt(index).widget() for index in range(tab._grid_layout.count())
    ]

    tab.update_snapshot(snapshot)
    widgets_after = [
        tab._grid_layout.itemAt(index).widget() for index in range(tab._grid_layout.count())
    ]

    assert widgets_before == widgets_after


def test_sensor_panel_widgets_are_rebuilt_when_panel_set_changes(qt_app):
    tab = LocalizationSensorTab()
    tab.update_snapshot(ConsoleSnapshot())
    before = tab._grid_layout.count()

    tab.update_snapshot(
        ConsoleSnapshot(
            sensor_panels=[ImageReference(panel_id='rear_camera', title='Rear Camera')]
        )
    )

    assert tab._grid_layout.count() == before + 1
