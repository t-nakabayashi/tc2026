"""自己位置・センサ情報タブ。

ローカルGUIで詳細確認が必要な場合に使う補助画面（robot_console_gui_screen_
function_design.md 7章）。走行中の通常監視はダッシュボード、遠隔または並行
確認はHTML UIを主とするため、本タブには手動介入・起動停止操作は置かない。

運行サマリでダッシュボードと同等の状況把握ができるため、GPS/自己位置の
詳細数値カード（ダッシュボードのGPS/Poseカードと重複する内容）は置かず、
空いた領域を地図表示の拡大に充てる。

地図表示は `robot_console_gui_screen_function_design.md` の「地図とセンサ」に従い、
QWebEngineViewで地図HTMLを埋め込み、Leaflet+OSMタイルで自己位置・目標waypoint・
route waypoint列を重畳表示する。
"""

from __future__ import annotations

from typing import Dict, List, Optional

from PyQt5 import QtWidgets

from robot_console.core.freshness import FreshnessLevel
from robot_console.core.image_store import ImageStore
from robot_console.core.snapshot_model import ConsoleSnapshot, ImageReference

from .widgets.color_rules import freshness_color
from .widgets.image_panel import ImagePanel
from .widgets.map_view import MapView
from .widgets.status_card import set_label_color

ROUTE_MAP_PANEL_ID = 'route_map'
CAMERA_PANEL_ID = 'front_camera'
# センサ・画像パネルは1列に縦積みする。カメラとSensor Viewerを左右に並べると
# 1枚あたりの幅が列幅の半分になり、横長画像では縦方向が余って表示が小さくなる。
GRID_COLUMNS = 1
# カメラ行とその他の行の高さ比。カメラ行は判定チップの高さも負担するため、等分に
# するとカメラの画像領域がSensor Viewerより小さくなる。3:2は、カメラを実用サイズ
# （実機の640x480で幅570相当）にしつつ、Sensor Viewerも読める大きさを残す配分。
# これ以上カメラ側へ寄せると16:9映像では枠が縦に余り、Viewerが縮むだけになる。
# 地図とセンサ列の横比を広げてもカメラ表示は大きくならない。高さが律速であり、
# 枠幅を増やしても左右の余白が増えるだけであるため、横比は変更しない。
CAMERA_ROW_STRETCH = 3
SENSOR_ROW_STRETCH = 2

# 判定チップを常設する認識種別。未受信でも枠を残し、認識ノードが未起動なのか
# 異常なのかを画面から区別できるようにする。
PERCEPTION_SOURCES: Dict[str, str] = {
    'traffic_signal': '信号',
    'road_blockage': '経路封鎖',
}

# screen_function_design.md 7.5節の初期候補パネル（route_mapを除く）。
# 受信有無によらず常設の枠として扱い、受信済みパネルで上書きする
# （`_update_sensor_panels`）。
# 並び順がそのまま上下の表示順になる。認識結果の判定チップを伴うカメラを上に置く。
DEFAULT_SENSOR_PANELS: List[ImageReference] = [
    # フロントカメラは 1 台のみで、信号認識・経路封鎖の検出枠は同じ画像へ重ねて
    # 描くため、カメラ系のパネルも 1 枚へ統合する。重畳は ConsoleCore が行う。
    ImageReference(
        panel_id=CAMERA_PANEL_ID, title='Front Camera', topic='/usb_cam/image_raw'
    ),
    ImageReference(panel_id='sensor_viewer', title='Sensor Viewer', topic='/sensor_viewer'),
]


class LocalizationSensorTab(QtWidgets.QWidget):
    """自己位置・センサ情報の詳細確認画面。"""

    def __init__(
        self,
        image_store: Optional[ImageStore] = None,
        parent: Optional[QtWidgets.QWidget] = None,
    ) -> None:
        super().__init__(parent)

        self._image_store = image_store or ImageStore()

        outer = QtWidgets.QVBoxLayout(self)
        outer.addWidget(self._build_summary_panel())
        # 判定チップはカメラパネルのセル内へ置くため、ここでは生成だけ行う。
        self._build_perception_panel()

        # GPS/自己位置の詳細カードは置かず（ダッシュボードのGPS/Poseカードと
        # 重複するため）、地図とセンサ・画像パネルへ領域を大きく割り当てる。
        content_row = QtWidgets.QHBoxLayout()
        content_row.addWidget(self._build_route_overlay_panel(), 3)
        content_row.addWidget(self._build_sensor_grid_panel(), 2)
        outer.addLayout(content_row, 1)

        self.update_snapshot(ConsoleSnapshot())

    # ---------- 運行サマリ（7.3節） ----------
    def _build_summary_panel(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox('運行サマリ')
        self._phase_label = QtWidgets.QLabel('-')
        self._follower_state_label = QtWidgets.QLabel('-')
        self._waypoint_label = QtWidgets.QLabel('-')
        self._progress_label = QtWidgets.QLabel('-')
        self._gps_state_label = QtWidgets.QLabel('-')
        self._localization_freshness_label = QtWidgets.QLabel('-')
        self._sensor_freshness_label = QtWidgets.QLabel('-')

        layout = QtWidgets.QHBoxLayout(group)
        for title, label in (
            ('運行フェーズ', self._phase_label),
            ('follower', self._follower_state_label),
            ('WP', self._waypoint_label),
            ('進捗', self._progress_label),
            ('GPS', self._gps_state_label),
            ('Localization', self._localization_freshness_label),
            ('Sensor', self._sensor_freshness_label),
        ):
            layout.addWidget(QtWidgets.QLabel(f'{title}:'))
            layout.addWidget(label)
        layout.addStretch(1)
        return group

    # ---------- 地図 / route overlay（7.2節左カラム、7.4節） ----------
    def _build_route_overlay_panel(self) -> QtWidgets.QGroupBox:
        self._route_overlay_group = QtWidgets.QGroupBox('地図 / Route Overlay')
        self._map_view = MapView()
        layout = QtWidgets.QVBoxLayout(self._route_overlay_group)
        fit_button = QtWidgets.QPushButton('ルート全体')
        fit_button.clicked.connect(self._map_view.fit_route)
        layout.addWidget(fit_button)
        self._route_status_label = QtWidgets.QLabel('ルート未受信')
        self._route_status_label.setWordWrap(True)
        layout.addWidget(self._route_status_label)
        layout.addWidget(self._map_view)
        return self._route_overlay_group

    # ---------- 画像認識の判定チップ（7.5節） ----------
    def _build_perception_panel(self) -> QtWidgets.QWidget:
        """画像認識の判定結果をチップとして表示する領域を構築する。

        検出枠はカメラ画像へ重畳済みのため、ここには判定結果だけを置く。走行中に
        運転者が読むのは GO/STOP・封鎖の有無という判定であり、画像内の小さな
        文字では数m離れた位置から判読できないため、画像と分けて表示する。

        配置はカメラパネルのカード内（画像の直下）とする。独立したカードにすると
        見出しと枠の分だけ縦を消費し、その分カメラ画像が小さくなる。種別名
        （信号 / 経路封鎖）をチップ自身が持つため、カードの見出しは不要である。
        """

        self._perception_group = QtWidgets.QWidget()
        self._perception_layout = QtWidgets.QHBoxLayout(self._perception_group)
        self._perception_layout.setContentsMargins(0, 0, 0, 0)
        # 末尾のstretchでチップを左寄せにする。stretchが無いと余白がチップ間へ
        # 分配され、カメラ幅に合わせたときに判定と種別名が離れて読みにくい。
        self._perception_layout.addStretch(1)
        self._perception_labels: Dict[str, QtWidgets.QLabel] = {}
        return self._perception_group

    def _update_perception_panel(self, snapshot: ConsoleSnapshot) -> None:
        """判定チップを更新する。"""

        decisions = {view.source: view for view in snapshot.perception_decisions}
        for source in PERCEPTION_SOURCES:
            view = decisions.get(source)
            label = self._perception_labels.get(source)
            if label is None:
                title = view.title if view is not None else PERCEPTION_SOURCES[source]
                # 末尾のstretchより前へ挿入し、チップの左寄せを保つ。
                index = self._perception_layout.count() - 1
                self._perception_layout.insertWidget(index, QtWidgets.QLabel(f'{title}:'))
                label = QtWidgets.QLabel('-')
                self._perception_layout.insertWidget(index + 1, label)
                self._perception_labels[source] = label

            if view is None:
                label.setText('未受信')
                set_label_color(label, freshness_color(FreshnessLevel.UNKNOWN))
                continue

            text = view.decision_text or '-'
            if view.status_note:
                text = f'{text}（{view.status_note}）'
            if view.detection_count:
                text = f'{text} / 検出{view.detection_count}'
            label.setText(text)
            set_label_color(label, freshness_color(view.freshness))

    # ---------- センサ・画像パネル（7.5節） ----------
    def _build_sensor_grid_panel(self) -> QtWidgets.QGroupBox:
        self._grid_group = QtWidgets.QGroupBox('センサ・画像パネル')
        self._grid_layout = QtWidgets.QGridLayout()
        self._grid_group.setLayout(self._grid_layout)
        # 現在グリッドに並べているパネルの構成。変化したときだけ作り直す
        # （`_rebuild_grid`）。
        self._panel_ids: List[str] = []
        self._panels: Dict[str, ImagePanel] = {}
        return self._grid_group

    def _apply_row_stretch(self, panel_ids: List[str]) -> None:
        """カメラ行へ他の行より大きい高さ比を割り当てる。

        Args:
            panel_ids (List[str]): グリッドへ並べた `panel_id` の一覧（行順）.
        """

        for row in range(self._grid_layout.rowCount()):
            self._grid_layout.setRowStretch(row, SENSOR_ROW_STRETCH)
        if CAMERA_PANEL_ID in panel_ids:
            # 1列に縦積みするため、行番号は並び順と一致する。
            self._grid_layout.setRowStretch(
                panel_ids.index(CAMERA_PANEL_ID), CAMERA_ROW_STRETCH
            )

    # ---------- Snapshot反映 ----------
    def update_snapshot(self, snapshot: ConsoleSnapshot) -> None:
        """`ConsoleSnapshot` の内容を反映する。"""

        self._update_summary(snapshot)
        self._update_perception_panel(snapshot)
        self._update_route_overlay(snapshot)
        self._update_sensor_panels(snapshot)

    def _update_summary(self, snapshot: ConsoleSnapshot) -> None:
        operation = snapshot.operation_state
        self._phase_label.setText(operation.phase)
        self._follower_state_label.setText(snapshot.follower_state.state)
        self._waypoint_label.setText(
            f'{operation.current_waypoint or "-"} -> {operation.next_waypoint or "-"}'
        )
        self._progress_label.setText(f'{operation.route_progress * 100.0:.1f}%')
        self._gps_state_label.setText(snapshot.gps_state.rtk_state)
        self._localization_freshness_label.setText(snapshot.localization_state.freshness.value)
        set_label_color(
            self._localization_freshness_label,
            freshness_color(snapshot.localization_state.freshness),
        )

        sensor_summary = self._summarize_freshness(
            [panel.freshness for panel in snapshot.sensor_panels]
        )
        self._sensor_freshness_label.setText(sensor_summary)

    @staticmethod
    def _summarize_freshness(levels: List[FreshnessLevel]) -> str:
        if not levels:
            return '-'
        if any(level == FreshnessLevel.LOST for level in levels):
            return 'LOST'
        if any(level == FreshnessLevel.STALE for level in levels):
            return 'STALE'
        if all(level == FreshnessLevel.OK for level in levels):
            return 'OK'
        return 'UNKNOWN'

    def _update_route_overlay(self, snapshot: ConsoleSnapshot) -> None:
        self._map_view.update_route(snapshot.route_state)
        self._map_view.update_map(snapshot.localization_state, snapshot.target_state)
        waypoints = snapshot.route_state.waypoints
        count = sum(p.latitude is not None and p.longitude is not None for p in waypoints)
        status = (f'配信ルート：{count}/{len(waypoints)}点（緯度経度あり）'
                  if waypoints else 'ルート未受信：起動後の配信を待っています')
        self._route_status_label.setText(
            status + '\n緑：未走行／灰：走行済み／水色：現在位置／黄：目標点'
        )

    def _update_sensor_panels(self, snapshot: ConsoleSnapshot) -> None:
        # 既定パネルを常設の枠として置き、受信済みパネルで上書きする。受信分だけに
        # 置き換えると、未起動・停止中のノードに対応する枠が「未受信」表示ではなく
        # グリッドから消え、異常なのか元々起動していないのかを区別できなくなる。
        panels_by_id: Dict[str, ImageReference] = {
            panel.panel_id: panel for panel in DEFAULT_SENSOR_PANELS
        }
        for panel in snapshot.sensor_panels:
            panels_by_id[panel.panel_id] = panel
        # route_mapは地図パネル（MapView）専用のため、センサ・画像パネルの
        # グリッドには含めない（HTML UI側のapp.js::renderSensorGridと同様の扱い）。
        panels_by_id.pop(ROUTE_MAP_PANEL_ID, None)

        self._rebuild_grid(list(panels_by_id.values()))

    def _rebuild_grid(self, panel_references: List[ImageReference]) -> None:
        """パネル構成を反映する。

        本メソッドはsnapshotポーリングのたびに呼ばれるため、毎回Widgetを作り直すと
        1秒ごとにグリッド全体のレイアウト再計算と再描画が走る。`ScaledCanvas` は
        タブ全体を `QGraphicsScene` 上に載せており、この再描画が同じ画面にある
        `MapView`（`QWebEngineView`）のちらつきにつながる。パネルの構成（順序と
        `panel_id`）が変わったときだけ作り直し、以降は既存Widgetの内容だけ更新する。
        """

        panel_ids = [reference.panel_id for reference in panel_references]
        if panel_ids != self._panel_ids:
            # 判定チップはカメラパネルの子Widgetとして載せているため、パネルを
            # 破棄する前に親子関係を解除する。解除しないと一緒に破棄される。
            self._perception_group.setParent(None)
            while self._grid_layout.count():
                item = self._grid_layout.takeAt(0)
                widget = item.widget()
                if widget is not None:
                    # setParent(None) で即座に親子関係を解除し描画から外す。
                    # deleteLater() だけでは次のイベントループまで旧Widgetが
                    # 画面に残り、新Widgetと重なって表示される。
                    widget.setParent(None)
                    widget.deleteLater()

            self._panels = {}
            for index, reference in enumerate(panel_references):
                panel = ImagePanel()
                row, column = divmod(index, GRID_COLUMNS)
                if reference.panel_id == CAMERA_PANEL_ID:
                    panel.add_footer_widget(self._perception_group)
                self._grid_layout.addWidget(panel, row, column)
                self._panels[reference.panel_id] = panel
            if CAMERA_PANEL_ID not in self._panels:
                # カメラパネルが無い構成でも、判定チップの枠は画面へ残す
                # （認識ノードが未起動なのか異常なのかを区別できるようにする）。
                row, column = divmod(len(panel_references), GRID_COLUMNS)
                self._grid_layout.addWidget(self._perception_group, row, column)
            self._apply_row_stretch(panel_ids)
            self._panel_ids = panel_ids

        for reference in panel_references:
            image = self._image_store.get(reference.image_id or reference.panel_id)
            self._panels[reference.panel_id].update_panel(reference, image)
