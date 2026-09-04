"""Node Healthカード: profile群の稼働状況サマリ表示。"""

from __future__ import annotations

from typing import List, Optional

from PyQt5 import QtCore, QtWidgets

from robot_console.core.freshness import FreshnessLevel
from robot_console.core.snapshot_model import HealthSummaryView

from .color_rules import COLOR_ERROR, COLOR_NOTICE, COLOR_OK, COLOR_UNKNOWN, COLOR_WARN
from .status_card import set_label_color
from .typography import NODE_HEALTH_SUMMARY_FONT_POINT_SIZE

CHIP_COLUMNS = 3

_STATUS_COLOR = {
    'RUNNING': COLOR_OK,
    'STARTING': COLOR_NOTICE,
    'STOPPED': COLOR_UNKNOWN,
    'ERROR': COLOR_ERROR,
}


def _needs_attention(item: HealthSummaryView) -> bool:
    return item.status == 'ERROR' or item.health in (FreshnessLevel.LOST, FreshnessLevel.STALE)


def sort_health_summaries(items: List[HealthSummaryView]) -> List[HealthSummaryView]:
    """WARN/ERROR/LOSTのprofileを先頭に並べ替える（6.8節）。"""

    return sorted(items, key=lambda item: (not _needs_attention(item), item.profile_id))


def _chip_color(item: HealthSummaryView) -> str:
    if item.required_but_not_selected:
        return COLOR_UNKNOWN
    if item.status == 'ERROR' or item.health == FreshnessLevel.LOST:
        return COLOR_ERROR
    if item.health == FreshnessLevel.STALE:
        return COLOR_WARN
    return _STATUS_COLOR.get(item.status, COLOR_UNKNOWN)


class NodeHealthCard(QtWidgets.QGroupBox):
    """profile群の稼働状況をチップ形式で一覧表示するカード。"""

    profile_selected = QtCore.pyqtSignal(str)

    def __init__(self, parent: Optional[QtWidgets.QWidget] = None) -> None:
        super().__init__('Node Health', parent)

        self._summary_label = QtWidgets.QLabel('RUNNING 0 / STOPPED 0 / WARN 0 / ERROR 0')
        summary_font = self._summary_label.font()
        summary_font.setPointSize(NODE_HEALTH_SUMMARY_FONT_POINT_SIZE)
        summary_font.setBold(True)
        self._summary_label.setFont(summary_font)

        self._chip_area = QtWidgets.QWidget()
        self._chip_layout = QtWidgets.QGridLayout(self._chip_area)
        self._chip_layout.setContentsMargins(0, 0, 0, 0)

        scroll_area = QtWidgets.QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setWidget(self._chip_area)
        scroll_area.setFrameShape(QtWidgets.QFrame.NoFrame)

        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(self._summary_label)
        layout.addWidget(scroll_area)
        self.setLayout(layout)

    def update_snapshot(self, health: List[HealthSummaryView]) -> None:
        """profile健全性一覧を反映する。"""

        running = sum(1 for item in health if item.status == 'RUNNING')
        stopped = sum(1 for item in health if item.status == 'STOPPED')
        error = sum(
            1 for item in health if item.status == 'ERROR' or item.health == FreshnessLevel.LOST
        )
        warn = sum(
            1
            for item in health
            if item.health == FreshnessLevel.STALE and item.status != 'ERROR'
        )
        self._summary_label.setText(
            f'RUNNING {running} / STOPPED {stopped} / WARN {warn} / ERROR {error}'
        )
        set_label_color(self._summary_label, COLOR_ERROR if error > 0 else COLOR_OK)

        while self._chip_layout.count():
            taken = self._chip_layout.takeAt(0)
            widget = taken.widget()
            if widget is not None:
                # setParent(None) で即座に親子関係を解除し描画から外す。
                # deleteLater() だけでは次のイベントループまで旧Widgetが
                # 画面に残り、新Widgetと重なって表示される。
                widget.setParent(None)
                widget.deleteLater()

        for index, item in enumerate(sort_health_summaries(health)):
            chip = self._build_chip(item)
            row, column = divmod(index, CHIP_COLUMNS)
            self._chip_layout.addWidget(chip, row, column)

    def _build_chip(self, item: HealthSummaryView) -> QtWidgets.QToolButton:
        chip = QtWidgets.QToolButton()
        chip.setText(item.profile_id)
        tooltip = f'{item.profile_id} / {item.status} / health={item.health.value}'
        if item.required_but_not_selected:
            tooltip += ' / required_but_not_selected'
        chip.setToolTip(tooltip)
        chip.setStyleSheet(f'background-color: {_chip_color(item)}; color: white; padding: 4px;')
        chip.clicked.connect(
            lambda _checked=False, profile_id=item.profile_id: self.profile_selected.emit(
                profile_id
            )
        )
        return chip
