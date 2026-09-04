"""Eventカード: 優先度付きイベントバナー表示。"""

from __future__ import annotations

from typing import List, Optional

from PyQt5 import QtWidgets

from robot_console.core.event_priority import PRIORITY_ORDER, sort_by_priority
from robot_console.core.snapshot_model import EventBanner

from .color_rules import severity_color
from .typography import EVENT_HISTORY_FONT_POINT_SIZE, EVENT_PRIMARY_FONT_POINT_SIZE

# 優先順位定義はHTML遠隔観測UIとも共有するため core/event_priority.py に置く。
__all__ = ['PRIORITY_ORDER', 'sort_by_priority', 'EventBannerCard', 'HISTORY_DISPLAY_COUNT']

HISTORY_DISPLAY_COUNT = 3


class EventBannerCard(QtWidgets.QGroupBox):
    """最上位イベントを大きく、続く数件を小型履歴として表示するカード。"""

    def __init__(self, parent: Optional[QtWidgets.QWidget] = None) -> None:
        super().__init__('Event', parent)

        self._primary_label = QtWidgets.QLabel('イベントなし')
        primary_font = self._primary_label.font()
        primary_font.setPointSize(EVENT_PRIMARY_FONT_POINT_SIZE)
        primary_font.setBold(True)
        self._primary_label.setFont(primary_font)
        self._primary_label.setWordWrap(True)

        history_font = self._primary_label.font()
        history_font.setPointSize(EVENT_HISTORY_FONT_POINT_SIZE)
        history_font.setBold(False)
        self._history_labels = [QtWidgets.QLabel('') for _ in range(HISTORY_DISPLAY_COUNT)]
        for label in self._history_labels:
            label.setWordWrap(True)
            label.setFont(history_font)

        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(self._primary_label)
        for label in self._history_labels:
            layout.addWidget(label)
        layout.addStretch(1)
        self.setLayout(layout)

    def update_snapshot(self, banners: List[EventBanner]) -> None:
        """優先順位付けしたEventBanner一覧を反映する。"""

        ordered = sort_by_priority(banners)
        if not ordered:
            self._primary_label.setText('イベントなし')
            self._primary_label.setStyleSheet('')
            for label in self._history_labels:
                label.setText('')
            return

        primary = ordered[0]
        self._primary_label.setText(primary.message or primary.event_type)
        self._primary_label.setStyleSheet(f'color: {severity_color(primary.severity)};')

        history = ordered[1:1 + HISTORY_DISPLAY_COUNT]
        for index, label in enumerate(self._history_labels):
            if index < len(history):
                banner = history[index]
                label.setText(banner.message or banner.event_type)
                label.setStyleSheet(f'color: {severity_color(banner.severity)};')
            else:
                label.setText('')
                label.setStyleSheet('')
