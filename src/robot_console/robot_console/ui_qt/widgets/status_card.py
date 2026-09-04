"""ダッシュボード各カードで共有する汎用カードコンテナ。"""

from __future__ import annotations

from typing import Optional

from PyQt5 import QtWidgets


class StatusCard(QtWidgets.QGroupBox):
    """タイトル付きの汎用カード。本体は `QFormLayout` で `label: value` を並べる。"""

    def __init__(self, title: str, parent: Optional[QtWidgets.QWidget] = None) -> None:
        super().__init__(title, parent)
        self.form_layout = QtWidgets.QFormLayout()
        self.setLayout(self.form_layout)

    def add_value_row(self, label_text: str) -> QtWidgets.QLabel:
        """`label_text: 値` の行を追加し、更新対象のQLabelを返す。"""

        value_label = QtWidgets.QLabel('-')
        self.form_layout.addRow(f'{label_text}:', value_label)
        return value_label


def set_label_color(label: QtWidgets.QLabel, color: str) -> None:
    """QLabelの文字色をstylesheetで設定する。"""

    label.setStyleSheet(f'color: {color};')
