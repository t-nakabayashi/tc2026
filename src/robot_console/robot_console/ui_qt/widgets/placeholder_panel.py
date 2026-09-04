"""後続フェーズで内容を実装するタブ向けの仮表示パネル。"""

from __future__ import annotations

from typing import Optional

from PyQt5 import QtCore, QtWidgets

TITLE_FONT_POINT_SIZE = 20


class PlaceholderPanel(QtWidgets.QWidget):
    """タブ骨格の段階で内容が未実装であることを示すパネル。"""

    def __init__(
        self,
        title: str,
        description: str,
        parent: Optional[QtWidgets.QWidget] = None,
    ) -> None:
        super().__init__(parent)

        title_label = QtWidgets.QLabel(title, self)
        title_font = title_label.font()
        title_font.setPointSize(TITLE_FONT_POINT_SIZE)
        title_font.setBold(True)
        title_label.setFont(title_font)
        title_label.setAlignment(QtCore.Qt.AlignCenter)

        description_label = QtWidgets.QLabel(description, self)
        description_label.setAlignment(QtCore.Qt.AlignCenter)
        description_label.setWordWrap(True)

        layout = QtWidgets.QVBoxLayout(self)
        layout.setAlignment(QtCore.Qt.AlignCenter)
        layout.addWidget(title_label)
        layout.addWidget(description_label)
