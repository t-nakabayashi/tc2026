"""16:9論理キャンバスを保ったまま拡大縮小するコンテナWidget。"""

from __future__ import annotations

from typing import Optional

from PyQt5 import QtCore, QtGui, QtWidgets

LOGICAL_WIDTH = 1920
LOGICAL_HEIGHT = 1080


class ScaledCanvas(QtWidgets.QWidget):
    """`content` を16:9の論理キャンバスとして扱い拡縮するコンテナ。

    robot_console_gui_screen_function_design.md 2.1節の方針に従い、ウィンドウ
    サイズが変わっても `min(width / 16, height / 9)` 相当のスケールでアスペクト比
    を保ったまま拡縮し、余白は上下または左右のletterboxとする。タブ内に縦・横
    スクロールは置かない。
    """

    def __init__(
        self, content: QtWidgets.QWidget, parent: Optional[QtWidgets.QWidget] = None
    ) -> None:
        super().__init__(parent)
        content.setFixedSize(LOGICAL_WIDTH, LOGICAL_HEIGHT)

        self._scene = QtWidgets.QGraphicsScene(self)
        self._scene.setSceneRect(0, 0, LOGICAL_WIDTH, LOGICAL_HEIGHT)
        self._scene.addWidget(content)

        self._view = QtWidgets.QGraphicsView(self._scene, self)
        self._view.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self._view.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self._view.setFrameShape(QtWidgets.QFrame.NoFrame)
        self._view.setRenderHints(
            QtGui.QPainter.Antialiasing | QtGui.QPainter.SmoothPixmapTransform
        )

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self._view)

        self._content = content

    @property
    def content(self) -> QtWidgets.QWidget:
        """論理キャンバス上に配置しているcontent widgetを返す。"""

        return self._content

    @property
    def view_transform(self) -> QtGui.QTransform:
        """現在の拡縮に使われているQGraphicsViewの変換行列を返す。"""

        return self._view.transform()

    def showEvent(self, event: QtGui.QShowEvent) -> None:
        super().showEvent(event)
        self._fit_content()

    def resizeEvent(self, event: QtGui.QResizeEvent) -> None:
        super().resizeEvent(event)
        self._fit_content()

    def _fit_content(self) -> None:
        self._view.fitInView(self._scene.sceneRect(), QtCore.Qt.KeepAspectRatio)
