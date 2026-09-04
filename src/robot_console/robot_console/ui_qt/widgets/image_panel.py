"""センサ・画像パネル1枚分の表示Widget（screen_function_design.md 7.5〜7.7節）。

`id/title/type/topic/image/status/updated_at` を持つパネルとして、画像本体が
無い場合はplaceholderを、鮮度が古い場合はSTALE/LOST表示を行う。
"""

from __future__ import annotations

from typing import Optional

from PIL import Image
from PyQt5 import QtCore, QtGui, QtWidgets

from robot_console.core.snapshot_model import ImageReference

from .color_rules import freshness_color

PLACEHOLDER_TEXT = 'No Image'
IMAGE_BACKGROUND = '#202020'
IMAGE_PLACEHOLDER_TEXT_COLOR = '#9e9e9e'


def pil_to_qpixmap(image: Image.Image) -> QtGui.QPixmap:
    """PIL.Image を QPixmap へ変換する。

    `PIL.ImageQt.ImageQt` は環境依存でQtバインディングの自動検出に失敗することが
    あるため、生バイト列からの手動変換を用いる。
    """

    rgba = image.convert('RGBA')
    data = rgba.tobytes('raw', 'RGBA')
    qimage = QtGui.QImage(data, rgba.width, rgba.height, QtGui.QImage.Format_RGBA8888).copy()
    return QtGui.QPixmap.fromImage(qimage)


class ImagePanel(QtWidgets.QGroupBox):
    """1件のセンサ・画像パネル（`ImageReference` + 画像本体）を表示する。

    カメラ画像やセンサビューアなど、元画像に意味のある固有アスペクト比が
    ある場合は `preserve_aspect_ratio=True`（既定）で歪みなく表示する。
    地図のようにパネル領域いっぱいに表示したい場合は `False` を指定する。
    """

    def __init__(
        self,
        parent: Optional[QtWidgets.QWidget] = None,
        *,
        preserve_aspect_ratio: bool = True,
    ) -> None:
        super().__init__(parent)
        self._preserve_aspect_ratio = preserve_aspect_ratio

        self._image_label = QtWidgets.QLabel(PLACEHOLDER_TEXT)
        self._image_label.setAlignment(QtCore.Qt.AlignCenter)
        self._image_label.setMinimumHeight(120)
        self._image_label.setStyleSheet(
            f'background-color: {IMAGE_BACKGROUND}; color: {IMAGE_PLACEHOLDER_TEXT_COLOR};'
        )
        self._status_label = QtWidgets.QLabel('-')

        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(self._image_label, 1)
        layout.addWidget(self._status_label)

        self._pixmap: Optional[QtGui.QPixmap] = None

    def update_panel(self, reference: ImageReference, image: Optional[Image.Image]) -> None:
        """`ImageReference` メタデータと画像本体（あれば）を反映する。"""

        self.setTitle(reference.title or reference.panel_id)

        if image is not None:
            self._pixmap = pil_to_qpixmap(image)
            self._render_pixmap()
        else:
            self._pixmap = None
            self._image_label.setPixmap(QtGui.QPixmap())
            self._image_label.setText(PLACEHOLDER_TEXT)

        updated_text = (
            reference.updated_at.strftime('%H:%M:%S') if reference.updated_at else '未受信'
        )
        self._status_label.setText(
            f'{reference.topic or "-"} / {reference.freshness.value} / {updated_text}'
        )
        self._status_label.setStyleSheet(f'color: {freshness_color(reference.freshness)};')

    def resizeEvent(self, event: QtGui.QResizeEvent) -> None:
        super().resizeEvent(event)
        self._render_pixmap()

    def _render_pixmap(self) -> None:
        if self._pixmap is None:
            return
        aspect_mode = (
            QtCore.Qt.KeepAspectRatio
            if self._preserve_aspect_ratio
            else QtCore.Qt.IgnoreAspectRatio
        )
        scaled = self._pixmap.scaled(
            self._image_label.size(),
            aspect_mode,
            QtCore.Qt.SmoothTransformation,
        )
        self._image_label.setPixmap(scaled)
