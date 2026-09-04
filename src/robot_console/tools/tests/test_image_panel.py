"""ImagePanel の単体テスト（`QT_QPA_PLATFORM=offscreen` 前提）。"""

from datetime import datetime, timezone

import pytest
from PIL import Image
from PyQt5 import QtWidgets

from robot_console.core.freshness import FreshnessLevel
from robot_console.core.snapshot_model import ImageReference
from robot_console.ui_qt.widgets.image_panel import PLACEHOLDER_TEXT, ImagePanel, pil_to_qpixmap


@pytest.fixture(scope='module')
def qt_app():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    yield app


def test_pil_to_qpixmap_preserves_size(qt_app):
    image = Image.new('RGB', (12, 8), color='green')
    pixmap = pil_to_qpixmap(image)
    assert pixmap.width() == 12
    assert pixmap.height() == 8


def test_update_panel_shows_placeholder_without_image(qt_app):
    panel = ImagePanel()
    reference = ImageReference(panel_id='lidar_view', title='LiDAR View')

    panel.update_panel(reference, None)

    assert panel.title() == 'LiDAR View'
    assert panel._image_label.text() == PLACEHOLDER_TEXT
    assert panel._image_label.pixmap() is None or panel._image_label.pixmap().isNull()
    assert '未受信' in panel._status_label.text()


def test_update_panel_renders_image_when_available(qt_app):
    panel = ImagePanel()
    reference = ImageReference(
        panel_id='sensor_viewer',
        title='Sensor Viewer',
        topic='/sensor_viewer',
        freshness=FreshnessLevel.OK,
        updated_at=datetime(2026, 1, 1, 9, 30, 0, tzinfo=timezone.utc),
    )

    panel.update_panel(reference, Image.new('RGB', (10, 10), color='blue'))

    assert panel._image_label.text() == ''
    assert not panel._image_label.pixmap().isNull()
    assert panel._status_label.text() == '/sensor_viewer / OK / 09:30:00'


def test_update_panel_falls_back_to_panel_id_when_title_missing(qt_app):
    panel = ImagePanel()
    reference = ImageReference(panel_id='front_camera', title='')

    panel.update_panel(reference, None)

    assert panel.title() == 'front_camera'


def test_preserve_aspect_ratio_false_fills_panel_without_letterbox(qt_app):
    # 地図パネル向け: 領域いっぱいに引き伸ばし、アスペクト比を保持しない。
    panel = ImagePanel(preserve_aspect_ratio=False)
    panel.resize(400, 100)
    reference = ImageReference(panel_id='route_map', title='Route Map')

    panel.update_panel(reference, Image.new('RGB', (10, 10), color='green'))

    scaled = panel._image_label.pixmap()
    assert scaled.width() == panel._image_label.width()
    assert scaled.height() == panel._image_label.height()


def test_preserve_aspect_ratio_true_keeps_source_aspect_ratio(qt_app):
    # 既定（カメラ・センサ画像向け）: アスペクト比を保ち、レターボックスを許容する。
    panel = ImagePanel()
    panel.resize(400, 100)
    reference = ImageReference(panel_id='sensor_viewer', title='Sensor Viewer')

    panel.update_panel(reference, Image.new('RGB', (10, 10), color='blue'))

    scaled = panel._image_label.pixmap()
    assert scaled.width() == scaled.height()
