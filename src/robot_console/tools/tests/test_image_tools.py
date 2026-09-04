"""image_tools モジュールのテスト。"""

import sys
from pathlib import Path

import types

import pytest

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
try:
    import PIL  # noqa: F401
    import PIL.Image  # noqa: F401
    import PIL.ImageDraw  # noqa: F401
    import PIL.ImageFont  # noqa: F401
except ImportError:
    pass


if 'PIL' not in sys.modules:
    pil_root = types.ModuleType('PIL')
    sys.modules['PIL'] = pil_root
else:
    pil_root = sys.modules['PIL']

if 'PIL.Image' not in sys.modules:
    image_module = types.ModuleType('PIL.Image')

    class _StubImage:
        """テスト用の簡易画像スタブ。"""

        mode = 'RGB'

        def __init__(self) -> None:
            self.width = 1
            self.height = 1
            self.info = {}

        @property
        def size(self) -> tuple[int, int]:  # pragma: no cover - スタブ用
            return (self.width, self.height)

        def resize(self, size, _filter=None):  # pragma: no cover - スタブ用
            self.width, self.height = size
            return self

        def copy(self):  # pragma: no cover - スタブ用
            return self

        def paste(self, _other, box=None, mask=None):  # pragma: no cover - スタブ用
            return None

    def _new(_mode, size, color=None):
        image = _StubImage()
        image.width, image.height = size
        return image

    image_module.Image = _StubImage
    image_module.NEAREST = 0
    image_module.new = _new
    sys.modules['PIL.Image'] = image_module
    pil_root.Image = image_module
else:
    image_module = sys.modules['PIL.Image']

if hasattr(image_module, 'Image') and not hasattr(image_module.Image, 'size'):
    image_module.Image.size = property(  # type: ignore[attr-defined]
        lambda self: (getattr(self, 'width', 0), getattr(self, 'height', 0))
    )

if 'PIL.ImageDraw' not in sys.modules:
    image_draw_module = types.ModuleType('PIL.ImageDraw')
    sys.modules['PIL.ImageDraw'] = image_draw_module
    pil_root.ImageDraw = image_draw_module
else:
    image_draw_module = sys.modules['PIL.ImageDraw']

if not hasattr(image_draw_module, 'Draw'):
    class _StubDraw:
        def __init__(self, _image: object) -> None:  # pragma: no cover - スタブ用
            self._image = _image

        def textbbox(self, *_args, **_kwargs):  # pragma: no cover - スタブ用
            return (0, 0, 0, 0)

        def textsize(self, *_args, **_kwargs):  # pragma: no cover - スタブ用
            return (0, 0)

        def text(self, *_args, **_kwargs):  # pragma: no cover - スタブ用
            return None

    image_draw_module.Draw = lambda image: _StubDraw(image)  # pragma: no cover - スタブ用

if 'PIL.ImageFont' not in sys.modules:
    image_font_module = types.ModuleType('PIL.ImageFont')
    sys.modules['PIL.ImageFont'] = image_font_module
    pil_root.ImageFont = image_font_module
else:
    image_font_module = sys.modules['PIL.ImageFont']

if not hasattr(image_font_module, 'load_default'):
    image_font_module.load_default = lambda: object()  # pragma: no cover - スタブ用

from robot_console.utils import image_tools


def test_create_placeholder_image_fallback(monkeypatch: pytest.MonkeyPatch) -> None:
    """textbbox() が ValueError を送出した場合に textsize() へフォールバックする。"""

    class _DummyDraw:
        def __init__(self) -> None:
            self.textsize_called = False
            self.text_called = False

        def textbbox(self, *_args, **_kwargs):
            raise ValueError("Only supported for TrueType fonts")

        def textsize(self, *_args, **_kwargs):
            self.textsize_called = True
            return (20, 10)

        def text(self, *_args, **_kwargs) -> None:
            self.text_called = True

    dummy_draw = _DummyDraw()

    monkeypatch.setattr(image_draw_module, 'Draw', lambda _image: dummy_draw, raising=False)
    monkeypatch.setattr(image_font_module, 'load_default', lambda: object(), raising=False)

    image = image_tools.create_placeholder_image((100, 60), 'TEST')

    assert image is not None
    assert dummy_draw.textsize_called is True
    assert dummy_draw.text_called is True
