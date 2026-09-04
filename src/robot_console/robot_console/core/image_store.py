"""センサ・画像パネルの画像本体を保持するモジュール。

`ConsoleSnapshot.sensor_panels`（`ImageReference`）はメタデータのみを持ち、
画像本体は本モジュールが `image_id`（無ければ`panel_id`）をキーに保持する。
フレームワーク非依存の `PIL.Image.Image` として保持し、PyQt5向けの
`QImage`/`QPixmap` 変換はUI層（`ui_qt/widgets/image_panel.py`）が行う
（architecture_design.md 8.2節）。
"""

from __future__ import annotations

import threading
from typing import Dict, Optional

from PIL import Image


class ImageStore:
    """画像キー（`image_id` または `panel_id`）ごとに最新画像を保持する。"""

    def __init__(self) -> None:
        self._images: Dict[str, Image.Image] = {}
        self._lock = threading.Lock()

    def set(self, key: str, image: Image.Image) -> None:
        """指定キーの最新画像を更新する。"""

        with self._lock:
            self._images[key] = image

    def get(self, key: Optional[str]) -> Optional[Image.Image]:
        """指定キーの最新画像を返す。未受信または `key` が None の場合は None。"""

        if key is None:
            return None
        with self._lock:
            return self._images.get(key)

    def clear(self, key: str) -> None:
        """指定キーの画像を破棄する。"""

        with self._lock:
            self._images.pop(key, None)
