"""ノード別ログ・統合ログの表示に用いる端末風ログビュー。

`screen_function_design.md` 10.3節・10.4節に従い、Ubuntu端末風の配色を
`QPlainTextEdit` で実現する。ANSIエスケープシーケンスの解釈は行わず、
`[INFO]`/`[WARN]`/`[ERROR]`/`[FATAL]`/`[DEBUG]` の文字列ベース色分けとする。
"""

from __future__ import annotations

from typing import Iterable, Optional

from PyQt5 import QtGui, QtWidgets

from robot_console.core.log_manager import detect_log_level

LOG_VIEW_BACKGROUND = '#1e1e1e'
LOG_VIEW_DEFAULT_TEXT = '#d4d4d4'
HIGHLIGHT_BACKGROUND = '#5c4a00'

LEVEL_COLORS = {
    'DEBUG': '#9e9e9e',
    'INFO': LOG_VIEW_DEFAULT_TEXT,
    'WARN': '#e6c200',
    'ERROR': '#f44336',
    'FATAL': '#f44336',
}


class LogView(QtWidgets.QPlainTextEdit):
    """Ubuntu端末風の配色でログ行を表示する読み取り専用ビュー。"""

    def __init__(self, parent: Optional[QtWidgets.QWidget] = None) -> None:
        super().__init__(parent)
        self.setReadOnly(True)
        self.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
        font = QtGui.QFont('monospace')
        font.setStyleHint(QtGui.QFont.Monospace)
        self.setFont(font)
        self.setStyleSheet(
            f'background-color: {LOG_VIEW_BACKGROUND}; color: {LOG_VIEW_DEFAULT_TEXT};'
        )

    def set_lines(
        self,
        lines: Iterable[str],
        *,
        highlight_term: str = '',
        tail_follow: bool = True,
    ) -> None:
        """ログ行をレベル別に色分けして再描画する。"""

        scrollbar = self.verticalScrollBar()
        previous_position = scrollbar.value()

        self.clear()
        cursor = self.textCursor()
        for line in lines:
            level = detect_log_level(line)
            char_format = QtGui.QTextCharFormat()
            char_format.setForeground(
                QtGui.QColor(LEVEL_COLORS.get(level or 'INFO', LOG_VIEW_DEFAULT_TEXT))
            )
            cursor.insertText(line.rstrip('\n') + '\n', char_format)

        if highlight_term:
            self._apply_highlight(highlight_term)

        if tail_follow:
            scrollbar.setValue(scrollbar.maximum())
        else:
            scrollbar.setValue(previous_position)

    def _apply_highlight(self, term: str) -> None:
        highlight_format = QtGui.QTextCharFormat()
        highlight_format.setBackground(QtGui.QColor(HIGHLIGHT_BACKGROUND))

        document = self.document()
        cursor = QtGui.QTextCursor(document)
        while True:
            cursor = document.find(term, cursor)
            if cursor.isNull():
                break
            cursor.mergeCharFormat(highlight_format)
