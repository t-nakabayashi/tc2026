"""robot_console のHTML遠隔観測UI（読み取り専用）。

`robot_console_gui_screen_function_design.md` 8章の方針に従い、PyQt5 UIと
同じ `ConsoleSnapshot` を共通の情報源として使う。操作系API（manual_start
送信、launch操作、param編集等）は一切提供しない。
"""
