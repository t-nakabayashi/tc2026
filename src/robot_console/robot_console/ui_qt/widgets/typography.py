"""実運用を想定した文字サイズの基準値。

robot_console はロボット搭載ノートPCの画面に表示され、走行中は数m離れた
位置から随行するエンジニアが状態を確認する運用を想定する。細かい文字は
判読できないため、最重要情報（運行フェーズ、イベントバナー、Node Health
集計）は特に大きく表示し、詳細診断情報（各カードの数値項目など）は
近くで確認する前提でこれより小さいサイズとする。
"""

from __future__ import annotations

# アプリ全体の既定フォントサイズ。1920x1080論理キャンバス上で改行・
# オーバーフローが起きない範囲の最大値として14ptを採用する
# （Qt既定は12pt）。
BASE_FONT_POINT_SIZE = 14

# ダッシュボード運行フェーズ: 最重要の一目情報。
PHASE_STATUS_FONT_POINT_SIZE = 40
PHASE_DETAIL_FONT_POINT_SIZE = 18

# Eventカード: 走行中に遠目でも認識できる大きな表示
# （screen_function_design.md 6.7節）。
EVENT_PRIMARY_FONT_POINT_SIZE = 32
EVENT_HISTORY_FONT_POINT_SIZE = 18

# Node Healthカード集計行: 異常有無を一目で把握するための強調表示。
NODE_HEALTH_SUMMARY_FONT_POINT_SIZE = 24
