"""ノード設定編集パネルで使う起動時override引数の入力形式ヒント。

既知の引数だけ、入力設計方針（screen_function_design.md 10章: 既知の選択肢は
ComboBox/RadioButton/CheckBox、numericはSpinBox/DoubleSpinBoxを使う）に沿った
専用ウィジェットを割り当てる。未掲載の引数（topic名やデバイスパスなど）は、
候補リストで表現できないため詳細入力欄（QLineEdit）へフォールバックする。
"""

from __future__ import annotations

from typing import Dict, List

BOOL_ARGUMENTS = {
    'start_coordinator',
    'enable_pylons',
    'enable_route_blocker',
    'start_gazebo_gui',
    'use_sim_time',
    'start_gui',
}

ENUM_ARGUMENTS: Dict[str, List[str]] = {
    'joy_input': ['joy_node', 'ps3_joy_sim'],
    'road_type': ['straight'],
}

NUMBER_ARGUMENTS = {
    'road_width': float,
    'pylon_seed': int,
}


def widget_kind(argument_name: str) -> str:
    """引数名から入力ウィジェット種別（'bool'/'enum'/'number'/'text'）を返す。"""

    if argument_name in BOOL_ARGUMENTS:
        return 'bool'
    if argument_name in ENUM_ARGUMENTS:
        return 'enum'
    if argument_name in NUMBER_ARGUMENTS:
        return 'number'
    return 'text'
