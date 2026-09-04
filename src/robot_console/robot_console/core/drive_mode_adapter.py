"""走行制御（drive_mode_manager）のROSメッセージを表示用Viewへ変換する純粋関数群。

`DriveModeStateView` は `drive_mode_status` / `cmd_vel` / `cmd_vel/autonomous` の
3トピックがそれぞれ別のフィールドを埋めるため、本モジュールの `apply_*_msg()`
系関数は既存の `DriveModeStateView` を受け取り、対象フィールドだけを更新した
新しいViewを返す（`core/route_adapter.py` と同じ部分更新の方針）。

ROSメッセージ型を直接importしないため、`rclpy` が無い環境でも単体テストできる。
"""

from __future__ import annotations

import math
from dataclasses import replace
from typing import Any

from .snapshot_model import DriveModeStateView

# tc_route_msgs/DriveModeStatus の mode 定数（MODE_AUTONOMOUS=1 / MODE_MANUAL=2）。
_DRIVE_MODE_LABELS = {
    1: 'autonomous',
    2: 'manual',
}

# tc_route_msgs/DriveModeStatus の output_source 定数
# （SOURCE_ZERO=0 / SOURCE_AUTONOMOUS_CMD=1 / SOURCE_MANUAL_CMD=2）。
_OUTPUT_SOURCE_LABELS = {
    0: 'zero',
    1: 'autonomous_cmd',
    2: 'manual_cmd',
}


def drive_mode_label(value: int) -> str:
    """`DriveModeStatus.mode`（uint8）を表示用文字列へ変換する。"""

    return _DRIVE_MODE_LABELS.get(int(value), 'unknown')


def output_source_label(value: int) -> str:
    """`DriveModeStatus.output_source`（uint8）を表示用文字列へ変換する。"""

    return _OUTPUT_SOURCE_LABELS.get(int(value), 'unknown')


def apply_drive_mode_status_msg(view: DriveModeStateView, msg: Any) -> DriveModeStateView:
    """`tc_route_msgs/DriveModeStatus`（`drive_mode_status`）の内容を反映する。"""

    return replace(
        view,
        mode=drive_mode_label(msg.mode),
        output_source=output_source_label(msg.output_source),
        auto_resume_pending=bool(msg.auto_resume_pending),
    )


def apply_cmd_vel_msg(view: DriveModeStateView, msg: Any) -> DriveModeStateView:
    """`geometry_msgs/Twist`（`cmd_vel`）の内容を反映する。

    角速度はROSのrad/sから、表示用のdeg/sへ変換する（6.5節の表示単位）。
    """

    return replace(
        view,
        cmd_vel_linear_mps=float(msg.linear.x),
        cmd_vel_angular_dps=math.degrees(float(msg.angular.z)),
    )


def apply_cmd_vel_autonomous_msg(view: DriveModeStateView, msg: Any) -> DriveModeStateView:
    """`geometry_msgs/Twist`（`cmd_vel/autonomous`）の内容を反映する。

    mux後の `cmd_vel` との差分確認用であり、未受信の間は `None` のままとする
    （6.5節「自律指令の有無、mux後との差分」）。
    """

    return replace(
        view,
        cmd_vel_autonomous_linear_mps=float(msg.linear.x),
        cmd_vel_autonomous_angular_dps=math.degrees(float(msg.angular.z)),
    )
