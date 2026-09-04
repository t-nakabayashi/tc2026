"""ダッシュボード各カードで共有する色ルール。

robot_console_gui_screen_function_design.md 6.6節（GPS/Poseカード色ルール）・
6.8節（Node Health状態色）・6.7節（Eventカード優先度）に基づく。
"""

from __future__ import annotations

from robot_console.core.freshness import FreshnessLevel
from robot_console.utils import NodeLaunchStatus

COLOR_OK = '#2e7d32'  # 緑
COLOR_NOTICE = '#1565c0'  # 青
COLOR_WARN = '#f9a825'  # 黄
COLOR_ERROR = '#c62828'  # 赤
COLOR_UNKNOWN = '#757575'  # 灰


def freshness_color(level: FreshnessLevel) -> str:
    """FreshnessLevel から表示色を返す。"""

    if level == FreshnessLevel.OK:
        return COLOR_OK
    if level == FreshnessLevel.STALE:
        return COLOR_WARN
    if level == FreshnessLevel.LOST:
        return COLOR_ERROR
    return COLOR_UNKNOWN


def rtk_state_color(rtk_state: str, fix_freshness: FreshnessLevel) -> str:
    """RTK状態と鮮度から6.6節の色ルールに従い表示色を返す。"""

    if fix_freshness == FreshnessLevel.LOST:
        return COLOR_ERROR
    state = rtk_state.upper()
    if state == 'RTK_FIX':
        return COLOR_OK
    if state == 'RTK_FLOAT':
        return COLOR_WARN
    if state in ('DGPS', 'STANDALONE'):
        return COLOR_NOTICE
    return COLOR_UNKNOWN


def severity_color(severity: str) -> str:
    """EventBanner.severity から表示色を返す。"""

    mapping = {
        'error': COLOR_ERROR,
        'warn': COLOR_WARN,
        'notice': COLOR_NOTICE,
        'info': COLOR_OK,
    }
    return mapping.get(severity.lower(), COLOR_UNKNOWN)


def phase_color(phase: str) -> str:
    """運行フェーズ文字列から表示色を返す（6.3節の運行フェーズ列挙値）。"""

    mapping = {
        '走行中': COLOR_OK,
        '走行準備完了': COLOR_NOTICE,
        '起動確認中': COLOR_NOTICE,
        '終了処理中': COLOR_NOTICE,
        '一時停止': COLOR_WARN,
        '異常': COLOR_ERROR,
        '未起動': COLOR_UNKNOWN,
    }
    return mapping.get(phase, COLOR_UNKNOWN)


def launch_status_color(status: NodeLaunchStatus) -> str:
    """NodeLaunchStatus からNode Healthチップの表示色を返す。"""

    if status == NodeLaunchStatus.RUNNING:
        return COLOR_OK
    if status == NodeLaunchStatus.STARTING:
        return COLOR_NOTICE
    if status == NodeLaunchStatus.ERROR:
        return COLOR_ERROR
    if status == NodeLaunchStatus.STOPPING:
        return COLOR_WARN
    return COLOR_UNKNOWN
