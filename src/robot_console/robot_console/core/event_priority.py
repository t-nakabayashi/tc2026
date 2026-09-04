"""Eventカードの優先順位定義（UI非依存）。

`robot_console_gui_screen_function_design.md` 6.7節の優先順位は、PyQt5の
Eventカードと HTML遠隔観測UI の双方が同じ並びで表示する必要がある。PyQt5側の
widgetモジュールへ置くとHTML UI（PyQt5非依存）から参照できないため、UI非依存の
core側へ定義を置き、両UIがこれを共有する。
"""

from __future__ import annotations

from typing import List

from .snapshot_model import EventBanner

# 6.7節「優先順位は、profile error/topic lost、road_blocked、front_blocked、
# signal STOP/WAITING_STOP、manual_start待ち、signal GO、route update の順を基本とする」。
PRIORITY_ORDER = [
    'profile_error',
    'topic_lost',
    'road_blocked',
    'front_blocked',
    'signal_stop',
    'manual_start_pending',
    'signal_go',
    'route_update',
]


def priority_rank(banner: EventBanner) -> int:
    """イベント種別の優先順位を返す。未知の種別は最下位とする。"""

    try:
        return PRIORITY_ORDER.index(banner.event_type)
    except ValueError:
        return len(PRIORITY_ORDER)


def sort_by_priority(banners: List[EventBanner]) -> List[EventBanner]:
    """6.7節の優先順位に従いEventBannerを並び替える。"""

    return sorted(banners, key=priority_rank)
