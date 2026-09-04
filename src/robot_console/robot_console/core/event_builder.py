"""Eventカード向けのイベントバナーを生成する純粋関数群。

`robot_console_gui_screen_function_design.md` 6.7節が定義するEventカードは、
走行中に操作者が即座に気づくべき状態変化を優先度付きで表示する。表示対象は
単一のtopicではなく、profile起動状態・topic鮮度・障害物・信号・道路封鎖・
route更新の組み合わせから決まるため、Snapshot生成時にまとめて組み立てる。

`event_type` は `core/event_priority.py` の `PRIORITY_ORDER` と対応させる
（並び替えは表示側が `sort_by_priority()` で行う）。

ROSに依存しないため、`ConsoleCore` と同様にROSなしで単体テストできる。
"""

from __future__ import annotations

from datetime import datetime
from typing import Dict, List, Optional

from ..utils import NodeLaunchStatus
from .freshness import FreshnessLevel
from .launch_profile import LaunchProfileState
from .snapshot_model import (
    EventBanner,
    FollowerView,
    ManualControlsView,
    ObstacleStateView,
    RouteView,
)

# `sig_recog` の値と表示文言の対応（route_followerのSTOP解除条件に合わせる）。
SIG_RECOG_STOP = 0
SIG_RECOG_GO = 1

# route_follower（`follower_core.py` の `FollowerStatus`）のうち、停止待機を表す状態。
_FOLLOWER_WAITING_STATES = ('WAITING_STOP', 'WAITING_REROUTE', 'STAGNATION_DETECTED')


def _banner(
    event_type: str,
    message: str,
    severity: str,
    source: str,
    occurred_at: Optional[datetime],
) -> EventBanner:
    return EventBanner(
        event_type=event_type,
        message=message,
        severity=severity,
        source=source,
        occurred_at=occurred_at,
    )


def build_event_banners(
    *,
    launch_states: Dict[str, LaunchProfileState],
    route: RouteView,
    follower: FollowerView,
    obstacle: ObstacleStateView,
    manual_controls: ManualControlsView,
    operation_phase: str,
    lost_topics: Optional[Dict[str, float]] = None,
    now: Optional[datetime] = None,
) -> List[EventBanner]:
    """現在の状態から表示すべきイベントバナー一覧を組み立てる。

    Args:
        launch_states (Dict[str, LaunchProfileState]): profileごとの起動状態。
        route (RouteView): route_manager由来の運行状態。
        follower (FollowerView): route_follower由来の追従状態。
        obstacle (ObstacleStateView): 障害物hint由来の状態。
        manual_controls (ManualControlsView): 手動操作トピックの現在値。
        operation_phase (str): 運行フェーズ（`manual_start待ち` の判定に使う）。
        lost_topics (Optional[Dict[str, float]]): 途絶したtopic名と経過秒。
        now (Optional[datetime]): 発生時刻として記録する時刻。

    Returns:
        List[EventBanner]: 表示対象のイベント一覧（並び替えは表示側が行う）。
    """

    banners: List[EventBanner] = []

    # 1) profile error: 起動に失敗したノードは走行継続の可否に直結する。
    for profile_id, state in launch_states.items():
        if state.status == NodeLaunchStatus.ERROR:
            detail = state.error_message.strip().splitlines()[0] if state.error_message else ''
            message = f'{profile_id} ERROR'
            if detail:
                message = f'{message} {detail}'
            banners.append(
                _banner('profile_error', message, 'error', profile_id, state.last_action_time)
            )

    # 2) topic lost: 表示値が現在値でなくなっていることを知らせる。
    for topic, elapsed_sec in (lost_topics or {}).items():
        banners.append(
            _banner(
                'topic_lost',
                f'{topic} LOST {elapsed_sec:.1f}s',
                'error',
                topic,
                now,
            )
        )

    # 3) road_blocked: 経路自体が通れないため最優先の走行判断材料になる。
    if manual_controls.road_blocked_value:
        banners.append(
            _banner(
                'road_blocked',
                f'ROAD BLOCKED {manual_controls.road_blocked_source}',
                'warn',
                manual_controls.road_blocked_source,
                manual_controls.road_blocked_last_sent_at or now,
            )
        )

    # 4) front_blocked: 前方障害物。follower/hintのどちらかが検知していれば出す。
    if follower.front_blocked or obstacle.front_blocked:
        clearance = (
            follower.front_clearance_m if follower.front_blocked else obstacle.front_clearance_m
        )
        left = follower.left_offset_m if follower.front_blocked else obstacle.left_offset_m
        right = follower.right_offset_m if follower.front_blocked else obstacle.right_offset_m
        banners.append(
            _banner(
                'front_blocked',
                f'FRONT BLOCKED clearance={clearance:.1f}m L={left:.1f} R={right:.1f}',
                'warn',
                'follower' if follower.front_blocked else 'obstacle_hint',
                now,
            )
        )

    # 5) signal stop / 待機: 停止waypointでの待機は操作者の介入待ちであり得る。
    follower_state = follower.state.upper()
    if follower_state in _FOLLOWER_WAITING_STATES:
        label = follower.active_waypoint_label or f'#{follower.active_waypoint_index}'
        message = f'{follower_state} {label}'
        if follower.stagnation_reason:
            message = f'{message} ({follower.stagnation_reason})'
        banners.append(_banner('signal_stop', message, 'warn', 'route_follower', now))

    # 6) manual_start待ち: 走行準備完了のまま開始操作を待っている状態。
    if operation_phase == '走行準備完了' and not manual_controls.manual_start_value:
        banners.append(
            _banner('manual_start_pending', 'manual_start 待ち', 'notice', 'gui', now)
        )

    # 7) signal GO: 直近のGO受信を短く出す（停止解除の根拠を残すため）。
    if manual_controls.sig_recog_value == SIG_RECOG_GO:
        banners.append(
            _banner(
                'signal_go',
                f'SIGNAL GO {manual_controls.input_source}',
                'info',
                manual_controls.input_source,
                manual_controls.sig_recog_last_sent_at or now,
            )
        )

    # 8) route update: 再計画が起きたことを版数と理由で示す。
    if route.last_decision and route.last_decision not in ('none', ''):
        message = f'ROUTE {route.last_decision.upper()} v{route.route_version}'
        if route.last_replan_reason:
            message = f'{message} {route.last_replan_reason}'
        banners.append(_banner('route_update', message, 'notice', 'route_manager', now))

    return banners
