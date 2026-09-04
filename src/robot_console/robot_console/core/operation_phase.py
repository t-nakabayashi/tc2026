"""運行フェーズ領域（ダッシュボード上段）の表示内容を算出する純粋関数群。

`robot_console_gui_screen_function_design.md` 6.3節が定義する運行フェーズ
（`未起動` / `起動確認中` / `走行準備完了` / `走行中` / `一時停止` / `異常` /
`終了処理中`）は単独のROS topicでは表現されず、起動管理状態・route/follower
状態・drive mode・manual_startの組み合わせから判定する必要がある。本モジュール
はその判定と、業務モード・進捗・current/next waypointの決定を1か所に集約する。

判定は「最も重い状態を優先する」方式とし、以下の順に評価する。

1. `異常`      : profile起動失敗、follower ERROR、route error
2. `終了処理中`: profile停止処理中、route completed、follower FINISHED
3. `未起動`    : 起動中/起動済みprofileが無く、運行系topicも未受信
4. `起動確認中`: profileがSTARTING、または起動済みだが運行系topicが未受信
5. `一時停止`  : 走行開始後（manual_start=True）に停止要因あり
6. `走行中`    : manual_start=True かつ follower/route が走行状態
7. `走行準備完了`: 上記以外（運行系topic受信済みで走行開始待ち）

`robot_console` 自身がノードを起動しない使い方（HTML遠隔観測UIの単独起動など）
では全profileがSTOPPEDのままになるため、起動管理状態だけでフェーズを決めず、
運行系topic（`route_state` / `follower_state`）の受信有無を併用する。

ROSに依存しないため、`ConsoleCore` と同様にROSなしで単体テストできる。
"""

from __future__ import annotations

from datetime import datetime
from typing import Dict, Optional, Tuple

from ..utils import NodeLaunchStatus
from .freshness import FreshnessLevel
from .launch_profile import LaunchProfileState
from .snapshot_model import (
    DriveModeStateView,
    FollowerView,
    OperationStateView,
    RouteView,
)

PHASE_NOT_STARTED = '未起動'
PHASE_STARTING = '起動確認中'
PHASE_READY = '走行準備完了'
PHASE_DRIVING = '走行中'
PHASE_PAUSED = '一時停止'
PHASE_ERROR = '異常'
PHASE_SHUTTING_DOWN = '終了処理中'

# 業務モード（起動・設定タブの走行モード選択）と、運行フェーズ領域の
# 走行モード表示（6.3節の `手動` / `自律`）の対応。
_DRIVE_MODE_SELECTION_LABELS = {
    '手動走行': '手動',
    '自律走行': '自律',
}

# `DriveModeStatus.mode`（`core/drive_mode_adapter.py` の変換結果）の表示文言。
_DRIVE_MODE_STATUS_LABELS = {
    'autonomous': '自律',
    'manual': '手動',
}

# route_follower（`follower_core.py` の `FollowerStatus`）のうち、走行中とみなす状態。
_FOLLOWER_DRIVING_STATES = ('RUNNING', 'AVOIDING')

# route_follower のうち、一時停止として扱う状態と表示する停止理由。
_FOLLOWER_PAUSE_REASONS = {
    'WAITING_STOP': '停止waypointで待機中（信号/停止線）',
    'WAITING_REROUTE': '再経路待ち（route_managerの応答待ち）',
    'STAGNATION_DETECTED': '滞留検知（回避判断中）',
}


def _has_status(launch_states: Dict[str, LaunchProfileState], status: NodeLaunchStatus) -> bool:
    return any(state.status == status for state in launch_states.values())


def _is_received(level: FreshnessLevel) -> bool:
    """一度でも受信した実績があるか（`UNKNOWN` 以外か）を返す。

    `STALE` / `LOST` は「受信済みだが途絶えた」状態であり、フェーズ判定上は
    未起動とは区別する（途絶自体はEventカード・鮮度表示側の責務）。
    """

    return level != FreshnessLevel.UNKNOWN


def resolve_pause_reason(
    *,
    route: RouteView,
    follower: FollowerView,
    drive_mode: DriveModeStateView,
) -> str:
    """一時停止の理由を1つ返す。停止要因が無い場合は空文字を返す。

    複数の要因が同時に成立し得るため、操作者が次に取るべき判断に近い順
    （follower の待機状態 → route_manager の holding → 手動介入・復帰待ち）で
    最初に成立したものを返す。
    """

    follower_reason = _FOLLOWER_PAUSE_REASONS.get(follower.state.upper())
    if follower_reason is not None:
        if follower.stagnation_reason:
            return f'{follower_reason}: {follower.stagnation_reason}'
        return follower_reason
    if route.state == 'holding':
        return f'route_manager holding: {route.last_replan_reason or "理由不明"}'
    if drive_mode.mode == 'manual':
        return '手動介入中（drive_mode=manual）'
    if drive_mode.auto_resume_pending:
        return '自律復帰待ち（auto resume pending）'
    return ''


def resolve_phase(
    *,
    launch_states: Dict[str, LaunchProfileState],
    route: RouteView,
    follower: FollowerView,
    drive_mode: DriveModeStateView,
    manual_start: bool,
    route_freshness: FreshnessLevel,
    follower_freshness: FreshnessLevel,
    pause_reason: str,
) -> str:
    """6.3節の運行フェーズを判定する（モジュールdocstringの優先順に評価する）。"""

    follower_state = follower.state.upper()

    if (
        _has_status(launch_states, NodeLaunchStatus.ERROR)
        or follower_state == 'ERROR'
        or route.state == 'error'
    ):
        return PHASE_ERROR

    if (
        _has_status(launch_states, NodeLaunchStatus.STOPPING)
        or route.state == 'completed'
        or follower_state == 'FINISHED'
    ):
        return PHASE_SHUTTING_DOWN

    operation_topics_received = _is_received(route_freshness) or _is_received(follower_freshness)

    if not operation_topics_received:
        if _has_status(launch_states, NodeLaunchStatus.STARTING) or _has_status(
            launch_states, NodeLaunchStatus.RUNNING
        ):
            return PHASE_STARTING
        return PHASE_NOT_STARTED

    if _has_status(launch_states, NodeLaunchStatus.STARTING):
        return PHASE_STARTING

    if manual_start:
        if pause_reason:
            return PHASE_PAUSED
        if follower_state in _FOLLOWER_DRIVING_STATES or route.state == 'running':
            return PHASE_DRIVING

    return PHASE_READY


def resolve_waypoint_labels(route: RouteView, follower: FollowerView) -> Tuple[str, str]:
    """current waypoint / next waypoint の表示文字列を返す。

    `RouteState.current_index` / `current_label` が現在追従中のwaypointを示す
    （route_managerがfollowerのindexを反映して配信する）。next waypointは
    `active_route` のwaypoint列から次のindexのlabelを引き、labelが無い場合は
    index表記（`#3`）へフォールバックする。

    Returns:
        Tuple[str, str]: (current waypoint表示, next waypoint表示)。
    """

    current_index = route.current_index
    # route_state / active_route のいずれも未受信の間は総数が0のままであり、
    # index表記（`#0`）を出すと受信済みと誤認させるため未確定表示にする。
    if current_index < 0 or (route.total_waypoints <= 0 and not route.waypoints):
        return ('-', '-')

    current_label = route.current_label or follower.active_waypoint_label
    current_text = current_label or f'#{current_index}'

    if route.total_waypoints and current_index + 1 >= route.total_waypoints:
        return (current_text, 'goal')

    next_index = current_index + 1
    next_label = ''
    for waypoint in route.waypoints:
        if waypoint.index == next_index:
            next_label = waypoint.label
            break
    return (current_text, next_label or f'#{next_index}')


def build_operation_state(
    *,
    environment: str,
    drive_mode_selection: str,
    launch_states: Dict[str, LaunchProfileState],
    route: RouteView,
    follower: FollowerView,
    drive_mode: DriveModeStateView,
    manual_start: bool,
    route_freshness: FreshnessLevel,
    follower_freshness: FreshnessLevel,
    now: Optional[datetime] = None,
) -> OperationStateView:
    """運行フェーズ領域の表示用データ（`OperationStateView`）を組み立てる。

    Args:
        environment (str): 起動・設定タブで選択中の実行環境（未選択時は `unknown`）。
        drive_mode_selection (str): 起動・設定タブで選択中の走行モード。
        launch_states (Dict[str, LaunchProfileState]): profileごとの起動状態。
        route (RouteView): route_manager由来の運行状態。
        follower (FollowerView): route_follower由来の追従状態。
        drive_mode (DriveModeStateView): drive_mode_manager由来の走行制御状態。
        manual_start (bool): `manual_start` topicの現在値。
        route_freshness (FreshnessLevel): `route_state` の受信鮮度。
        follower_freshness (FreshnessLevel): `follower_state` の受信鮮度。
        now (Optional[datetime]): 更新時刻。省略時は `updated_at` を埋めない。

    Returns:
        OperationStateView: 運行フェーズ領域の表示用データ。
    """

    pause_reason = resolve_pause_reason(route=route, follower=follower, drive_mode=drive_mode)
    phase = resolve_phase(
        launch_states=launch_states,
        route=route,
        follower=follower,
        drive_mode=drive_mode,
        manual_start=manual_start,
        route_freshness=route_freshness,
        follower_freshness=follower_freshness,
        pause_reason=pause_reason,
    )
    current_waypoint, next_waypoint = resolve_waypoint_labels(route, follower)

    # 走行モードは実際に走行制御が選択しているモード（drive_mode_status）を正とし、
    # 未受信の間だけ起動・設定タブの選択値で代替する。
    drive_mode_text = _DRIVE_MODE_STATUS_LABELS.get(
        drive_mode.mode,
        _DRIVE_MODE_SELECTION_LABELS.get(drive_mode_selection, drive_mode_selection or 'unknown'),
    )

    return OperationStateView(
        environment=environment or 'unknown',
        drive_mode=drive_mode_text,
        phase=phase,
        route_progress=route.progress_ratio,
        current_waypoint=current_waypoint,
        next_waypoint=next_waypoint,
        manual_start=manual_start,
        pause_reason=pause_reason if phase == PHASE_PAUSED else '',
        updated_at=now,
    )
