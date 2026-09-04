"""route/follower/targetのROSメッセージを表示用Viewへ変換する純粋関数群。

`RouteView` は `route_state` / `manager_status` / `active_route` の3トピックが
それぞれ別のフィールドを埋めるため、本モジュールの `apply_*_msg()` 系関数は
既存の `RouteView` を受け取り、対象フィールドだけを更新した新しい `RouteView`
を返す（`dataclasses.replace` によるイミュータブルな部分更新）。

ROSメッセージ型を直接importしないため、`rclpy` が無い環境でも単体テスト
できる（テストでは `types.SimpleNamespace` 等の同形オブジェクトを渡す）。
"""

from __future__ import annotations

from dataclasses import replace
from typing import Any, List

from .metrics import compute_progress_ratio
from .snapshot_model import FollowerView, ObstacleStateView, RouteView, RouteWaypointView, TargetView

_ROUTE_STATUS_LABELS = {
    0: 'unknown',
    1: 'idle',
    2: 'running',
    3: 'updating_route',
    4: 'holding',
    5: 'completed',
    6: 'error',
}


def route_status_label(value: int) -> str:
    """`tc_route_msgs/RouteState.status`（uint8）を表示用文字列へ変換する。"""

    return _ROUTE_STATUS_LABELS.get(int(value), 'unknown')


def apply_route_state_msg(route: RouteView, msg: Any) -> RouteView:
    """`tc_route_msgs/RouteState`（`route_state`）の内容を`RouteView`へ反映する。

    `current_index` は「現在追従中のwaypoint」を指し、完走時も最終waypointの
    index（`total_waypoints - 1`）で止まる。そのため進捗率をindexだけから
    算出すると、goal到達後も 100% に到達しない（21点なら 20/21 = 95.2%）。
    完走通知（`STATUS_COMPLETED`）を受けた場合は全waypoint走破として扱う。
    """

    state = route_status_label(msg.status)
    is_completed = state == 'completed'
    total_waypoints = int(msg.total_waypoints)
    current_index = int(msg.current_index)
    progress_ratio = (
        1.0 if is_completed else compute_progress_ratio(current_index, total_waypoints)
    )
    return replace(
        route,
        state=state,
        route_version=int(msg.route_version),
        current_index=current_index,
        current_label=str(getattr(msg, 'current_label', '')),
        total_waypoints=total_waypoints,
        progress_ratio=progress_ratio,
        is_completed=is_completed,
    )


def traveled_waypoint_count(route: RouteView) -> int:
    """走行済みとして扱うwaypoint数を返す（地図の色分け・進捗カウンタ用）。

    走行中は「現在追従中のwaypoint」の手前までが走行済みであり、`current_index`
    がそのまま走行済み点数になる。完走時のみ最終waypointを含む全点を走行済みと
    する（`current_index` は最終waypointのindexで止まるため、これを補正しないと
    最後の1点だけ未走行の色で残る）。
    """

    if route.is_completed:
        return max(route.total_waypoints, len(route.waypoints))
    return max(route.current_index, 0)


def apply_manager_status_msg(route: RouteView, msg: Any) -> RouteView:
    """`tc_route_msgs/ManagerStatus`（`manager_status`）の内容を`RouteView`へ反映する。"""

    return replace(
        route,
        last_decision=str(getattr(msg, 'decision', '')),
        last_replan_reason=str(getattr(msg, 'last_cause', '')),
    )


def waypoints_from_route_msg(msg: Any) -> List[RouteWaypointView]:
    """`tc_route_msgs/Route.waypoints[]` から地図重畳用のwaypoint列を組み立てる。

    `has_geo_pose` が真の要素のみ緯度経度を埋め、偽の要素はNoneのままとする
    （`geo_pose_source`がENUからの逆投影に依存し得るため、地図表示側で
    座標変換は行わない方針: architecture_design.md 6章）。`label` は緯度経度の
    有無に関わらず保持する（運行フェーズ領域のcurrent/next waypoint表示に使う）。
    """

    waypoints: List[RouteWaypointView] = []
    for waypoint in getattr(msg, 'waypoints', []):
        latitude = None
        longitude = None
        if bool(getattr(waypoint, 'has_geo_pose', False)):
            point = waypoint.geo_pose.point
            latitude = float(point.latitude)
            longitude = float(point.longitude)
        waypoints.append(
            RouteWaypointView(
                index=int(waypoint.index),
                label=str(getattr(waypoint, 'label', '')),
                latitude=latitude,
                longitude=longitude,
            )
        )
    return waypoints


def apply_route_msg(route: RouteView, msg: Any) -> RouteView:
    """`tc_route_msgs/Route`（`active_route`）の内容を`RouteView`へ反映する。"""

    return replace(
        route,
        route_version=int(msg.version),
        waypoints=waypoints_from_route_msg(msg),
    )


def follower_view_from_msg(msg: Any) -> FollowerView:
    """`tc_route_msgs/FollowerState`（`follower_state`）から`FollowerView`を組み立てる。"""

    return FollowerView(
        state=str(msg.state),
        active_waypoint_index=int(msg.active_waypoint_index),
        active_waypoint_label=str(msg.active_waypoint_label),
        stagnation_reason=str(getattr(msg, 'last_stagnation_reason', '')),
        avoidance_attempt_count=int(msg.avoidance_attempt_count),
        front_blocked=bool(msg.front_blocked),
        front_clearance_m=float(msg.front_clearance_m),
        left_offset_m=float(msg.left_offset_m),
        right_offset_m=float(msg.right_offset_m),
    )


def obstacle_view_from_hint_msg(msg: Any) -> ObstacleStateView:
    """`tc_route_msgs/ObstacleAvoidanceHint`（`obstacle_avoidance_hint`）から
    `ObstacleStateView` を組み立てる。
    """

    return ObstacleStateView(
        front_blocked=bool(msg.front_blocked),
        front_clearance_m=float(msg.front_clearance_m),
        left_offset_m=float(msg.left_offset_m),
        right_offset_m=float(msg.right_offset_m),
    )


def target_view_from_pose_msg(msg: Any) -> TargetView:
    """`geometry_msgs/PoseStamped`（`active_target`）から`TargetView`を組み立てる。

    distance_m/within_arrival_thresholdは自己位置との相対計算が必要なため
    ここでは埋めない（`ConsoleCore.update_active_target()`が算出する）。
    """

    position = msg.pose.position
    return TargetView(x_m=float(position.x), y_m=float(position.y), z_m=float(position.z))


def apply_active_target_llh_msg(target: TargetView, msg: Any) -> TargetView:
    """`tc_route_msgs/ActiveTargetLlh`（`route/active_target_llh`）を`TargetView`へ反映する。

    ENU→LLH変換は geo_pose_converter（`route_geo_projector_node`）の責務であり、
    `robot_console` は変換済みのLLHを受け取るだけとする。同メッセージは
    「GUI・HTML UI・ログ向け」に定義されており（`ActiveTargetLlh.msg`）、
    制御用のENU目標は `active_target` のまま維持される。ENU座標側は
    `active_target` 由来の値を保持し、緯度経度と距離・bearingのみ更新する。
    """

    point = msg.target_pose.point
    return replace(
        target,
        target_id=str(getattr(msg, 'target_label', '') or ''),
        latitude=float(point.latitude),
        longitude=float(point.longitude),
        altitude=float(point.altitude) if bool(getattr(point, 'has_altitude', False)) else None,
        distance_m=float(getattr(msg, 'distance_m', target.distance_m)),
        bearing_deg=float(getattr(msg, 'bearing_deg', 0.0)),
    )
