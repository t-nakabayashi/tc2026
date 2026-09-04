"""core/route_adapter.py の単体テスト（ROS非依存）。"""

from types import SimpleNamespace

import pytest

from robot_console.core.route_adapter import (
    apply_active_target_llh_msg,
    apply_manager_status_msg,
    apply_route_msg,
    apply_route_state_msg,
    follower_view_from_msg,
    obstacle_view_from_hint_msg,
    route_status_label,
    target_view_from_pose_msg,
    traveled_waypoint_count,
    waypoints_from_route_msg,
)
from robot_console.core.snapshot_model import RouteView, TargetView


def _waypoint(index, *, has_geo_pose=False, latitude=None, longitude=None):
    geo_pose = SimpleNamespace(point=SimpleNamespace(latitude=latitude, longitude=longitude))
    return SimpleNamespace(index=index, has_geo_pose=has_geo_pose, geo_pose=geo_pose)


def test_route_status_label_maps_known_values():
    assert route_status_label(2) == 'running'
    assert route_status_label(6) == 'error'


def test_route_status_label_falls_back_to_unknown():
    assert route_status_label(99) == 'unknown'


def test_apply_route_state_msg_updates_progress_fields():
    msg = SimpleNamespace(status=2, route_version=4, current_index=3, total_waypoints=10)

    route = apply_route_state_msg(RouteView(), msg)

    assert route.state == 'running'
    assert route.route_version == 4
    assert route.current_index == 3
    assert route.total_waypoints == 10
    assert route.progress_ratio == 0.3


def test_apply_route_state_msg_preserves_waypoints_from_other_updates():
    route = RouteView(waypoints=[])
    msg = SimpleNamespace(status=2, route_version=4, current_index=1, total_waypoints=2)

    updated = apply_route_state_msg(route, msg)

    assert updated.waypoints == route.waypoints  # 他フィールドは変更しない


def test_apply_manager_status_msg_updates_decision_fields():
    msg = SimpleNamespace(decision='avoid', last_cause='obstacle_detected')

    route = apply_manager_status_msg(RouteView(), msg)

    assert route.last_decision == 'avoid'
    assert route.last_replan_reason == 'obstacle_detected'


def test_waypoints_from_route_msg_extracts_lat_lon_only_when_has_geo_pose():
    msg = SimpleNamespace(
        waypoints=[
            _waypoint(0, has_geo_pose=True, latitude=36.083, longitude=140.113),
            _waypoint(1, has_geo_pose=False),
        ]
    )

    waypoints = waypoints_from_route_msg(msg)

    assert waypoints[0].index == 0
    assert waypoints[0].latitude == 36.083
    assert waypoints[1].latitude is None
    assert waypoints[1].longitude is None


def test_apply_route_msg_updates_version_and_waypoints():
    msg = SimpleNamespace(version=7, waypoints=[_waypoint(0, has_geo_pose=True, latitude=1.0, longitude=2.0)])

    route = apply_route_msg(RouteView(), msg)

    assert route.route_version == 7
    assert len(route.waypoints) == 1


def test_follower_view_from_msg_maps_all_fields():
    msg = SimpleNamespace(
        state='following', active_waypoint_index=5, active_waypoint_label='A-5',
        last_stagnation_reason='', avoidance_attempt_count=1, front_blocked=True,
        front_clearance_m=0.5, left_offset_m=0.1, right_offset_m=0.2,
    )

    view = follower_view_from_msg(msg)

    assert view.state == 'following'
    assert view.active_waypoint_index == 5
    assert view.front_blocked is True


def test_obstacle_view_from_hint_msg_maps_all_fields():
    msg = SimpleNamespace(front_blocked=True, front_clearance_m=0.8, left_offset_m=0.1, right_offset_m=0.2)

    view = obstacle_view_from_hint_msg(msg)

    assert view.front_blocked is True
    assert view.front_clearance_m == 0.8
    assert view.left_offset_m == 0.1
    assert view.right_offset_m == 0.2


def test_target_view_from_pose_msg_extracts_position_only():
    msg = SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=3.0, y=4.0, z=0.0)))

    view = target_view_from_pose_msg(msg)

    assert view.x_m == 3.0
    assert view.y_m == 4.0
    assert view.distance_m == 0.0  # 未算出（ConsoleCore側で計算）


def test_apply_route_state_msg_keeps_current_label():
    msg = SimpleNamespace(
        status=2, route_version=4, current_index=3, current_label='A-13', total_waypoints=10
    )

    route = apply_route_state_msg(RouteView(), msg)

    assert route.current_label == 'A-13'


def test_waypoints_from_route_msg_keeps_label_without_geo_pose():
    """labelは運行フェーズ領域のnext waypoint表示に使うため、緯度経度が無くても保持する。"""

    waypoint = _waypoint(2)
    waypoint.label = 'A-12'
    msg = SimpleNamespace(waypoints=[waypoint])

    views = waypoints_from_route_msg(msg)

    assert views[0].label == 'A-12'
    assert views[0].latitude is None


def test_apply_route_state_msg_reaches_full_progress_on_completion():
    """完走時は current_index が最終waypointのindexで止まるため、

    indexからの算出のままでは 100% にならない（21点なら 20/21 = 95.2%）。
    完走通知を受けた場合は全waypoint走破として扱うことを確認する。
    """

    msg = SimpleNamespace(
        status=5, route_version=100, current_index=20, current_label='30', total_waypoints=21
    )

    route = apply_route_state_msg(RouteView(), msg)

    assert route.state == 'completed'
    assert route.is_completed is True
    assert route.progress_ratio == 1.0


def test_apply_route_state_msg_keeps_index_based_progress_while_running():
    msg = SimpleNamespace(
        status=2, route_version=100, current_index=20, current_label='30', total_waypoints=21
    )

    route = apply_route_state_msg(RouteView(), msg)

    assert route.is_completed is False
    assert route.progress_ratio == pytest.approx(20 / 21)


def test_traveled_waypoint_count_uses_current_index_while_running():
    route = RouteView(current_index=14, total_waypoints=21)

    assert traveled_waypoint_count(route) == 14


def test_traveled_waypoint_count_covers_all_waypoints_on_completion():
    """完走時に最終waypointが未走行の色で残らないことを保証する。"""

    route = RouteView(current_index=20, total_waypoints=21, is_completed=True)

    assert traveled_waypoint_count(route) == 21


def _active_target_llh_msg(latitude, longitude, *, label='30', distance_m=3.5, bearing_deg=12.0):
    return SimpleNamespace(
        target_label=label,
        target_index=20,
        route_version=100,
        target_pose=SimpleNamespace(
            point=SimpleNamespace(
                latitude=latitude, longitude=longitude, altitude=0.0, has_altitude=False
            )
        ),
        distance_m=distance_m,
        bearing_deg=bearing_deg,
    )


def test_apply_active_target_llh_msg_fills_latitude_longitude():
    """ENU→LLH変換は geo_pose_converter の責務であり、変換済みLLHを受け取るだけとする。"""

    target = apply_active_target_llh_msg(TargetView(), _active_target_llh_msg(36.0833, 140.0769))

    assert target.latitude == 36.0833
    assert target.longitude == 140.0769
    assert target.target_id == '30'
    assert target.bearing_deg == 12.0


def test_apply_active_target_llh_msg_keeps_enu_position_from_active_target():
    """制御用ENU目標（active_target由来）はLLH反映で失われない。"""

    target = TargetView(x_m=28000.0, y_m=44640.0, z_m=0.0)

    updated = apply_active_target_llh_msg(target, _active_target_llh_msg(36.0833, 140.0769))

    assert updated.x_m == 28000.0
    assert updated.y_m == 44640.0
