"""llh_osm_viewer_node の表示用状態生成に関する単体テスト."""

from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from tc_geo_msgs.msg import GeoPoseWithQuality  # noqa: E402
from tc_route_msgs.msg import ActiveTargetLlh, Route, Waypoint  # noqa: E402

from geo_pose_converter.llh_osm_viewer_node import (  # noqa: E402
    PoseStore,
    _active_target_to_dict,
    _make_html,
    _pose_llh_to_dict,
    _route_to_dict,
)


def _fill_geo_pose(geo_pose, latitude: float, longitude: float, heading_deg: float) -> None:
    """テスト用GeoPoseへLLH/headingを設定する."""
    geo_pose.point.latitude = latitude
    geo_pose.point.longitude = longitude
    geo_pose.point.altitude = 10.0
    geo_pose.point.has_altitude = True
    geo_pose.heading_deg = heading_deg
    geo_pose.has_heading = True
    geo_pose.child_frame_id = 'base_link'


def test_pose_llh_to_dict_keeps_quality_and_received_time() -> None:
    """GeoPoseWithQualityをHTTP JSON用dictへ変換できる."""
    msg = GeoPoseWithQuality()
    msg.header.frame_id = 'earth'
    msg.header.stamp.sec = 12
    msg.header.stamp.nanosec = 500000000
    _fill_geo_pose(msg.pose, 36.0, 140.0, 450.0)
    msg.fix_quality = GeoPoseWithQuality.FIX_RTK_FIX
    msg.fusion_status = GeoPoseWithQuality.FUSION_OK
    msg.source = GeoPoseWithQuality.SOURCE_SIMULATION
    msg.status_text = 'ok'

    pose = _pose_llh_to_dict(msg)

    assert pose is not None
    assert pose['latitude'] == 36.0
    assert pose['longitude'] == 140.0
    assert pose['heading_deg'] == 90.0
    assert pose['fix_quality'] == GeoPoseWithQuality.FIX_RTK_FIX
    assert pose['fusion_status'] == GeoPoseWithQuality.FUSION_OK
    assert pose['source'] == GeoPoseWithQuality.SOURCE_SIMULATION
    assert pose['stamp'] == 12.5
    assert 'received_wall_time' in pose


def test_pose_llh_to_dict_rejects_invalid_latitude() -> None:
    """表示できない緯度はJSON化しない."""
    msg = GeoPoseWithQuality()
    _fill_geo_pose(msg.pose, 100.0, 140.0, 0.0)

    assert _pose_llh_to_dict(msg) is None


def test_route_to_dict_uses_only_geo_waypoints() -> None:
    """Route overlayはLLHを持つwaypointだけを抽出する."""
    route = Route()
    route.route_id = 'test_route'
    route.version = 3
    route.total_distance = 12.5
    route.map_frame_id = 'map'
    route.earth_frame_id = 'earth'
    route.projection.projection_id = 'tokyo_station'

    wp1 = Waypoint()
    wp1.index = 1
    wp1.label = 'A'
    wp1.has_geo_pose = True
    wp1.segment_is_fixed = True
    _fill_geo_pose(wp1.geo_pose, 36.0, 140.0, 0.0)

    wp2 = Waypoint()
    wp2.index = 2
    wp2.label = 'B'
    wp2.has_geo_pose = False

    route.waypoints = [wp1, wp2]

    data = _route_to_dict(route, max_waypoints=10)

    assert data['route_id'] == 'test_route'
    assert data['version'] == 3
    assert len(data['waypoints']) == 1
    assert data['waypoints'][0]['label'] == 'A'
    assert data['skipped_waypoints'] == 1
    assert data['projection_id'] == 'tokyo_station'


def test_active_target_to_dict_keeps_target_metadata() -> None:
    """ActiveTargetLlhをHTTP JSON用dictへ変換できる."""
    msg = ActiveTargetLlh()
    msg.route_version = 2
    msg.target_index = 5
    msg.target_label = 'C5'
    msg.distance_m = 8.25
    msg.bearing_deg = 12.0
    msg.target_kind = 'route_waypoint'
    _fill_geo_pose(msg.target_pose, 36.1, 140.2, 180.0)

    data = _active_target_to_dict(msg)

    assert data is not None
    assert data['target_index'] == 5
    assert data['target_label'] == 'C5'
    assert data['distance_m'] == 8.25
    assert data['bearing_deg'] == 12.0


def test_pose_store_reports_ok_stale_lost_status() -> None:
    """PoseStoreは受信時刻からOK/STALE/LOSTを判定する."""
    store = PoseStore()
    store.update({'latitude': 36.0, 'longitude': 140.0, 'received_wall_time': time.time()})

    assert store.get_state(1.0, 3.0)['pose_status'] == 'OK'

    store.update({'latitude': 36.0, 'longitude': 140.0, 'received_wall_time': time.time() - 2.0})
    assert store.get_state(1.0, 3.0)['pose_status'] == 'STALE'

    store.update({'latitude': 36.0, 'longitude': 140.0, 'received_wall_time': time.time() - 4.0})
    assert store.get_state(1.0, 3.0)['pose_status'] == 'LOST'


def test_make_html_polls_state_and_draws_overlays() -> None:
    """生成HTMLは/stateを購読しroute/target overlayを描画する."""
    html = _make_html(
        poll_interval_ms=200,
        initial_zoom=19,
        default_latitude=36.0,
        default_longitude=140.0,
        default_zoom=17,
        triangle_height_px=48.0,
    )

    assert "fetch('/state'" in html
    assert 'updateRoute(state.route)' in html
    assert 'updateActiveTarget(state.active_target)' in html
