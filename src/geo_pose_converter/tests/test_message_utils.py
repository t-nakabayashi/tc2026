"""message_utils の高度有効フラグに関する単体テスト."""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from geometry_msgs.msg import Pose
from std_msgs.msg import Header

from geo_pose_converter.geo_core import LlhPoint, ProjectionConfig
from geo_pose_converter.message_utils import (
    llh_to_pose_with_covariance,
    make_active_target_llh,
    make_geo_pose,
    pose_to_llh_pose,
)


def _projection() -> ProjectionConfig:
    return ProjectionConfig(
        origin_latitude=36.0,
        origin_longitude=140.0,
        origin_altitude=25.0,
        map_yaw_offset_rad=0.0,
    )


def test_llh_to_pose_with_covariance_keeps_enu_z_zero() -> None:
    """GNSS高度があっても走行用ENU zは0.0へ正規化する."""

    pose = llh_to_pose_with_covariance(
        Header(),
        LlhPoint(36.000001, 140.000002, 30.0),
        90.0,
        True,
        _projection(),
    )

    assert pose.pose.pose.position.z == 0.0


def test_pose_to_llh_pose_marks_projected_altitude_invalid_by_default() -> None:
    """ENU逆投影で作ったLLH poseは既定で高度を有効扱いしない."""

    pose = Pose()
    pose.position.x = 1.0
    pose.position.y = 2.0
    pose.position.z = 0.0
    pose.orientation.w = 1.0

    geo_pose = pose_to_llh_pose(Header(), pose, _projection(), "active_target")

    assert not geo_pose.point.has_altitude


def test_make_active_target_llh_ignores_invalid_altitude_for_distance() -> None:
    """高度無効のtarget/currentでも2D距離を算出できる."""

    projection = _projection()
    current = make_geo_pose(
        Header(),
        LlhPoint(36.0, 140.0, 0.0),
        0.0,
        True,
        "current",
        has_altitude=False,
    )
    target = make_geo_pose(
        Header(),
        LlhPoint(36.000001, 140.0, 0.0),
        0.0,
        True,
        "target",
        has_altitude=False,
    )

    msg = make_active_target_llh(Header(), 1, -1, "", target, current, projection, True)

    assert msg.is_avoidance_subgoal
    assert msg.distance_m > 0.0
    assert not msg.target_pose.point.has_altitude


def test_pose_to_llh_pose_round_trips_with_llh_to_pose_far_from_origin() -> None:
    """原点から離れた地点でも LLH -> ENU -> LLH の水平位置が一致することを確認する.

    走行用ENU poseはzを0.0へ正規化するため、逆投影でzを素直に信じると
    原点から離れるほど水平誤差が出る（東京駅原点・つくばで約1.8m）。
    GUI・HTML UIの地図はこのLLHで自己位置・目標を描画するため、
    routeのwaypoint（route file由来のLLH）とズレないことを保証する。
    """

    import math

    projection = ProjectionConfig(
        origin_latitude=35.681382,
        origin_longitude=139.766084,
        origin_altitude=3.86,
        map_yaw_offset_rad=0.0,
        projection_id="tokyo_station",
    )
    original = LlhPoint(36.0829271, 140.0769037, 0.0)

    enu_pose_msg = llh_to_pose_with_covariance(Header(), original, 0.0, False, projection)
    pose = Pose()
    pose.position.x = enu_pose_msg.pose.pose.position.x
    pose.position.y = enu_pose_msg.pose.pose.position.y
    pose.position.z = enu_pose_msg.pose.pose.position.z  # 0.0 に正規化されている
    pose.orientation.w = 1.0

    geo_pose = pose_to_llh_pose(Header(), pose, projection, "base_link")

    d_north = (geo_pose.point.latitude - original.latitude) * 111320.0
    d_east = (
        (geo_pose.point.longitude - original.longitude)
        * 111320.0
        * math.cos(math.radians(original.latitude))
    )
    assert math.hypot(d_north, d_east) < 0.01


def _tokyo_station_projection() -> ProjectionConfig:
    return ProjectionConfig(
        origin_latitude=35.681382,
        origin_longitude=139.766084,
        origin_altitude=3.86,
        map_yaw_offset_rad=0.0,
        projection_id="tokyo_station",
    )


def test_llh_to_pose_ignores_gnss_altitude_for_horizontal_position() -> None:
    """GNSS高度が変わっても水平ENU座標が変わらないことを確認する.

    local tangent planeでは高度差が水平座標へ 高度差 × d/R で効くため、
    高度規約が経路ごとに異なると同じ地点が別のENU座標になる。waypoint生成と
    同じ `origin_altitude` 基準へ揃え、実機GNSSの高度変動で自己位置が
    waypointに対してずれないようにする。
    """

    projection = _tokyo_station_projection()
    header = Header()

    low = llh_to_pose_with_covariance(
        header, LlhPoint(36.0829271, 140.0769037, 0.0), 0.0, False, projection
    )
    high = llh_to_pose_with_covariance(
        header, LlhPoint(36.0829271, 140.0769037, 62.0), 0.0, False, projection
    )

    assert low.pose.pose.position.x == high.pose.pose.position.x
    assert low.pose.pose.position.y == high.pose.pose.position.y


def test_enu_llh_round_trip_is_consistent_across_conversion_paths() -> None:
    """ENU -> LLH -> ENU の往復が一致することを確認する（高度規約の統一）."""

    import math

    projection = _tokyo_station_projection()
    x, y = 27995.143, 44598.093

    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = 0.0
    pose.orientation.w = 1.0

    geo_pose = pose_to_llh_pose(Header(), pose, projection, "base_link")
    restored = llh_to_pose_with_covariance(
        Header(),
        LlhPoint(geo_pose.point.latitude, geo_pose.point.longitude, 0.0),
        0.0,
        False,
        projection,
    )

    assert math.isclose(restored.pose.pose.position.x, x, abs_tol=0.01)
    assert math.isclose(restored.pose.pose.position.y, y, abs_tol=0.01)
