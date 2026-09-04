"""geo_core の座標変換に関する単体テスト."""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from geo_pose_converter.geo_core import (  # noqa: E402
    EnuPoint,
    LlhPoint,
    ProjectionConfig,
    bearing_from_map_delta,
    enu_to_llh,
    enu_to_llh_on_ground,
    heading_deg_to_yaw_enu_rad,
    llh_to_enu,
    load_projection_config_from_yaml,
    yaw_enu_rad_to_heading_deg,
)


def test_llh_enu_round_trip_preserves_point() -> None:
    """LLH -> ENU -> LLH の往復で元の座標へ戻ることを確認する."""

    projection = ProjectionConfig(
        origin_latitude=36.082331,
        origin_longitude=140.111681,
        origin_altitude=25.0,
        map_yaw_offset_rad=0.0,
        projection_id="test",
    )
    original = LlhPoint(36.082421, 140.111792, 27.5)

    enu = llh_to_enu(original, projection)
    restored = enu_to_llh(enu, projection)

    assert math.isclose(restored.latitude, original.latitude, abs_tol=1e-8)
    assert math.isclose(restored.longitude, original.longitude, abs_tol=1e-8)
    assert math.isclose(restored.altitude, original.altitude, abs_tol=1e-4)


def test_projection_yaw_offset_rotates_map_axes() -> None:
    """map_yaw_offset_rad により map 軸と ENU 軸が回転して対応することを確認する."""

    projection = ProjectionConfig(
        origin_latitude=36.0,
        origin_longitude=140.0,
        origin_altitude=0.0,
        map_yaw_offset_rad=math.pi / 2.0,
    )
    point = enu_to_llh(EnuPoint(10.0, 0.0, 0.0), projection)
    mapped = llh_to_enu(point, projection)

    assert math.isclose(mapped.x, 10.0, abs_tol=1e-6)
    assert math.isclose(mapped.y, 0.0, abs_tol=1e-6)


def test_heading_yaw_conversions_use_north_clockwise_heading() -> None:
    """heading は真北0度・時計回り、ENU yaw は東0rad・反時計回りとして相互変換する."""

    assert math.isclose(heading_deg_to_yaw_enu_rad(0.0), math.pi / 2.0)
    assert math.isclose(heading_deg_to_yaw_enu_rad(90.0), 0.0)
    assert math.isclose(yaw_enu_rad_to_heading_deg(math.pi / 2.0), 0.0)
    assert math.isclose(yaw_enu_rad_to_heading_deg(0.0), 90.0)


def test_bearing_from_map_delta_respects_projection_offset() -> None:
    """map 座標上の差分から現在の投影設定に基づく方位を算出する."""

    projection = ProjectionConfig(origin_latitude=36.0, origin_longitude=140.0, map_yaw_offset_rad=0.0)
    assert math.isclose(bearing_from_map_delta(0.0, 1.0, projection), 0.0)
    assert math.isclose(bearing_from_map_delta(1.0, 0.0, projection), 90.0)



def test_load_projection_config_from_yaml_reads_common_params(tmp_path: Path) -> None:
    """ROS 2 wildcard parameterからProjectionConfigを読み込む."""

    yaml_path = tmp_path / "default.yaml"
    yaml_path.write_text(
        "/**:\n"
        "  ros__parameters:\n"
        "    projection_id: tokyo_station\n"
        "    datum: WGS84\n"
        "    map_frame_id: map\n"
        "    earth_frame_id: earth\n"
        "    origin_latitude: 35.681382\n"
        "    origin_longitude: 139.766084\n"
        "    origin_altitude: 3.86\n"
        "    map_yaw_offset_rad: 0.1\n",
        encoding="utf-8",
    )

    projection = load_projection_config_from_yaml(str(yaml_path))

    assert projection.projection_id == "tokyo_station"
    assert projection.origin_latitude == 35.681382
    assert projection.origin_longitude == 139.766084
    assert projection.origin_altitude == 3.86
    assert projection.map_yaw_offset_rad == 0.1


def _tokyo_station_projection() -> ProjectionConfig:
    """params/default.yaml と同じ、東京駅を原点とする投影設定."""

    return ProjectionConfig(
        origin_latitude=35.681382,
        origin_longitude=139.766084,
        origin_altitude=3.86,
        map_yaw_offset_rad=0.0,
        projection_id="tokyo_station",
    )


def _horizontal_error_m(actual: LlhPoint, expected: LlhPoint) -> float:
    d_north = (actual.latitude - expected.latitude) * 111320.0
    d_east = (
        (actual.longitude - expected.longitude)
        * 111320.0
        * math.cos(math.radians(expected.latitude))
    )
    return math.hypot(d_north, d_east)


def test_enu_to_llh_with_zero_z_has_horizontal_error_far_from_origin() -> None:
    """z=0.0 のまま逆投影すると遠方で水平誤差が出る（本関数が解く問題の再現）.

    走行用ENU poseは2D座標としてzを0.0へ正規化するため、原点から離れた地点では
    接平面の曲率降下ぶんだけ「原点の鉛直方向へ持ち上げた点」を逆投影してしまう。
    """

    projection = _tokyo_station_projection()
    ground = LlhPoint(36.0829271, 140.0769037, 0.0)  # つくば（原点から約52.7km）
    enu = llh_to_enu(ground, projection)

    restored = enu_to_llh(EnuPoint(enu.x, enu.y, 0.0), projection)

    assert _horizontal_error_m(restored, ground) > 1.0


def test_enu_to_llh_on_ground_removes_horizontal_error_far_from_origin() -> None:
    """水平座標を地表点として扱えば、遠方でも往復が一致することを確認する."""

    projection = _tokyo_station_projection()
    ground = LlhPoint(36.0829271, 140.0769037, 0.0)
    enu = llh_to_enu(ground, projection)

    restored = enu_to_llh_on_ground(enu.x, enu.y, projection)

    assert _horizontal_error_m(restored, ground) < 0.01


def test_enu_to_llh_on_ground_round_trips_through_llh_to_enu() -> None:
    """`llh_to_enu()` との往復で水平座標が保存されることを確認する."""

    projection = _tokyo_station_projection()

    restored = enu_to_llh_on_ground(27995.143, 44598.093, projection)
    enu = llh_to_enu(LlhPoint(restored.latitude, restored.longitude, 0.0), projection)

    assert math.isclose(enu.x, 27995.143, abs_tol=0.01)
    assert math.isclose(enu.y, 44598.093, abs_tol=0.01)


def test_enu_to_llh_on_ground_honors_ground_altitude() -> None:
    """指定した地表高度の点として解けることを確認する."""

    projection = _tokyo_station_projection()
    ground = LlhPoint(36.0829271, 140.0769037, 25.0)
    enu = llh_to_enu(ground, projection)

    restored = enu_to_llh_on_ground(enu.x, enu.y, projection, ground_altitude=25.0)

    assert _horizontal_error_m(restored, ground) < 0.01


def test_enu_to_llh_on_ground_matches_enu_to_llh_near_origin() -> None:
    """原点近傍では従来の変換と実質差が無いことを確認する（回帰防止）."""

    projection = _tokyo_station_projection()

    on_ground = enu_to_llh_on_ground(10.0, 20.0, projection)
    plain = enu_to_llh(EnuPoint(10.0, 20.0, 0.0), projection)

    assert _horizontal_error_m(on_ground, plain) < 0.001
