"""route_builder の LLH CSV 入出力に関する単体テスト."""

from __future__ import annotations

import csv
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from geo_pose_converter.geo_core import LlhPoint, ProjectionConfig, llh_to_enu  # noqa: E402
from route_planner.route_builder import parse_waypoint_csv, write_waypoints_to_csv  # noqa: E402


def test_parse_waypoint_csv_reads_llh_altitude_and_heading(tmp_path: Path) -> None:
    """waypoint CSV の LLH/altitude/heading_deg を WaypointRecord に保持する."""

    csv_path = tmp_path / "waypoints.csv"
    csv_path.write_text(
        "label,x,y,z,q1,q2,q3,q4,latitude,longitude,altitude,heading_deg\n"
        "wp1,1.0,2.0,3.0,0,0,0,1,36.082331,140.111681,25.5,90.0\n",
        encoding="utf-8",
    )

    waypoints = parse_waypoint_csv(str(csv_path))

    assert len(waypoints) == 1
    assert waypoints[0].latitude == 36.082331
    assert waypoints[0].longitude == 140.111681
    assert waypoints[0].altitude == 25.5
    assert waypoints[0].heading_deg == 90.0


def test_write_waypoints_to_csv_keeps_llh_columns(tmp_path: Path) -> None:
    """WaypointRecord を CSV に書き戻す際も LLH 系カラムを出力する."""

    source_path = tmp_path / "source.csv"
    source_path.write_text(
        "label,x,y,z,q1,q2,q3,q4,lat,lon,alt,heading\n"
        "wp1,1.0,2.0,3.0,0,0,0,1,36.082331,140.111681,25.5,90.0\n",
        encoding="utf-8",
    )
    waypoints = parse_waypoint_csv(str(source_path))
    out_path = tmp_path / "out.csv"

    write_waypoints_to_csv(str(out_path), waypoints)

    with out_path.open(newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    assert rows[0]["latitude"] == "36.082331"
    assert rows[0]["longitude"] == "140.111681"
    assert rows[0]["altitude"] == "25.5"
    assert rows[0]["heading_deg"] == "90.0"


class _Logger:
    """テスト用 warning 収集 logger."""

    def __init__(self) -> None:
        self.messages: list[str] = []

    def warn(self, message: str) -> None:
        self.messages.append(message)


def test_parse_waypoint_csv_projects_llh_only_route(tmp_path: Path) -> None:
    """LLH-only CSVはprojectionでENU poseを生成する."""

    projection = ProjectionConfig(
        origin_latitude=36.0,
        origin_longitude=140.0,
        origin_altitude=0.0,
        map_yaw_offset_rad=0.0,
    )
    csv_path = tmp_path / "llh_only.csv"
    csv_path.write_text(
        "label,latitude,longitude,altitude,heading_deg\n"
        "wp1,36.000001,140.000002,1.5,90.0\n",
        encoding="utf-8",
    )

    waypoints = parse_waypoint_csv(str(csv_path), projection=projection)
    expected = llh_to_enu(LlhPoint(36.000001, 140.000002, 1.5), projection)

    assert len(waypoints) == 1
    assert abs(waypoints[0].pose.position.x - expected.x) < 1.0e-6
    assert abs(waypoints[0].pose.position.y - expected.y) < 1.0e-6
    assert waypoints[0].pose.position.z == 0.0
    assert abs(waypoints[0].pose.orientation.z) < 1.0e-9
    assert abs(waypoints[0].pose.orientation.w - 1.0) < 1.0e-9


def test_parse_waypoint_csv_prefers_llh_and_warns_when_both_exist(tmp_path: Path) -> None:
    """LLHとENUが併記された場合はwarningし、LLH正本でposeを上書きする."""

    projection = ProjectionConfig(
        origin_latitude=36.0,
        origin_longitude=140.0,
        origin_altitude=0.0,
        map_yaw_offset_rad=0.0,
    )
    csv_path = tmp_path / "both.csv"
    csv_path.write_text(
        "label,x,y,z,q1,q2,q3,q4,latitude,longitude,altitude,heading_deg\n"
        "wp1,100.0,200.0,0.0,0,0,0,1,36.000001,140.000002,1.5,90.0\n",
        encoding="utf-8",
    )
    logger = _Logger()

    waypoints = parse_waypoint_csv(str(csv_path), projection=projection, logger=logger)
    expected = llh_to_enu(LlhPoint(36.000001, 140.000002, 1.5), projection)

    assert abs(waypoints[0].pose.position.x - expected.x) < 1.0e-6
    assert waypoints[0].pose.position.z == 0.0
    assert any("LLH座標とENU座標が併記" in message for message in logger.messages)
    assert any("水平誤差" in message for message in logger.messages)


def test_parse_waypoint_csv_projects_llh_without_altitude_to_2d_enu(tmp_path: Path) -> None:
    """altitude空欄のLLH-only CSVはENU zを0.0にし、altitudeは未指定のまま保持する."""

    projection = ProjectionConfig(
        origin_latitude=36.0,
        origin_longitude=140.0,
        origin_altitude=25.0,
        map_yaw_offset_rad=0.0,
    )
    csv_path = tmp_path / "llh_without_altitude.csv"
    csv_path.write_text(
        "label,latitude,longitude,altitude,heading_deg\n"
        "wp1,36.000001,140.000002,,90.0\n",
        encoding="utf-8",
    )

    waypoints = parse_waypoint_csv(str(csv_path), projection=projection)
    expected = llh_to_enu(LlhPoint(36.000001, 140.000002, 25.0), projection)

    assert waypoints[0].altitude is None
    assert abs(waypoints[0].pose.position.x - expected.x) < 1.0e-6
    assert abs(waypoints[0].pose.position.y - expected.y) < 1.0e-6
    assert waypoints[0].pose.position.z == 0.0

def test_parse_waypoint_csv_keeps_enu_only_route(tmp_path: Path) -> None:
    """ENU-only CSVはprojectionなしで従来通りposeを使用する."""

    csv_path = tmp_path / "enu_only.csv"
    csv_path.write_text(
        "label,x,y,z,q1,q2,q3,q4\n"
        "wp1,1.0,2.0,3.0,0.0,0.0,0.70710678,0.70710678\n",
        encoding="utf-8",
    )

    waypoints = parse_waypoint_csv(str(csv_path))

    assert waypoints[0].pose.position.x == 1.0
    assert waypoints[0].pose.position.y == 2.0
    assert waypoints[0].pose.position.z == 3.0
    assert waypoints[0].pose.orientation.z == 0.70710678


def _tokyo_station_projection() -> ProjectionConfig:
    """params/default.yaml と同じ、東京駅を原点とする投影設定."""

    return ProjectionConfig(
        origin_latitude=35.681382,
        origin_longitude=139.766084,
        origin_altitude=3.86,
        map_yaw_offset_rad=0.0,
        projection_id="tokyo_station",
    )


def test_enu_pose_does_not_depend_on_csv_altitude(tmp_path: Path) -> None:
    """CSVのaltitude列が水平ENU座標へ影響しないことを確認する.

    local tangent planeでは同一の緯度経度でも高度が変わると水平座標がずれる
    （原点距離d[m]に対し 高度差 × d/R）。waypointごとに異なる高度で投影すると
    同じ地点が別のENU座標になり、自己位置（geo_pose_converterが origin_altitude
    基準でENU化する）との整合が崩れるため、投影基準は常に origin_altitude とする。
    """

    projection = _tokyo_station_projection()
    header = "label,latitude,longitude,altitude,heading_deg\n"

    without_alt = tmp_path / "no_alt.csv"
    without_alt.write_text(header + "wp1,36.0829271,140.0769037,,90.0\n", encoding="utf-8")
    with_alt = tmp_path / "with_alt.csv"
    with_alt.write_text(header + "wp1,36.0829271,140.0769037,62.0,90.0\n", encoding="utf-8")

    a = parse_waypoint_csv(str(without_alt), projection)[0]
    b = parse_waypoint_csv(str(with_alt), projection)[0]

    assert a.pose.position.x == b.pose.position.x
    assert a.pose.position.y == b.pose.position.y


def test_enu_pose_matches_origin_altitude_projection(tmp_path: Path) -> None:
    """生成されるENU poseが origin_altitude 基準の投影と一致することを確認する."""

    projection = _tokyo_station_projection()
    csv_path = tmp_path / "wp.csv"
    csv_path.write_text(
        "label,latitude,longitude,altitude,heading_deg\n"
        "wp1,36.0829271,140.0769037,62.0,90.0\n",
        encoding="utf-8",
    )

    wp = parse_waypoint_csv(str(csv_path), projection)[0]
    expected = llh_to_enu(
        LlhPoint(36.0829271, 140.0769037, projection.origin_altitude), projection
    )

    assert abs(wp.pose.position.x - expected.x) < 1e-9
    assert abs(wp.pose.position.y - expected.y) < 1e-9
