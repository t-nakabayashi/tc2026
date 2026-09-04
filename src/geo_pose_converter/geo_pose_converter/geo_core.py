#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""LLH/ENU 変換のROS非依存コア処理."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Mapping, Tuple

import yaml

WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


@dataclass(frozen=True)
class LlhPoint:
    """WGS84 緯度経度高度."""

    latitude: float
    longitude: float
    altitude: float = 0.0


@dataclass(frozen=True)
class EnuPoint:
    """map frame 上のENU座標."""

    x: float
    y: float
    z: float = 0.0


@dataclass(frozen=True)
class ProjectionConfig:
    """local tangent plane 変換の設定."""

    origin_latitude: float
    origin_longitude: float
    origin_altitude: float = 0.0
    map_yaw_offset_rad: float = 0.0
    projection_id: str = "default"
    datum: str = "WGS84"
    map_frame_id: str = "map"
    earth_frame_id: str = "earth"


def projection_config_from_mapping(values: Mapping[str, Any]) -> ProjectionConfig:
    """辞書からProjectionConfigを生成する."""

    return ProjectionConfig(
        origin_latitude=float(values.get("origin_latitude", 35.681382)),
        origin_longitude=float(values.get("origin_longitude", 139.766084)),
        origin_altitude=float(values.get("origin_altitude", 3.86)),
        map_yaw_offset_rad=float(values.get("map_yaw_offset_rad", 0.0)),
        projection_id=str(values.get("projection_id", "tokyo_station")),
        datum=str(values.get("datum", "WGS84")),
        map_frame_id=str(values.get("map_frame_id", "map")),
        earth_frame_id=str(values.get("earth_frame_id", "earth")),
    )


def load_projection_config_from_yaml(
    yaml_path: str,
    node_name: str | None = None,
) -> ProjectionConfig:
    """ROS 2 parameter YAMLからProjectionConfigを読み込む."""

    with open(yaml_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    params: dict[str, Any] = {}
    if isinstance(data, Mapping):
        common = data.get("/**", {})
        if isinstance(common, Mapping):
            common_params = common.get("ros__parameters", {})
            if isinstance(common_params, Mapping):
                params.update(common_params)
        if node_name:
            node_data = data.get(node_name, {})
            if isinstance(node_data, Mapping):
                node_params = node_data.get("ros__parameters", {})
                if isinstance(node_params, Mapping):
                    params.update(node_params)

    return projection_config_from_mapping(params)


def normalize_heading_deg(heading_deg: float) -> float:
    """headingを[0, 360)へ正規化する."""

    return float(heading_deg) % 360.0


def heading_deg_to_yaw_enu_rad(heading_deg: float) -> float:
    """真北基準CW heading[deg]をENU yaw[rad]へ変換する."""

    return math.pi / 2.0 - math.radians(float(heading_deg))


def yaw_enu_rad_to_heading_deg(yaw_enu_rad: float) -> float:
    """ENU yaw[rad]を真北基準CW heading[deg]へ変換する."""

    return normalize_heading_deg(math.degrees(math.pi / 2.0 - float(yaw_enu_rad)))


def yaw_to_quaternion(yaw_rad: float) -> Tuple[float, float, float, float]:
    """ENU yaw[rad]をgeometry_msgs互換の(x, y, z, w)へ変換する."""

    half = float(yaw_rad) * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """quaternionからyaw[rad]を取り出す."""

    siny_cosp = 2.0 * (float(w) * float(z) + float(x) * float(y))
    cosy_cosp = 1.0 - 2.0 * (float(y) * float(y) + float(z) * float(z))
    return math.atan2(siny_cosp, cosy_cosp)


def geodetic_to_ecef(point: LlhPoint) -> Tuple[float, float, float]:
    """WGS84 LLHをECEFへ変換する."""

    lat = math.radians(point.latitude)
    lon = math.radians(point.longitude)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    x = (n + point.altitude) * cos_lat * math.cos(lon)
    y = (n + point.altitude) * cos_lat * math.sin(lon)
    z = (n * (1.0 - WGS84_E2) + point.altitude) * sin_lat
    return x, y, z


def ecef_to_geodetic(x: float, y: float, z: float) -> LlhPoint:
    """ECEFをWGS84 LLHへ変換する."""

    b = WGS84_A * (1.0 - WGS84_F)
    ep2 = (WGS84_A * WGS84_A - b * b) / (b * b)
    p = math.hypot(x, y)
    theta = math.atan2(z * WGS84_A, p * b)
    lon = math.atan2(y, x)
    sin_theta = math.sin(theta)
    cos_theta = math.cos(theta)
    lat = math.atan2(
        z + ep2 * b * sin_theta ** 3,
        p - WGS84_E2 * WGS84_A * cos_theta ** 3,
    )
    sin_lat = math.sin(lat)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    alt = p / math.cos(lat) - n
    return LlhPoint(math.degrees(lat), math.degrees(lon), alt)


def _ecef_delta_to_enu(
    dx: float,
    dy: float,
    dz: float,
    origin: LlhPoint,
) -> Tuple[float, float, float]:
    lat = math.radians(origin.latitude)
    lon = math.radians(origin.longitude)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)
    east = -sin_lon * dx + cos_lon * dy
    north = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
    up = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz
    return east, north, up


def _enu_to_ecef_delta(
    east: float,
    north: float,
    up: float,
    origin: LlhPoint,
) -> Tuple[float, float, float]:
    lat = math.radians(origin.latitude)
    lon = math.radians(origin.longitude)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)
    dx = -sin_lon * east - sin_lat * cos_lon * north + cos_lat * cos_lon * up
    dy = cos_lon * east - sin_lat * sin_lon * north + cos_lat * sin_lon * up
    dz = cos_lat * north + sin_lat * up
    return dx, dy, dz


def _enu_to_map(east: float, north: float, yaw: float) -> Tuple[float, float]:
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    x = cos_yaw * east + sin_yaw * north
    y = -sin_yaw * east + cos_yaw * north
    return x, y


def _map_to_enu(x: float, y: float, yaw: float) -> Tuple[float, float]:
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    east = cos_yaw * x - sin_yaw * y
    north = sin_yaw * x + cos_yaw * y
    return east, north


def llh_to_enu(point: LlhPoint, projection: ProjectionConfig) -> EnuPoint:
    """WGS84 LLHをmap frameのENU座標へ変換する."""

    origin = LlhPoint(
        projection.origin_latitude,
        projection.origin_longitude,
        projection.origin_altitude,
    )
    px, py, pz = geodetic_to_ecef(point)
    ox, oy, oz = geodetic_to_ecef(origin)
    east, north, up = _ecef_delta_to_enu(px - ox, py - oy, pz - oz, origin)
    x, y = _enu_to_map(east, north, projection.map_yaw_offset_rad)
    return EnuPoint(x, y, up)


def enu_to_llh(point: EnuPoint, projection: ProjectionConfig) -> LlhPoint:
    """map frameのENU座標をWGS84 LLHへ変換する."""

    origin = LlhPoint(
        projection.origin_latitude,
        projection.origin_longitude,
        projection.origin_altitude,
    )
    east, north = _map_to_enu(point.x, point.y, projection.map_yaw_offset_rad)
    dx, dy, dz = _enu_to_ecef_delta(east, north, point.z, origin)
    ox, oy, oz = geodetic_to_ecef(origin)
    return ecef_to_geodetic(ox + dx, oy + dy, oz + dz)


def enu_to_llh_on_ground(
    x: float,
    y: float,
    projection: ProjectionConfig,
    ground_altitude: float = 0.0,
    iterations: int = 3,
) -> LlhPoint:
    """map frameの水平座標(x, y)を、指定高度の地表点としてWGS84 LLHへ変換する.

    本プロジェクトの走行用ENU poseは2D座標として扱い、zは0.0へ正規化する
    (`llh_to_enu_pose()` 参照)。一方、local tangent planeでは原点から離れるほど
    地表が接平面から下がるため(距離d[m]に対し概ね d^2/2R)、z=0.0のまま
    `enu_to_llh()` へ渡すと「原点の鉛直方向へ持ち上げた点」を逆投影することになる。
    原点の鉛直方向は遠方では地表の鉛直方向から d/R だけ傾いているため、
    水平位置に (d^2/2R) * (d/R) の誤差が出る(原点52.7km先で約1.8m)。

    そこで、指定した地表高度と整合する接平面zを反復的に求めてから逆投影し、
    水平位置の誤差を除去する。`llh_to_enu()` との往復が一致することを保証する。

    Args:
        x (float): map frameのX座標 [m]。
        y (float): map frameのY座標 [m]。
        projection (ProjectionConfig): 投影設定。
        ground_altitude (float): 対象点の楕円体高 [m]。既定は0.0。
        iterations (int): 反復回数。数kmから数十kmの範囲では3回で収束する。

    Returns:
        LlhPoint: 水平位置が(x, y)と整合するWGS84 LLH。
    """

    z = 0.0
    for _ in range(max(iterations, 1)):
        candidate = enu_to_llh(EnuPoint(x, y, z), projection)
        z = llh_to_enu(
            LlhPoint(candidate.latitude, candidate.longitude, ground_altitude), projection
        ).z
    return enu_to_llh(EnuPoint(x, y, z), projection)


def bearing_from_map_delta(dx: float, dy: float, projection: ProjectionConfig) -> float:
    """map frame差分から真北基準CW bearing[deg]を算出する."""

    east, north = _map_to_enu(dx, dy, projection.map_yaw_offset_rad)
    return normalize_heading_deg(math.degrees(math.atan2(east, north)))
