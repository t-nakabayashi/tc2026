#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
route_planner ノード（GetRoute 完了 + UpdateRoute 完了：仮想エッジ/履歴/経度キー修正対応）

主な機能と仕様:
-----------------
本ノードは、複数の固定/可変ブロックから構成されるルートを生成・配布（GetRoute）し、
走行中の経路封鎖イベントに応じて可変ブロックを再探索した新ルートを配布（UpdateRoute）する。

"""

from __future__ import annotations

import math
import os
import sys
import traceback
from pathlib import Path
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Set, Tuple

import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

import numpy as np

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Quaternion, Pose
from sensor_msgs.msg import Image

from tc_route_msgs.msg import Route, Waypoint
from tc_geo_msgs.msg import MapProjection
from tc_route_msgs.srv import GetRoute, UpdateRoute

from geo_pose_converter.geo_core import (
    ProjectionConfig,
    enu_to_llh_on_ground,
    load_projection_config_from_yaml,
)

# 可変ルート探索（外部モジュール）
_THIS_DIR = Path(__file__).resolve().parent
if str(_THIS_DIR) not in sys.path:
    sys.path.append(str(_THIS_DIR))

from .route_builder import (
    RouteBuilder,
    RouteBuildResult,
    WaypointOrigin,
    WaypointRecord,
    render_variable_route_overlay,
    resolve_path,
)

def _copy_pose(src: Pose) -> Pose:
    p = Pose()
    p.position.x = src.position.x
    p.position.y = src.position.y
    p.position.z = src.position.z
    p.orientation.x = src.orientation.x
    p.orientation.y = src.orientation.y
    p.orientation.z = src.orientation.z
    p.orientation.w = src.orientation.w
    return p


def convert_rgb_array_to_image(rgb_array: np.ndarray) -> Image:
    """RGB配列を sensor_msgs/Image に変換する。"""

    if rgb_array.ndim != 3 or rgb_array.shape[2] != 3:
        raise ValueError("RGB配列は (height, width, 3) の形状である必要があります。")

    rgb_uint8 = np.clip(rgb_array, 0, 255).astype(np.uint8)
    img = Image()
    img.header = Header()
    img.header.frame_id = "map"
    height, width, _ = rgb_uint8.shape
    img.height = int(height)
    img.width = int(width)
    img.encoding = "rgb8"
    img.step = int(width) * 3
    img.data = rgb_uint8.tobytes()
    return img


# ===== データクラス ==================================================================

@dataclass
class SegmentCacheEntry:
    """CSVから読み込んだ waypoint 配列のキャッシュ。

    Attributes:
        segment_id: CSVファイル（相対/絶対パス）を識別するID。
        waypoints: CSVから復元済みのWaypoint配列。ここでは端点ラベルの刻印等は行わない。
    """
    segment_id: str
    waypoints: List[Waypoint]


# ===== ユーティリティ関数 =============================================================

def yaw_to_quaternion(yaw: float) -> Quaternion:
    """yaw[rad] をクォータニオンに変換する（roll,pitch=0固定）。"""
    q = Quaternion()
    half = yaw * 0.5
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q


def quaternion_to_yaw(q: Quaternion) -> float:
    """Quaternionからmap frameのyaw[rad]を抽出する。"""

    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_enu_rad_to_heading_deg(yaw_enu_rad: float) -> float:
    """ENU yaw[rad]を真北0度・時計回りのheading[deg]へ変換する。"""

    return (90.0 - math.degrees(yaw_enu_rad)) % 360.0


def heading_deg_to_yaw_enu_rad(heading_deg: float) -> float:
    """真北0度・時計回りのheading[deg]をENU yaw[rad]へ変換する。"""

    return math.radians(90.0 - float(heading_deg))


def make_text_png_image(text: str = "No variable route in this plan") -> Image:
    """最小1x1の有効Imageを返す。"""
    img = Image()
    img.header = Header()
    img.header.frame_id = "map"
    img.height = 1
    img.width = 1
    img.encoding = "bgr8"
    img.step = 3
    img.data = b"\x00\x00\x00"
    return img


def almost_same_xy(ax: float, ay: float, bx: float, by: float, eps: float = 1e-6) -> bool:
    """2点のXY距離が閾値以下か判定。"""
    return math.hypot(ax - bx, ay - by) <= eps


def concat_with_dedup(base: List[Waypoint], ext: List[Waypoint], eps: float = 1e-6) -> List[Waypoint]:
    """2つの waypoint 配列を連結。境界が同一点なら ext 先頭を除外（重複排除）して結合する。"""
    if not base:
        return list(ext)
    if not ext:
        return base
    bx = base[-1].pose.position.x
    by = base[-1].pose.position.y
    ex = ext[0].pose.position.x
    ey = ext[0].pose.position.y
    if almost_same_xy(bx, by, ex, ey, eps):
        # ext[0]にラベルがありbase[-1]に無ければ移す（端点ラベルを保護）
        if ext[0].label and not base[-1].label:
            base[-1].label = ext[0].label
        return base + ext[1:]
    return base + ext


def apply_segment_fixed_flags(
    wps: List[Waypoint], origins: List[WaypointOrigin], blocks: List[Dict[str, Any]]
) -> None:
    """Waypointごとに「直前区間が固定かどうか」のフラグを設定する。"""
    if not wps:
        return
    block_type_map: Dict[Tuple[str, int], str] = {}
    for block in blocks:
        name = block.get("name")
        index = block.get("index")
        btype = block.get("type")
        if name is None or index is None:
            continue
        block_type_map[(str(name), int(index))] = str(btype or "")

    for wp in wps:
        wp.segment_is_fixed = False

    count = min(len(wps), len(origins))
    for i in range(count):
        origin = origins[i]
        wp = wps[i]
        block_name = getattr(origin, "block_name", None)
        block_index = getattr(origin, "block_index", None)
        if block_name is None or block_index is None:
            wp.segment_is_fixed = False
            continue
        try:
            block_key = (str(block_name), int(block_index))
        except (TypeError, ValueError):
            wp.segment_is_fixed = False
            continue
        block_type = block_type_map.get(block_key, "")
        is_fixed = block_type == "fixed"
        if getattr(origin, "segment_id", None) == "__virtual__":
            is_fixed = False
        wp.segment_is_fixed = bool(is_fixed)


def stamp_edge_end_labels(wps: List[Waypoint], src_label: str, dst_label: str) -> None:
    """エッジ両端の waypoint にノードラベルを刻印（境界識別・スライス用）。"""
    if not wps:
        return
    wps[0].label = src_label
    wps[-1].label = dst_label


def indexing(wps: List[Waypoint]) -> None:
    """Waypoint.index を 0..N-1 に再採番する。"""
    for i, wp in enumerate(wps):
        wp.index = i


def slice_by_labels(
    wps: List[Waypoint],
    start_label: str,
    goal_label: str,
) -> Tuple[List[Waypoint], int]:
    """start_label を前方から、goal_label を後方から検索し、その範囲でスライスする。
    start_label または goal_label が空文字/None の場合は、
    それぞれ先頭または末尾ノードを採用する。

    Returns:
        (sliced_list, start_offset) を返す。
        start_offset は元配列に対する切り出し開始位置。
    """
    start_idx: Optional[int] = None
    goal_idx: Optional[int] = None

    # --- start_label の補完または検索 ---
    if not start_label:  # 空文字や None の場合は先頭ノードから
        start_idx = 0
        print(f"[WARN][slice_by_labels] start_label is empty; using first waypoint '{wps[0].label}'")
    else:
        for i, wp in enumerate(wps):
            if wp.label == start_label:
                start_idx = i
                break

    # --- goal_label の補完または検索 ---
    if not goal_label:  # 空文字や None の場合は末尾ノードまで
        goal_idx = len(wps) - 1
        print(f"[WARN][slice_by_labels] goal_label is empty; using last waypoint '{wps[-1].label}'")
    else:
        for j in range(len(wps) - 1, -1, -1):
            if wps[j].label == goal_label:
                goal_idx = j
                break

    # --- バリデーション ---
    if start_idx is None:
        raise ValueError(f"Start label '{start_label}' not found.")
    if goal_idx is None:
        raise ValueError(f"Goal label '{goal_label}' not found.")
    if start_idx > goal_idx:
        raise ValueError("Invalid slice order: start > goal.")

    # --- スライス処理 ---
    sliced = wps[start_idx: goal_idx + 1]
    if len(sliced) <= 1:
        # start == goal で1点のみはエラー扱い（要件）
        raise ValueError("Single-point route not allowed.")

    return sliced, start_idx


def calc_total_distance(wps: List[Waypoint]) -> float:
    """総距離[m]を算出。"""
    dist = 0.0
    for i in range(1, len(wps)):
        x0, y0 = wps[i - 1].pose.position.x, wps[i - 1].pose.position.y
        x1, y1 = wps[i].pose.position.x, wps[i].pose.position.y
        dist += math.hypot(x1 - x0, y1 - y0)
    return dist


def adjust_orientations(
    wps: List[Waypoint],
    recalc_ranges: List[Tuple[int, int]],
    block_boundaries: List[int],
) -> None:
    """姿勢を再計算する範囲を限定して実施する。

    Args:
        wps: ルート全体のwaypoints（スライス後）。
        recalc_ranges: 完全再計算すべき連続区間のインデックス範囲（start_idx, end_idx）。逆走区間や仮想区間を入れる。
        block_boundaries: ブロック末尾インデックス（端点姿勢のみ補正）。
    """
    # 完全再計算区間（仮想/逆走を含む）
    for start, end in recalc_ranges:
        if start < 0 or end >= len(wps) or start >= end:
            continue
        for i in range(start, end):
            dx = wps[i+1].pose.position.x - wps[i].pose.position.x
            dy = wps[i+1].pose.position.y - wps[i].pose.position.y
            yaw = math.atan2(dy, dx)
            wps[i].pose.orientation = yaw_to_quaternion(yaw)
        dx = wps[end].pose.position.x - wps[end-1].pose.position.x
        dy = wps[end].pose.position.y - wps[end-1].pose.position.y
        yaw = math.atan2(dy, dx)
        wps[end].pose.orientation = yaw_to_quaternion(yaw)

    # ブロック末尾waypoint（端点のみ）
    for idx in block_boundaries:
        if 0 < idx < len(wps):
            dx = wps[idx].pose.position.x - wps[idx-1].pose.position.x
            dy = wps[idx].pose.position.y - wps[idx-1].pose.position.y
            yaw = math.atan2(dy, dx)
            wps[idx].pose.orientation = yaw_to_quaternion(yaw)

    # ルート全体の末尾（端点のみ）
    if len(wps) > 1:
        dx = wps[-1].pose.position.x - wps[-2].pose.position.x
        dy = wps[-1].pose.position.y - wps[-2].pose.position.y
        yaw = math.atan2(dy, dx)
        wps[-1].pose.orientation = yaw_to_quaternion(yaw)


def pack_route_msg(
    wps: List[Waypoint],
    version: int,
    total_distance: float,
    route_image: Optional[Image],
) -> Route:
    """Route.msg を組み立てる。必要に応じて route_image はテキストPNGプレースホルダを入れる。"""
    msg = Route()
    msg.header = Header()
    msg.header.frame_id = "map"
    msg.waypoints = wps
    msg.version = int(version)
    msg.total_distance = float(total_distance)
    msg.route_image = route_image if route_image is not None else make_text_png_image()
    return msg


# ===== RoutePlanner ノード ============================================================

class RoutePlannerNode(Node):
    """ルート配布サーバ（GetRoute 完了 + UpdateRoute 完了：仮想エッジ/履歴対応）。"""

    def __init__(self) -> None:
        super().__init__("route_planner")

        # --- 起動時パラメータ ---
        self.declare_parameter("config_yaml_path", "routes/config.yaml")
        self.declare_parameter("csv_base_dir", "routes")
        self.declare_parameter("map_image_path", None)
        self.declare_parameter("map_worldfile_path", None)
        self.declare_parameter("route_id", "default_route")
        self.declare_parameter("projection_config_path", "params/default.yaml")

        try:
            pkg_share = get_package_share_directory("route_planner")
        except PackageNotFoundError:
            pkg_share = None
            self.get_logger().warn("パッケージ共有ディレクトリが取得できませんでした。相対パスはそのまま扱います。")

        try:
            geo_pkg_share = get_package_share_directory("geo_pose_converter")
        except PackageNotFoundError:
            geo_pkg_share = None
            self.get_logger().warn(
                "geo_pose_converter の共有ディレクトリが取得できませんでした。"
                "projection_config_path はそのまま扱います。"
            )

        config_yaml_raw = str(self.get_parameter("config_yaml_path").value)
        csv_base_dir_raw = str(self.get_parameter("csv_base_dir").value)
        map_image_raw = self.get_parameter("map_image_path").value
        map_worldfile_raw = self.get_parameter("map_worldfile_path").value
        self.route_id = str(self.get_parameter("route_id").value)
        projection_config_raw = str(self.get_parameter("projection_config_path").value)
        get_service_name = 'get_route'
        update_service_name = 'update_route'

        self.config_yaml_path: str = resolve_path(pkg_share, config_yaml_raw) if config_yaml_raw else ""
        self.csv_base_dir: str = resolve_path(pkg_share, csv_base_dir_raw) if csv_base_dir_raw else ""
        self.map_image_path: Optional[str] = self._resolve_map_resource(
            map_image_raw,
            pkg_share,
            "map_image_path",
        )
        self.map_worldfile_path: Optional[str] = self._resolve_map_resource(
            map_worldfile_raw,
            pkg_share,
            "map_worldfile_path",
        )
        self.projection_config_path: str = resolve_path(
            geo_pkg_share,
            projection_config_raw,
        ) if projection_config_raw else ""
        try:
            self.projection_config: ProjectionConfig = load_projection_config_from_yaml(
                self.projection_config_path
            )
        except Exception as exc:
            self.get_logger().error(f"projection設定の読み込みに失敗しました: {exc}")
            self.projection_config = ProjectionConfig(35.681382, 139.766084, 3.86)


        if self.map_image_path and self.map_worldfile_path:
            self.get_logger().info(f"Using map_image_path: {self.map_image_path}")
            self.get_logger().info(f"Using map_worldfile_path: {self.map_worldfile_path}")
        elif self.map_image_path or self.map_worldfile_path:
            self.get_logger().warn(
                "map_image_path と map_worldfile_path の片方しか解決できませんでした。地図描画は無効化されます。"
            )
            self.map_image_path = None
            self.map_worldfile_path = None
        else:
            self.get_logger().info("可変ブロック描画用の地図リソースは指定されていません。")

        if not self.config_yaml_path:
            self.get_logger().error("config_yaml_path is required.")
        else:
            self.get_logger().info(f"Using config_yaml_path: {self.config_yaml_path}")
        if self.csv_base_dir:
            self.get_logger().info(f"Using csv_base_dir: {self.csv_base_dir}")
        if self.projection_config_path:
            self.get_logger().info(f"Using projection_config_path: {self.projection_config_path}")
        self.get_logger().info(
            "Using projection: "
            f"{self.projection_config.projection_id} "
            f"origin=({self.projection_config.origin_latitude}, "
            f"{self.projection_config.origin_longitude}, "
            f"{self.projection_config.origin_altitude})"
        )

        # --- メンバ（状態） ---
        self.blocks: List[Dict[str, Any]] = []                  # YAMLのブロック原義（固定/可変）
        self.segments: Dict[str, SegmentCacheEntry] = {}        # segment_id -> キャッシュ済みwaypoints
        self.current_route: Optional[Route] = None
        self.current_route_origins: List[WaypointOrigin] = []   # current_route.waypoints と同長
        self.route_version: int = 0                             # Route.msgのversion(int32)に対応
        self.last_request_checkpoints: Set[str] = set()         # GetRoute時に追加されたチェックポイント
        self.visited_checkpoints_hist: Dict[str, Set[str]] = {} # ブロック名 -> 訪問済みチェックポイント（永続）

        # --- ルート定義のロード ---
        self.route_builder = RouteBuilder(
            config_yaml_path=self.config_yaml_path,
            csv_base_dir=self.csv_base_dir,
            logger=self.get_logger(),
            projection=self.projection_config,
        )
        try:
            self.route_builder.load()
            self.csv_base_dir = self.route_builder.csv_base_dir
            self.blocks = list(self.route_builder.blocks)
            self._refresh_segment_cache()
        except Exception as exc:
            self.get_logger().error(f"ルート設定の読み込みに失敗しました: {exc}")
            self.blocks = []
            self.segments = {}

        # --- サービス登録（逐次処理: MutuallyExclusive） ---
        cb_group = MutuallyExclusiveCallbackGroup()
        self._srv_get = self.create_service(
            GetRoute, get_service_name, self.handle_get_route, callback_group=cb_group
        )
        self._srv_update = self.create_service(
            UpdateRoute, update_service_name, self.handle_update_route, callback_group=cb_group
        )

        self.get_service_name = self._resolve_service_name(get_service_name)
        self.update_service_name = self._resolve_service_name(update_service_name)

        self.get_logger().info("route_planner is ready.")

    def _resolve_service_name(self, name: str) -> str:
        """リマップ後のサービス名を取得する。"""
        try:
            return self.resolve_service_name(name)
        except AttributeError:
            return name

    def _resolve_map_resource(
        self,
        raw_value: Optional[Any],
        pkg_share: Optional[str],
        param_name: str,
    ) -> Optional[str]:
        """地図関連パラメータをパッケージ相対パスから絶対パスへ解決する。"""

        if raw_value is None:
            return None
        value = str(raw_value).strip()
        if not value:
            return None

        candidates = []
        if os.path.isabs(value):
            candidates.append(os.path.normpath(value))
        if pkg_share:
            candidates.append(resolve_path(pkg_share, value))
        package_root = str(_THIS_DIR.parent)
        candidates.append(resolve_path(package_root, value))

        for candidate in candidates:
            if candidate and os.path.exists(candidate):
                return candidate

        self.get_logger().warn(f"{param_name} で指定されたファイルが見つかりません: {value}")
        return None

    def _render_variable_route_image(
        self,
        nodes: Dict[str, Tuple[float, float]],
        solver_result: Dict[str, Any],
    ) -> Optional[Image]:
        """solve_variable_routeの結果を用いて地図画像を生成する。"""

        if not (self.map_image_path and self.map_worldfile_path):
            self.get_logger().warn(
                "可変ブロックが存在しますが map_image_path と map_worldfile_path が設定されていないため、ダミー画像を返却します。"
            )
            return None

        rgb_array = render_variable_route_overlay(
            nodes,
            solver_result,
            self.map_image_path,
            self.map_worldfile_path,
            logger=self.get_logger(),
        )
        if rgb_array is None:
            return None

        try:
            return convert_rgb_array_to_image(rgb_array)
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().warn(f"地図描画のメッセージ変換に失敗しました: {exc}")
            return None

    def _record_to_waypoint(self, record: WaypointRecord) -> Waypoint:
        """WaypointRecordをRoute用のWaypointメッセージへ変換する。"""

        wp = Waypoint()
        wp.label = record.label
        wp.index = int(record.index)
        wp.has_pose_enu = True
        wp.pose.position.x = float(record.pose.position.x)
        wp.pose.position.y = float(record.pose.position.y)
        wp.pose.position.z = float(record.pose.position.z)
        wp.pose.orientation.x = float(record.pose.orientation.x)
        wp.pose.orientation.y = float(record.pose.orientation.y)
        wp.pose.orientation.z = float(record.pose.orientation.z)
        wp.pose.orientation.w = float(record.pose.orientation.w)
        wp.right_open = float(record.right_open)
        wp.left_open = float(record.left_open)
        wp.line_stop = bool(record.line_stop)
        wp.signal_stop = bool(record.signal_stop)
        wp.not_skip = bool(record.not_skip)
        if hasattr(wp, "segment_is_fixed"):
            wp.segment_is_fixed = bool(record.segment_is_fixed)

        if record.latitude is not None and record.longitude is not None:
            wp.has_geo_pose = True
            wp.geo_pose.header = Header()
            wp.geo_pose.header.stamp = self.get_clock().now().to_msg()
            wp.geo_pose.header.frame_id = self.projection_config.earth_frame_id
            wp.geo_pose.child_frame_id = "route_waypoint"
            wp.geo_pose.point.latitude = float(record.latitude)
            wp.geo_pose.point.longitude = float(record.longitude)
            wp.geo_pose.point.altitude = float(record.altitude) if record.altitude is not None else 0.0
            wp.geo_pose.point.has_altitude = record.altitude is not None

            yaw_map = quaternion_to_yaw(wp.pose.orientation)
            if record.heading_deg is not None:
                heading_deg = float(record.heading_deg) % 360.0
                yaw_enu_rad = heading_deg_to_yaw_enu_rad(heading_deg)
            else:
                yaw_enu_rad = yaw_map + self.projection_config.map_yaw_offset_rad
                heading_deg = yaw_enu_rad_to_heading_deg(yaw_enu_rad)
            wp.geo_pose.heading_deg = heading_deg
            wp.geo_pose.has_heading = True
            wp.geo_pose.yaw_enu_rad = yaw_enu_rad
            wp.geo_pose.has_yaw_enu = True
            wp.geo_pose_source = Waypoint.GEO_SOURCE_ROUTE_FILE
        else:
            self._fill_geo_pose_from_enu(wp)
        return wp

    def _fill_geo_pose_from_enu(self, wp: Waypoint) -> None:
        """LLHを持たないwaypointのgeo_poseを、ENU poseからの逆投影で補完する。

        ENU座標のみで定義されたroute（`routes/simulation` 配下など、CSVに
        latitude/longitude列が無いもの）でも、地図表示・ログ・route編集がLLHを
        参照できるようにする。走行制御が使う `pose`（ENU）は変更しないため、
        追従挙動には影響しない。

        水平座標の逆投影は `enu_to_llh_on_ground()` を用い、基準高度を
        `origin_altitude` に揃える（ENU⇔LLH変換の高度規約統一。詳細は
        `geo_pose_converter/message_utils.py` の各変換関数を参照）。
        """

        if self.projection_config is None:
            wp.has_geo_pose = False
            wp.geo_pose_source = Waypoint.GEO_SOURCE_UNKNOWN
            return

        point = enu_to_llh_on_ground(
            float(wp.pose.position.x),
            float(wp.pose.position.y),
            self.projection_config,
            ground_altitude=self.projection_config.origin_altitude,
        )
        wp.has_geo_pose = True
        wp.geo_pose.header = Header()
        wp.geo_pose.header.stamp = self.get_clock().now().to_msg()
        wp.geo_pose.header.frame_id = self.projection_config.earth_frame_id
        wp.geo_pose.child_frame_id = "route_waypoint"
        wp.geo_pose.point.latitude = point.latitude
        wp.geo_pose.point.longitude = point.longitude
        wp.geo_pose.point.altitude = 0.0
        # ENUのzは走行用2D座標として0に正規化されており高度情報を持たないため、
        # 逆投影した高度は有効値として扱わない。
        wp.geo_pose.point.has_altitude = False

        yaw_enu_rad = quaternion_to_yaw(wp.pose.orientation) + (
            self.projection_config.map_yaw_offset_rad
        )
        wp.geo_pose.heading_deg = yaw_enu_rad_to_heading_deg(yaw_enu_rad)
        wp.geo_pose.has_heading = True
        wp.geo_pose.yaw_enu_rad = yaw_enu_rad
        wp.geo_pose.has_yaw_enu = True
        wp.geo_pose_source = Waypoint.GEO_SOURCE_PROJECTED_FROM_ENU

    def _make_projection_msg(self) -> MapProjection:
        """Route.msgへ埋め込む地図投影メタデータを生成する。"""

        projection = MapProjection()
        projection.header = Header()
        projection.header.stamp = self.get_clock().now().to_msg()
        projection.header.frame_id = self.projection_config.earth_frame_id
        projection.projection_type = MapProjection.PROJECTION_LOCAL_TANGENT_PLANE
        projection.projection_id = self.projection_config.projection_id
        projection.datum = self.projection_config.datum
        projection.map_frame_id = self.projection_config.map_frame_id
        projection.earth_frame_id = self.projection_config.earth_frame_id
        projection.origin_latitude = self.projection_config.origin_latitude
        projection.origin_longitude = self.projection_config.origin_longitude
        projection.origin_altitude = self.projection_config.origin_altitude
        projection.map_yaw_offset_rad = self.projection_config.map_yaw_offset_rad
        projection.utm_zone = ""
        projection.utm_north = True
        return projection

    def _apply_route_metadata(self, route: Route) -> None:
        """Route全体の識別子・座標系メタデータを設定する。"""

        route.header.frame_id = self.projection_config.map_frame_id
        route.route_id = self.route_id
        route.map_frame_id = self.projection_config.map_frame_id
        route.earth_frame_id = self.projection_config.earth_frame_id
        route.projection = self._make_projection_msg()
        route.route_image.header.frame_id = self.projection_config.map_frame_id

    def _refresh_segment_cache(self) -> None:
        """RouteBuilderのキャッシュからROSメッセージ形式のセグメントを再構築する。"""

        self.segments = {}
        for seg_id, entry in self.route_builder.segments.items():
            waypoints = [self._record_to_waypoint(wp) for wp in entry.waypoints]
            self.segments[seg_id] = SegmentCacheEntry(seg_id, waypoints)

    # ===== GetRoute / UpdateRoute =====================================================

    def handle_get_route(self, request: GetRoute.Request, response: GetRoute.Response) -> GetRoute.Response:
        """初期ルートの生成と配布。Update 用の由来メタも同時に構築して保存する。"""
        try:
            # --- サービスリクエストのフィールド名を設計書仕様に統一 ---
            start_label = getattr(request, "start_label", None)
            goal_label = getattr(request, "goal_label", None)
            checkpoint_labels = getattr(request, "checkpoint_labels", [])
            self.get_logger().info(
                f"Get route: start_label='{start_label}', goal_label='{goal_label}', checkpoints='{checkpoint_labels}'"
            )
            if not self.route_builder.blocks:
                raise RuntimeError("No blocks configuration loaded.")

            result: RouteBuildResult = self.route_builder.build_route(
                start_label=str(start_label or ""),
                goal_label=str(goal_label or ""),
                checkpoint_labels=list(checkpoint_labels),
            )
            # builder側でブロック情報が更新されている可能性があるため反映しておく。
            self.blocks = list(self.route_builder.blocks)

            route_wps: List[Waypoint] = [
                self._record_to_waypoint(record) for record in result.waypoints
            ]
            origins: List[WaypointOrigin] = list(result.origins)
            total_distance = result.total_distance

            route_image: Optional[Image] = None
            if result.has_variable_block:
                solver_info = result.solver_info
                if solver_info is not None:
                    img = self._render_variable_route_image(
                        solver_info.nodes, solver_info.solver_result
                    )
                    if img is not None:
                        route_image = img
                if route_image is None:
                    if self.map_image_path and self.map_worldfile_path:
                        self.get_logger().warn(
                            "可変ブロックの地図描画に失敗したため、ダミー画像を返却します。"
                        )
                    route_image = make_text_png_image()
            else:
                route_image = make_text_png_image()

            self.route_version = 1
            route = pack_route_msg(route_wps, self.route_version, total_distance, route_image)
            self._apply_route_metadata(route)

            self.current_route = route
            self.current_route_origins = origins
            self.last_request_checkpoints = set(checkpoint_labels)
            self.visited_checkpoints_hist.clear()  # 初期ルート生成時に訪問履歴をリセット

            response.success = True
            response.message = ""
            response.route = route
            return response

        except Exception as e:
            self.get_logger().error(f"[GetRoute] {e}\n{traceback.format_exc()}")
            response.success = False
            response.message = str(e)
            response.route = Route()
            return response

    def handle_update_route(self, request: UpdateRoute.Request, response: UpdateRoute.Response) -> UpdateRoute.Response:
        """経路封鎖リルートの実行。仮想エッジ（current→prev→U）を先頭に挿入し、その後に新可変ルートと後続ブロックを連結する。"""
        try:
            # 0) 前提検証
            if self.current_route is None or not self.current_route.waypoints:
                response.success = False
                response.message = "No current route in server."
                response.route = Route()
                return response
            if request.route_version != self.current_route.version:
                self.get_logger().error("[UpdateRoute] Route version mismatch.")
                response.success = False
                response.message = "Route version mismatch."
                response.route = self.current_route or Route()
                return response

            wps = self.current_route.waypoints
            origins = self.current_route_origins
            n = len(wps)

            # 1) prev/current の正当性検証（現ルート上で隣接）
            if not (0 <= request.prev_index < n - 1):
                raise RuntimeError("Invalid prev_index.")
            if request.current_index != request.prev_index + 1:
                raise RuntimeError("prev/current must be adjacent (current_index = prev_index + 1).")
            if wps[request.prev_index].label != request.prev_wp_label:
                raise RuntimeError("prev_wp_label mismatch current route.")
            if not (0 <= request.current_index < n):
                raise RuntimeError("Invalid current_index.")
            if wps[request.current_index].label != request.current_wp_label:
                raise RuntimeError("current_wp_label mismatch current route.")

            prev_origin = origins[request.prev_index]
            current_origin = origins[request.current_index]

            # 2) 対象ブロックを特定（同一ブロックである必要あり）
            if prev_origin.block_name != current_origin.block_name:
                raise RuntimeError("prev/current belong to different blocks.")
            block_name = prev_origin.block_name
            block_idx = prev_origin.block_index

            # 対象ブロックを取得
            block = None
            for b in self.blocks:
                if b["name"] == block_name and b["index"] == block_idx:
                    block = b
                    break
            if block is None:
                raise RuntimeError(f"Block not found: {block_name}")
            if block["type"] == "fixed":
                # 固定ブロックはリルート不可（封鎖検知時はエラー返却。ただしノードは継続）
                self.get_logger().warn("Closure in fixed block (not reroutable).")
                response.success = False
                response.message = "Closure in fixed block (not reroutable)."
                response.route = self.current_route or Route()
                return response

            # 3) 現在走行中エッジの (U,V) と、prev のエッジ内位置を特定
            u_node = prev_origin.edge_u
            v_node = prev_origin.edge_v
            seg_id_running = prev_origin.segment_id
            u_first_flag = prev_origin.u_first
            local_idx_prev = prev_origin.index_in_edge
            if not (u_node and v_node and seg_id_running is not None and u_first_flag is not None and local_idx_prev is not None):
                # 可変ブロックなのに必要メタが欠けるのは異常
                raise RuntimeError("Failed to identify running edge metadata for closure.")

            # u_first_flag はCSVの向きとルート走行向きの一致可否を示すメタデータであり、
            # 走行中エッジの手前側ノード（U点）がどちらかを判別する指標にはならない。
            # GetRoute 時に記録した edge_u は常に "prev 側の端点" を指すため、
            # UpdateRoute では無条件に edge_u をリルート起点として採用する必要がある。
            # そのため u_first_flag による条件分岐を廃し、常に U 点から再探索を開始する。
            connection_node = u_node
            if not connection_node:
                raise RuntimeError("Failed to determine reroute anchor node.")
            connection_label = str(connection_node)

            removed = self.route_builder.remove_graph_edge(block_name, block_idx, u_node, v_node)
            if not removed:
                self.get_logger().warn(
                    f"封鎖対象エッジが見つかりませんでした: {u_node}-{v_node}"
                )

            goal = block.get("goal")
            if not goal:
                raise RuntimeError("Block goal not set.")

            # 5) 未通過チェックポイント集合（永続履歴を考慮）
            yaml_cps = set(block.get("checkpoints", []))
            req_cps = set(self.last_request_checkpoints)
            node_ids = {nd["id"] for nd in (block.get("nodes") or [])}
            required = {c for c in (yaml_cps | req_cps) if c in node_ids}

            hist = self.visited_checkpoints_hist.setdefault(block_name, set())
            visited_now: Set[str] = set()
            for i in range(0, request.prev_index + 1):
                lab = wps[i].label
                if lab in required:
                    visited_now.add(lab)
            hist |= visited_now

            remaining_cps_block = list(required - hist)
            if not remaining_cps_block:
                remaining_cps_block = [str(goal)]

            remaining_global_cps: List[str] = [
                cp for cp in self.last_request_checkpoints if cp not in hist
            ]

            # 7-1) 先頭: current（今回の現在位置）
            new_wps: List[Waypoint] = []
            new_origins: List[WaypointOrigin] = []
            new_wps.append(self._make_waypoint_from_pose(request.current_pose, label="current"))
            new_origins.append(
                WaypointOrigin(
                    block_name=block_name,
                    block_index=block_idx,
                    segment_id="__virtual__",
                    edge_u=u_node,
                    edge_v=v_node,
                    index_in_edge=None,
                    u_first=True,
                )
            )

            # 7-2) 仮想エッジ: current→prev→U の waypoint 群を生成して連結
            prev_wp_label = str(wps[request.prev_index].label or "")
            if not prev_wp_label:
                raise RuntimeError("Prev waypoint label is empty.")

            virtual_wps, virtual_indices = self._make_virtual_edge_waypoints(
                seg_id=seg_id_running,
                u_label=u_node,
                local_idx_prev=local_idx_prev,
                u_first_on_route=u_first_flag,
                prev_wp=wps[request.prev_index],
                current_pose=request.current_pose,
            )
            # 端点ラベル（Uラベル）を刻印（virtual_wps の末尾は U になる）
            if virtual_wps:
                stamp_edge_end_labels(
                    virtual_wps, src_label="current", dst_label=connection_label
                )

            # 追加（重複境界は concat 内で解消される）
            start_idx_virtual = len(new_wps)
            base_tail = new_wps[-1]
            head_virtual = virtual_wps[0] if virtual_wps else None
            deduped = False
            if head_virtual is not None:
                deduped = almost_same_xy(
                    base_tail.pose.position.x,
                    base_tail.pose.position.y,
                    head_virtual.pose.position.x,
                    head_virtual.pose.position.y,
                )
            new_wps = concat_with_dedup(new_wps, virtual_wps)
            appended_count = len(new_wps) - start_idx_virtual
            if deduped:
                indices_for_origins = virtual_indices[1:]
            else:
                indices_for_origins = virtual_indices
            if appended_count != len(indices_for_origins):
                raise RuntimeError("Virtual edge concatenation length mismatch.")
            for seg_index in indices_for_origins:
                new_origins.append(
                    WaypointOrigin(
                        block_name=block_name,
                        block_index=block_idx,
                        segment_id="__virtual__",
                        edge_u=u_node,
                        edge_v=v_node,
                        index_in_edge=seg_index,
                        u_first=True,
                    )
                )

            # 8) RouteBuilder を用いた再構築。封鎖と未通過チェックポイントを一時的に反映する。
            builder_block: Optional[Dict[str, Any]] = None
            for b in self.route_builder.blocks:
                if b.get("name") == block_name and b.get("index") == block_idx:
                    builder_block = b
                    break
            if builder_block is None:
                raise RuntimeError(f"RouteBuilder block not found: {block_name}")

            original_checkpoints = list(builder_block.get("checkpoints") or [])
            builder_block["checkpoints"] = list(remaining_cps_block)
            original_start = builder_block.get("start")
            builder_block["start"] = connection_label

            goal_label_total = wps[-1].label if wps else ""
            if not goal_label_total:
                goal_label_total = str(goal)

            try:
                rebuild_result: RouteBuildResult = self.route_builder.build_route(
                    start_label=connection_label,
                    goal_label=str(goal_label_total),
                    checkpoint_labels=list(remaining_global_cps),
                    start_label_origin=(block_name, block_idx),
                )
            finally:
                builder_block["checkpoints"] = original_checkpoints
                if original_start is None:
                    builder_block.pop("start", None)
                else:
                    builder_block["start"] = original_start

            rebuilt_wps: List[Waypoint] = [
                self._record_to_waypoint(record) for record in rebuild_result.waypoints
            ]
            rebuilt_origins: List[WaypointOrigin] = list(rebuild_result.origins)

            new_wps, new_origins = self._concat_waypoints_with_origins(
                new_wps,
                new_origins,
                rebuilt_wps,
                rebuilt_origins,
            )

            recalc_ranges = self._collect_recalc_ranges(new_origins)
            block_tail_indices = self._collect_block_tail_indices(new_origins)

            adjust_orientations(new_wps, recalc_ranges, block_tail_indices)
            apply_segment_fixed_flags(new_wps, new_origins, self.blocks)
            indexing(new_wps)
            total_distance = calc_total_distance(new_wps)

            solver_info = rebuild_result.solver_info
            if solver_info is not None:
                route_image = self._render_variable_route_image(
                    solver_info.nodes, solver_info.solver_result
                )
            else:
                route_image = None
            if route_image is None:
                if self.map_image_path and self.map_worldfile_path:
                    self.get_logger().warn(
                        "可変ブロックの地図描画に失敗したため、ダミー画像を返却します。"
                    )
                route_image = make_text_png_image()
            self.route_version += 1
            new_route = pack_route_msg(new_wps, self.route_version, total_distance, route_image)
            self._apply_route_metadata(new_route)

            # 状態更新
            self.current_route = new_route
            self.current_route_origins = new_origins
            # visited 履歴は維持（本ブロックのキーに対して既に更新済み）

            response.success = True
            response.message = ""
            response.route = new_route
            return response

        except Exception as e:
            self.get_logger().error(f"[UpdateRoute] {e}\n{traceback.format_exc()}")
            response.success = False
            response.message = str(e)
            response.route = Route()
            return response

    # ===== 内部補助 =====================================================

    def _make_waypoint_from_pose(self, pose_stamped: PoseStamped, label: str) -> Waypoint:
        """PoseStamped から Waypoint を作る簡易ヘルパ。index は後段で採番し直す。"""

        wp = Waypoint()
        wp.label = label
        wp.index = 0
        wp.pose = _copy_pose(pose_stamped.pose)
        if hasattr(wp, "segment_is_fixed"):
            wp.segment_is_fixed = False
        return wp

    def _concat_waypoints_with_origins(
        self,
        base_wps: List[Waypoint],
        base_origins: List[WaypointOrigin],
        ext_wps: List[Waypoint],
        ext_origins: List[WaypointOrigin],
    ) -> Tuple[List[Waypoint], List[WaypointOrigin]]:
        """Waypoint列と由来情報を重複除去しながら連結する。"""

        if not base_wps:
            return list(ext_wps), list(ext_origins)
        if not ext_wps:
            return list(base_wps), list(base_origins)

        combined_wps = list(base_wps)
        combined_origins = list(base_origins)

        bx = combined_wps[-1].pose.position.x
        by = combined_wps[-1].pose.position.y
        ex = ext_wps[0].pose.position.x
        ey = ext_wps[0].pose.position.y

        if almost_same_xy(bx, by, ex, ey):
            if ext_wps[0].label and not combined_wps[-1].label:
                combined_wps[-1].label = ext_wps[0].label
            combined_wps.extend(ext_wps[1:])
            combined_origins.extend(ext_origins[1:])
        else:
            combined_wps.extend(ext_wps)
            combined_origins.extend(ext_origins)

        return combined_wps, combined_origins

    @staticmethod
    def _needs_orientation_recalc(
        segment_id: Optional[str], u_first: Optional[bool]
    ) -> bool:
        """姿勢再計算が必要な区間かを判定する。"""

        if segment_id == "__virtual__":
            return True
        if segment_id is None:
            return False
        return u_first is False

    def _collect_recalc_ranges(
        self, origins: List[WaypointOrigin]
    ) -> List[Tuple[int, int]]:
        """由来情報から姿勢再計算が必要なインデックス範囲を抽出する。"""

        ranges: List[Tuple[int, int]] = []
        if not origins:
            return ranges

        seg_start = 0
        current_seg = origins[0].segment_id
        current_u_first = origins[0].u_first

        for idx in range(1, len(origins)):
            origin = origins[idx]
            if origin.segment_id == current_seg:
                continue
            if self._needs_orientation_recalc(current_seg, current_u_first):
                end = idx - 1
                if end > seg_start:
                    ranges.append((seg_start, end))
            seg_start = idx
            current_seg = origin.segment_id
            current_u_first = origin.u_first

        if self._needs_orientation_recalc(current_seg, current_u_first):
            end = len(origins) - 1
            if end > seg_start:
                ranges.append((seg_start, end))

        return ranges

    @staticmethod
    def _collect_block_tail_indices(origins: List[WaypointOrigin]) -> List[int]:
        """ブロック境界となる末尾インデックスを抽出する。"""

        if not origins:
            return []

        tails: Set[int] = set()
        for idx in range(len(origins) - 1):
            current = origins[idx]
            nxt = origins[idx + 1]
            if (
                current.block_name != nxt.block_name
                or current.block_index != nxt.block_index
            ):
                tails.add(idx)
        tails.add(len(origins) - 1)
        return sorted(tails)



    def _make_virtual_edge_waypoints(
        self,
        seg_id: str,
        u_label: str,
        local_idx_prev: int,
        u_first_on_route: bool,
        prev_wp: Waypoint,
        current_pose: PoseStamped,
    ) -> Tuple[List[Waypoint], List[Optional[int]]]:
        """仮想エッジ current→prev→U の waypoint 群を生成する。

        ロジック:
          - 走行中エッジ（seg_id）の CSV を取得し、端点ラベルを用いて U 点が先頭になるように向きを整える。
          - current（Pose）を先頭に、prev（既存Waypoint）を挟み、prev→U 方向に向かう配列を切り出す。
            * prev が既に U（= index 0）なら、[current, U] の最短列になる（処理は同一）。
          - 生成した配列は、後段で stamp_edge_end_labels("current", U) で端点ラベルを刻む。

        Args:
            seg_id: 走行中エッジの segment_id（CSVを特定）。
            u_label: 手前側ノードのラベル（U）。
            local_idx_prev: prev のエッジ内ローカルindex（GetRoute時の向き基準）。
            u_first_on_route: GetRoute時に推定したエッジ向きのメタデータ（整合性確認用に保持）。
            prev_wp: 現ルート上の prev Waypoint（座標/姿勢を持つ）。
            current_pose: 現在位置の PoseStamped。

        Returns:
            仮想エッジ（current→prev→U）に相当する Waypoint 配列と、
            各 Waypoint が元セグメント内で対応するindex（currentはNone）。
        """
        entry = self.segments.get(seg_id)
        if entry is None:
            raise RuntimeError(f"Virtual edge source segment not loaded: {seg_id}")

        base = entry.waypoints  # CSV由来の素の配列（端点ラベルは未刻印）
        if not base:
            raise RuntimeError("Virtual edge source segment is empty.")

        u_label_str = str(u_label)

        def _find_label_index(label: str) -> Optional[int]:
            for idx, wp in enumerate(base):
                if str(wp.label or "") == label:
                    return idx
            return None

        u_index = _find_label_index(u_label_str)
        if u_index is None:
            raise RuntimeError(
                "Failed to locate U endpoint in virtual edge source segment."
            )

        if u_index == 0:
            seg_u_first = list(base)
        elif u_index == len(base) - 1:
            seg_u_first = list(reversed(base))
        else:
            raise RuntimeError(
                "U endpoint was found inside the segment, which is unsupported."
            )

        seg_len = len(seg_u_first)
        if seg_len == 0:
            raise RuntimeError("Virtual edge segment has no waypoints.")

        if u_first_on_route:
            prev_idx_u_first = int(local_idx_prev)
        else:
            prev_idx_u_first = (seg_len - 1) - int(local_idx_prev)

        if not (0 <= prev_idx_u_first < seg_len):
            raise RuntimeError("Invalid prev index in virtual edge generation.")

        prev_x = prev_wp.pose.position.x
        prev_y = prev_wp.pose.position.y
        candidate_prev = seg_u_first[prev_idx_u_first]
        if not almost_same_xy(
            prev_x,
            prev_y,
            candidate_prev.pose.position.x,
            candidate_prev.pose.position.y,
        ):
            fallback_idx: Optional[int] = None
            for idx, wp in enumerate(seg_u_first):
                if almost_same_xy(prev_x, prev_y, wp.pose.position.x, wp.pose.position.y):
                    fallback_idx = idx
                    break
            if fallback_idx is None:
                raise RuntimeError(
                    "Failed to locate prev waypoint within the virtual edge segment."
                )
            prev_idx_u_first = fallback_idx

        # Uは seg_u_first[0] に対応する（末尾は V）。
        # prev_idx_u_first から 1 ずつデクリメントし、index 0 の U まで辿って連結する。
        virtual: List[Waypoint] = []
        indices: List[Optional[int]] = []
        # current を先頭に追加（Waypoint化）
        current_wp = self._make_waypoint_from_pose(current_pose, label="current")
        virtual.append(current_wp)
        indices.append(None)

        # prev を続けて追加（既存prevをコピーして先頭重複回避。位置はそのまま）
        prev_copy = Waypoint()
        prev_copy.label = prev_wp.label  # prev は中間なのでlabelは通常空/任意だが、そのまま踏襲
        prev_copy.index = 0
        prev_copy.pose = _copy_pose(prev_wp.pose)
        if hasattr(prev_copy, "right_open"):
            prev_copy.right_open = float(
                getattr(prev_wp, "right_open", getattr(prev_wp, "right_is_open", 0.0))
            )
        if hasattr(prev_copy, "left_open"):
            prev_copy.left_open = float(
                getattr(prev_wp, "left_open", getattr(prev_wp, "left_is_open", 0.0))
            )
        if hasattr(prev_copy, "line_stop"):
            prev_copy.line_stop = bool(
                getattr(prev_wp, "line_stop", getattr(prev_wp, "line_is_stop", False))
            )
        if hasattr(prev_copy, "signal_stop"):
            prev_copy.signal_stop = bool(
                getattr(prev_wp, "signal_stop", getattr(prev_wp, "signal_is_stop", False))
            )
        if hasattr(prev_copy, "not_skip"):
            prev_copy.not_skip = bool(
                getattr(prev_wp, "not_skip", getattr(prev_wp, "isnot_skipnum", False))
            )
        virtual.append(prev_copy)
        indices.append(int(prev_idx_u_first))

        # prev_idx_u_first から 1 ずつデクリメントして U=0 まで辿る
        # prev が U と同一（prev_idx_u_first==0）のときは追加なし（[current, prev(=U)]のみ）
        for idx in range(prev_idx_u_first - 1, -1, -1):
            src = seg_u_first[idx]
            # コピーして追加（オリジナルを汚さない）
            wp = Waypoint()
            wp.label = src.label
            wp.index = 0
            wp.pose = _copy_pose(src.pose)
            if hasattr(wp, "right_open"):
                wp.right_open = float(
                    getattr(src, "right_open", getattr(src, "right_is_open", 0.0))
                )
            if hasattr(wp, "left_open"):
                wp.left_open = float(
                    getattr(src, "left_open", getattr(src, "left_is_open", 0.0))
                )
            if hasattr(wp, "line_stop"):
                wp.line_stop = bool(
                    getattr(src, "line_stop", getattr(src, "line_is_stop", False))
                )
            if hasattr(wp, "signal_stop"):
                wp.signal_stop = bool(
                    getattr(src, "signal_stop", getattr(src, "signal_is_stop", False))
                )
            if hasattr(wp, "not_skip"):
                wp.not_skip = bool(
                    getattr(src, "not_skip", getattr(src, "isnot_skipnum", False))
                )
            virtual.append(wp)
            indices.append(int(idx))

        u_wp = seg_u_first[0]
        last_wp = virtual[-1]
        if not almost_same_xy(
            last_wp.pose.position.x,
            last_wp.pose.position.y,
            u_wp.pose.position.x,
            u_wp.pose.position.y,
        ):
            raise RuntimeError("Virtual edge does not reach the U endpoint.")

        return virtual, indices


# ===== エントリポイント ===============================================================

def main(argv: Optional[List[str]] = None) -> None:
    """ノードのエントリポイント。"""
    rclpy.init(args=argv)
    node = RoutePlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
