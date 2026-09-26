# -*- coding: utf-8 -*-
"""Obstacle Monitor Node (legacy-following, Phase2, ROS 2 Foxy)

Google Python Style / 型ヒント完備 / 日本語コメント。cv_bridgeはbgr8で出力。

本ノードは単一2D LiDARの `/scan` を購読し、以下を行う:
  * 前方くさび領域の閉塞判定 (front_blocked)  … legacy: ±front_cone_half_deg 内で x < stop_dist_m の点が存在
  * 左右の回避オフセット（可用幅）を legacy 方式で算出（ギャップ検出 + 外縁 + 下限0.75m）
  * 上記ヒントを `tc_route_msgs/ObstacleAvoidanceHint` として publish（front_clearance_m / left_offset_m / right_offset_m を [m] で出力）
  * デバッグ用に LiDAR 点群を画像化し `sensor_msgs/Image` (bgr8) で publish（laserScanViewer 相当の描画仕様）

参考に踏襲した実装: waypoint_manager.py の laserScanCallback / laserScanViewer。
差分・注意点は最下部のコメント参照。

前提:
  * LiDAR は base_link に前向きで固定搭載 (TF 変換は不要)
  * 出力 QoS は BestEffort / Volatile / depth=1
  * ROS 2 Foxy, rclpy, cv_bridge, numpy, opencv-python

Author: ChatGPT
"""

from typing import Tuple, Optional

import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from tc_diagnostics import DiagnosticReporter, report_alive
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, qos_profile_sensor_data

from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from builtin_interfaces.msg import Time as TimeMsg

# ユーザー指定: tc_route_msgs が正しい
from tc_route_msgs.msg import ObstacleAvoidanceHint  # stamp, front_blocked: bool, front_clearance_m: float32, left_offset_m: float32, right_offset_m: float32

from cv_bridge import CvBridge


class ObstacleMonitorNode(Node):
    """laserScanCallback / laserScanViewer を踏襲した Obstacle Monitor."""

    def __init__(self) -> None:
        super().__init__('obstacle_monitor')

        # ---- Parameters (legacy 準拠) ----
        # 前方閉塞判定くさび角（半角）
        self.declare_parameter('front_cone_half_deg', 10.0)
        # 停止しきい値 [m]（くさび内で x < stop_dist_m なら閉塞）
        self.declare_parameter('stop_dist_m', 1.0)
        # front_blocked 判定のために無視する近距離データの閾値 [m]
        self.declare_parameter('front_block_ignore_dist_m', 0.15)

        # 前方閉塞判定で利用する分位点[%]（帯内の下位何%を評価するか）
        self.declare_parameter('front_clearance_percentile', 5.0)
        # 前方閉塞判定を有効にするために必要な最小点数
        self.declare_parameter('front_clearance_min_points', 5)

        # 回避対象障害物の最大距離 [m]（legacy: 1.5m 固定）
        self.declare_parameter('max_obstacle_distance_m', 1.5)
        max_obstacle_distance = float(self.get_parameter('max_obstacle_distance_m').value)

        # 距離ヒントの算出対象距離 [m]（通知範囲以上を確保する）
        self.declare_parameter('hint_range_m', max_obstacle_distance)

        # ロボット幅 [m]（legacy: 0.8）
        self.declare_parameter('robot_width_m', 0.8)
        # 下限値 [m]（legacy: 0.75）
        self.declare_parameter('avoid_offset_min_m', 0.75)

        # viewer 仕様（legacy 相当: 8m x-range, ±4m y-range, 100 pix/m）
        self.declare_parameter('viewer_map_range_m', 8.0)
        self.declare_parameter('viewer_pixel_pitch', 100)  # [pix/m]
        self.declare_parameter('viewer_window', 'laserscan')

        # 描画サイズ（表示用にリサイズ）
        self.declare_parameter('viewer_resize_px', 500)

        # ---- QoS ----
        qos_be_volatile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )
        qos_rel_volatile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10,
        )
        # rqt_image_view などの可視化ツールは Reliable を要求するため、画像配信は信頼型QoSを用いる。
        qos_rel_volatile_shallow = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )

        # ---- Pub/Sub ----
        scan_topic = 'scan'
        hint_topic = 'obstacle_avoidance_hint'
        viewer_topic = 'sensor_viewer'
        pose_enu_topic = 'localization/pose_enu'
        active_target_topic = 'active_target'

        self.sub_scan = self.create_subscription(
            LaserScan,
            scan_topic,
            self.laser_scan_callback,
            qos_profile_sensor_data,  # SensorDataQoS
        )

        self.pub_hint = self.create_publisher(
            ObstacleAvoidanceHint,
            hint_topic,
            qos_be_volatile,
        )

        self.pub_img = self.create_publisher(
            Image,
            viewer_topic,
            qos_rel_volatile_shallow,
        )

        self.sub_pose_enu = self.create_subscription(
            PoseWithCovarianceStamped,
            pose_enu_topic,
            self._on_pose_enu,
            qos_rel_volatile,
        )

        self.sub_active_target = self.create_subscription(
            PoseStamped,
            active_target_topic,
            self._on_active_target,
            qos_rel_volatile,
        )

        # 解決済みの名前をログ用に保持
        self.scan_topic: str = self._resolve_topic_name(scan_topic)
        self.hint_topic: str = self._resolve_topic_name(hint_topic)
        self.viewer_topic: str = self._resolve_topic_name(viewer_topic)
        self.pose_enu_topic: str = self._resolve_topic_name(pose_enu_topic)
        self.active_target_topic: str = self._resolve_topic_name(active_target_topic)

        # ---- Misc ----
        self.bridge = CvBridge()

        # cache: 最新の点群（viewer 用）
        self._last_points_xy: Optional[np.ndarray] = None
        self._latest_front_clearance: float = float('inf')
        self._hint_range_m: float = max_obstacle_distance
        self._hint_range_last_warned: Optional[float] = None
        self._pose_enu_xyyaw: Optional[Tuple[float, float, float]] = None
        self._active_target_xy: Optional[Tuple[float, float]] = None

        # 初期化時に距離範囲の制約を適用
        self._ensure_hint_range(max_obstacle_distance)

        # 自己申告診断。生存はreporterのタイマーが回ることで表される。
        self._diagnostics = DiagnosticReporter(self)
        report_alive(self._diagnostics)
        self.get_logger().info('obstacle_monitor (legacy-following) started.')

    def _resolve_topic_name(self, name: str) -> str:
        """リマップ適用後のトピック名を取得するユーティリティ."""
        try:
            return self.resolve_topic_name(name)
        except AttributeError:
            # テスト環境などで rclpy の内部APIが無い場合は素の名前を返す。
            return name

    # ===============================
    # Legacy: laserScanCallback
    # ===============================
    def laser_scan_callback(self, msg: LaserScan) -> None:
        """LaserScan 受信時の処理（legacy 算法踏襲）.

        1) ±90° の前方のみ残す
        2) x <= max_obstacle_distance_m で抽出
        3) 左右に分割し、|y| 昇順で並べ替え
        4) ギャップ検出で offset を決定、外縁採用もあり。最終的に avoid_offset_min_m を下限に丸め
        5) front_blocked 判定: ±front_cone_half_deg くさび内に x<stop_dist_m の点があれば True
        6) ヒント publish（left_offset_m/right_offset_m に offset[m] を格納）
        7) viewer 画像 publish（laserScanViewer 相当の描画）
        """
        # ---- 角度列の生成 ----
        angle_min: float = float(msg.angle_min)
        angle_inc: float = float(msg.angle_increment)

        # ---- XY 配列を生成（NaN/Inf/近距離<0.2m 除外） ----
        xy = self._scan_to_xy(msg, angle_min, angle_inc)

        # ---- 前方 ±90° のみ残す（後方は無視） ----
        if xy.size > 0:
            # xy[:,2] は角度[rad] を保持する列にしていないので、角度条件は変換前に適用すべきだが
            # ここでは _scan_to_xy 内で既に ±90° をフィルタリングしている。
            pass

        self._last_points_xy = xy.copy() if xy.size > 0 else None

        # ---- x <= max_obstacle_distance_m で抽出 & 近距離点を除外 ----
        max_dist = float(self.get_parameter('max_obstacle_distance_m').value)
        hint_range = self._ensure_hint_range(max_dist)
        ignore_dist = float(self.get_parameter('front_block_ignore_dist_m').value)
        # 前方距離(x)が閾値未満の点を除外してから、最大距離でフィルタリング
        xy_extract = (
            xy[(xy[:, 0] >= ignore_dist) & (xy[:, 0] <= max_dist)]
            if xy.size > 0
            else np.empty((0, 2), dtype=np.float32)
        )

        # ---- 左右に分割（y 符号）し、|y| 昇順で並べ替え ----
        left = xy_extract[xy_extract[:, 1] >= 0.0] if xy_extract.size > 0 else np.empty((0, 2), dtype=np.float32)
        right = xy_extract[xy_extract[:, 1] < 0.0] if xy_extract.size > 0 else np.empty((0, 2), dtype=np.float32)

        left = left[np.argsort(np.abs(left[:, 1]))] if left.size > 0 else left
        right = right[np.argsort(np.abs(right[:, 1]))] if right.size > 0 else right

        # ---- オフセット算出（legacy: calcAvoidanceOffset） ----
        robot_width = float(self.get_parameter('robot_width_m').value)
        half_width = robot_width / 2.0
        min_lower = float(self.get_parameter('avoid_offset_min_m').value)

        offset_l = self._calc_avoidance_offset(left, robot_width, half_width)
        offset_r = self._calc_avoidance_offset(right, robot_width, half_width)

        # legacy: 下限 0.75m を適用
        offset_l = max(offset_l, min_lower) if offset_l > 0.0 else 0.0
        offset_r = max(offset_r, min_lower) if offset_r > 0.0 else 0.0

        # ---- 前方閉塞判定（±front_cone_half_deg くさび & x < stop_dist_m） ----
        stop_dist_m = float(self.get_parameter('stop_dist_m').value)
        front_clearance_percentile = float(
            self.get_parameter('front_clearance_percentile').value
        )
        front_clearance_min_points = int(
            self.get_parameter('front_clearance_min_points').value
        )
        front_blocked, front_clearance_raw = self._is_front_blocked(
            xy_extract,
            robot_width,
            stop_dist_m,
            front_clearance_percentile,
            front_clearance_min_points,
        )
        if math.isfinite(front_clearance_raw) and front_clearance_raw <= hint_range:
            front_clearance_hint = float(front_clearance_raw)
        else:
            front_clearance_hint = float('inf')

        self._latest_front_clearance = front_clearance_hint

        # ---- ヒント publish ----
        def _fmt(val: float) -> str:
            return f"{val:.2f}" if math.isfinite(val) else "inf"

        self.get_logger().info(
            "Publish hint: stop_dist=%.2f, front_blocked=%s, front_clearance=%s, left_offset=%.2f, right_offset=%.2f"
            % (
                stop_dist_m,
                front_blocked,
                _fmt(
                    front_clearance_hint
                    if math.isfinite(front_clearance_hint)
                    else front_clearance_raw
                ),
                offset_l,
                offset_r,
            )
        )
        self._publish_hint(front_blocked, front_clearance_hint, offset_l, offset_r)

        # ---- 画像 publish（laserScanViewer 相当） ----
        self._publish_scan_image(
            xy,
            offset_l,
            offset_r,
            robot_width,
            half_width,
            front_clearance_hint,
            hint_range,
        )

    # -------------------------------
    # XY 変換（±90° フィルタ含む）
    # -------------------------------
    def _scan_to_xy(self, msg: LaserScan, angle_min: float, angle_inc: float) -> np.ndarray:
        """LaserScan → XY。NaN/Inf/<0.2m 除外。±90° のみ残す。"""
        pts = []
        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r) or r < 0.2:
                continue
            angle = angle_min + i * angle_inc
            deg = math.degrees(angle)
            if deg < -90.0 or deg > 90.0:
                continue
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            pts.append((x, y))
        if not pts:
            return np.empty((0, 2), dtype=np.float32)
        return np.asarray(pts, dtype=np.float32)

    # -------------------------------
    # legacy: calcAvoidanceOffset
    # -------------------------------
    def _calc_avoidance_offset(self, xy: np.ndarray, robot_width: float, half_width: float) -> float:
        """隣接点の |y| 差分からギャップを検出し、外縁も考慮して offset を算出する（legacy準拠）.

        Args:
            xy: |y| 昇順にソート済みの点群（片側）。shape = (N, 2)
            robot_width: ロボット幅 [m]
            half_width: ロボット半幅 [m]

        Returns:
            offset [m]。見つからなければ 0.0。
        """
        if xy.size == 0:
            return 0.0

        offset = 0.0
        y_prev = 0.0
        thresh = robot_width + half_width  # legacy: 幅 + 半幅 を閾値に利用

        for i in range(len(xy)):
            y = abs(float(xy[i][1]))
            # 隣の点との間が (幅 + 半幅) よりも開いている場合 … ギャップとして採用
            if y - y_prev > thresh:
                # 車幅分のマージンを設けてオフセット算出
                offset = y_prev + robot_width
                # 直進経路上（y_prev==0）に障害物が無ければ回避不要（offset=0 のまま）
                if y_prev == 0.0:
                    break
                # あまり大きすぎるオフセットは採用しない（legacy 上限 3.0m 相当）
                if offset < 3.0:
                    break
            # 外側に点が存在しない場合 … 外縁 + マージン
            elif i == len(xy) - 1:
                offset = y + (thresh) / 2.0
                if offset < 3.0:
                    break
            y_prev = y
        return float(offset)

    # -------------------------------
    # 前方閉塞判定（帯＋分位点）
    # -------------------------------
    def _is_front_blocked(
        self,
        pts: np.ndarray,
        robot_width_m: float,
        stop_dist_m: float,
        percentile: float = 5.0,
        min_points: int = 5,
    ) -> Tuple[bool, float]:
        """前方閉塞判定（帯＋分位点）と距離算出。

        Args:
            pts: _scan_to_xy() が出力した XY点群 [ [x0, y0], [x1, y1], ... ]。
            robot_width_m: ロボット全幅[m]（安全マージン込み）。
            stop_dist_m: 停止距離[m]。
            percentile: 分位点[%]（例: 5.0 → 帯内x距離の下位5%を評価）。
            min_points: 判定に必要な最小点数。

        Returns:
            Tuple[bool, float]: (前方閉塞していればTrue, 最近傍距離[同分位点])。
        """
        if pts.size == 0 or pts.shape[0] < min_points:
            return False, float('inf')

        xs = pts[:, 0]
        ys = pts[:, 1]

        # y方向±half_widthの帯内
        half_w = 0.5 * robot_width_m
        mask = (xs > 0.0) & (np.abs(ys) <= half_w)
        if not np.any(mask):
            return False, float('inf')

        x_band = xs[mask]
        x_q = float(np.percentile(x_band, percentile))
        return x_q <= stop_dist_m, x_q


    # -------------------------------
    # ヒント publish
    # -------------------------------
    def _publish_hint(
        self,
        front_blocked: bool,
        front_clearance_m: float,
        left_offset_m: float,
        right_offset_m: float,
    ) -> None:
        """ObstacleAvoidanceHint を publish."""
        msg = ObstacleAvoidanceHint()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.front_blocked = bool(front_blocked)
        msg.front_clearance_m = float(front_clearance_m)
        msg.left_offset_m = float(left_offset_m)
        msg.right_offset_m = float(right_offset_m)

        self.pub_hint.publish(msg)

    # -------------------------------
    # Pose callbacks / helpers
    # -------------------------------
    def _on_pose_enu(self, msg: PoseWithCovarianceStamped) -> None:
        pose = msg.pose.pose
        yaw = self._yaw_from_quaternion(pose.orientation)
        self._pose_enu_xyyaw = (
            float(pose.position.x),
            float(pose.position.y),
            float(yaw),
        )

    def _on_active_target(self, msg: PoseStamped) -> None:
        self._active_target_xy = (
            float(msg.pose.position.x),
            float(msg.pose.position.y),
        )

    def _ensure_hint_range(self, max_distance: float) -> float:
        """距離ヒント範囲の下限を通知範囲以上に保ち、必要時に警告する。"""
        raw_value = float(self.get_parameter('hint_range_m').value)
        value = raw_value
        if value < max_distance:
            if self._hint_range_last_warned != raw_value:
                self.get_logger().warn(
                    "hint_range_m=%.2f is smaller than max_obstacle_distance_m=%.2f. Using %.2f instead.",
                    raw_value,
                    max_distance,
                    max_distance,
                )
                self._hint_range_last_warned = raw_value
            value = max_distance
        else:
            self._hint_range_last_warned = None
        self._hint_range_m = value
        return value

    @staticmethod
    def _yaw_from_quaternion(q) -> float:
        """geometry_msgs/Quaternion から yaw[rad] を算出。"""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    # ===============================
    # Legacy: laserScanViewer 相当
    # ===============================
    def _publish_scan_image(
        self,
        points_xy: np.ndarray,
        offset_l: float,
        offset_r: float,
        robot_width: float,
        half_width: float,
        front_clearance_m: float,
        hint_range_m: float,
    ) -> None:
        """legacy の laserScanViewer と同等の可視化を bgr8 で publish する."""
        # 表示仕様
        map_range = float(self.get_parameter('viewer_map_range_m').value)  # 8.0
        pixel_pitch = int(self.get_parameter('viewer_pixel_pitch').value)  # 100 pix/m
        pixel_pitch_f = float(pixel_pitch)
        resize_px = int(self.get_parameter('viewer_resize_px').value)
        window_name = str(self.get_parameter('viewer_window').value)

        # マップ領域（legacy: x:0..8, y:-4..4）
        x_min = 0.0
        x_max = (map_range / 2.0) * 2.0
        y_min = -1.0 * (map_range / 2.0)
        y_max = (map_range / 2.0)

        width = int((y_max - y_min) * pixel_pitch)   # 横
        height = int((x_max - x_min) * pixel_pitch)  # 縦
        grid = np.full((height, width, 3), 255, dtype=np.uint8)

        origin_px = (int(width / 2), height)

        # ロボット幅の縦ライン（黒）: 画像中心を基準に左右へ
        pix_x = int(width / 2) - int(robot_width * pixel_pitch)
        cv2.line(grid, (pix_x, height), (pix_x, 0), (0, 0, 0), thickness=2, lineType=cv2.LINE_AA)
        pix_x = int(width / 2) + int(robot_width * pixel_pitch)
        cv2.line(grid, (pix_x, height), (pix_x, 0), (0, 0, 0), thickness=2, lineType=cv2.LINE_AA)

        # 最大障害物距離ライン（黒）
        max_obstacle_distance = float(self.get_parameter('max_obstacle_distance_m').value)
        pix_y = height - int(max_obstacle_distance * pixel_pitch)
        cv2.line(grid, (0, pix_y), (width, pix_y), (0, 0, 0), thickness=2, lineType=cv2.LINE_AA)

        # 距離算出範囲ライン（オレンジ）
        hint_py = height - int(round(hint_range_m * pixel_pitch_f))
        if 0 <= hint_py < height:
            cv2.line(grid, (0, hint_py), (width, hint_py), (0, 165, 255), thickness=1, lineType=cv2.LINE_AA)

        # front_clearance ライン（マゼンタ）
        if math.isfinite(front_clearance_m):
            front_py = height - int(round(front_clearance_m * pixel_pitch_f))
            if 0 <= front_py < height:
                cv2.line(grid, (0, front_py), (width, front_py), (255, 0, 255), thickness=2, lineType=cv2.LINE_AA)

        # 点群（赤）: x 前方を下方向、y 左右を画像横方向に写像（legacy と同じ）
        if points_xy is not None and points_xy.size > 0:
            for x, y in points_xy:
                if x_min < x < x_max and y_min < y < y_max:
                    px = int(width / 2) - int(y * pixel_pitch)   # y: 左(+)->右(-) へ
                    py = height - int(x * pixel_pitch)           # x: 前方->下
                    if 0 <= px < width and 0 <= py < height:
                        cv2.circle(grid, (px, py), 4, (0, 0, 255), -1)

        # 回避ライン（青）: vline_l と vline_r
        # legacy 呼び出しでは vline_l=offset_l, vline_r=-offset_r を渡していたので同様に適用
        vline_l = offset_l
        vline_r = -1.0 * offset_r

        px = int(width / 2) - int(vline_l * pixel_pitch)
        if vline_l != 0.0 and 0 <= px < width:
            cv2.line(grid, (px, height), (px, 0), (255, 0, 0), thickness=2, lineType=cv2.LINE_AA)

        px = int(width / 2) - int(vline_r * pixel_pitch)
        if vline_r != 0.0 and 0 <= px < width:
            cv2.line(grid, (px, height), (px, 0), (255, 0, 0), thickness=2, lineType=cv2.LINE_AA)

        # active_target の描画
        target_px: Optional[Tuple[int, int]] = None
        if self._pose_enu_xyyaw is not None and self._active_target_xy is not None:
            ax, ay, yaw = self._pose_enu_xyyaw
            tx, ty = self._active_target_xy
            dx = float(tx) - float(ax)
            dy = float(ty) - float(ay)
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            local_x = cos_yaw * dx + sin_yaw * dy
            local_y = -sin_yaw * dx + cos_yaw * dy
            if math.isfinite(local_x) and math.isfinite(local_y):
                px_t = int(round(origin_px[0] - local_y * pixel_pitch_f))
                py_t = int(round(origin_px[1] - local_x * pixel_pitch_f))
                target_px = (px_t, py_t)

        if target_px is not None:
            clipped, pt1, pt2 = cv2.clipLine((0, 0, width, height), origin_px, target_px)
            if clipped:
                cv2.line(grid, pt1, pt2, (0, 140, 255), thickness=2, lineType=cv2.LINE_AA)
            if 0 <= target_px[0] < width and 0 <= target_px[1] < height:
                cv2.circle(grid, target_px, 6, (0, 140, 255), thickness=2)

        # 表示用リサイズ（ウィンドウ表示は行わず、画像トピックとして配信）
        grid_show = cv2.resize(grid, (resize_px, resize_px), interpolation=cv2.INTER_AREA)
        # デバッグ表示
        #cv2.imshow("Sensor Viewer", grid_show)
        #cv2.waitKey(1)

        # bgr8 で publish
        img_msg = self.bridge.cv2_to_imgmsg(grid_show, encoding='bgr8')
        # frame_id は設計上必要なし（Image の header に frame は含まれるが、legacy では未使用）
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = 'base_link'
        self.pub_img.publish(img_msg)

    # -------------------------------
    # Utility
    # -------------------------------
    def _now(self) -> TimeMsg:
        now = self.get_clock().now().to_msg()
        return now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObstacleMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
