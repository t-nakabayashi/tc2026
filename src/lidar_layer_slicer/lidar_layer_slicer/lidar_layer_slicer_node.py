# -*- coding: utf-8 -*-
"""MID-360 の点群と topURG の scan から水平スライス層を作るノード.

MID-360 は前下がり取付であり、観測できる床上高さが距離に依存する。層の高さと厚さは
パラメータで与え、点群から床上高さで切り出す。取付姿勢は `base_link` からの TF を使い、
ノード内に角度を直書きしない。

本ノードは前方閉塞 (`front_blocked`) を判定しない。停止判定は obstacle_monitor が
topURG の `/scan` のみから行う分担を保つ。
"""

from collections import deque
from typing import Deque, List, Optional, Tuple

import math

import numpy as np

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import Image, LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from tc_diagnostics import DiagnosticReporter, report_alive
from tc_perception_msgs.msg import ObstacleLayer, ObstacleLayers
from tf2_ros import Buffer, TransformListener

from lidar_layer_slicer.layer_core import (SOURCE_POINT_CLOUD, SOURCE_SCAN,
                                           ClearanceConfig, LayerObservation,
                                           LayerSpec, observe_layer, scan_to_xy,
                                           slice_layer_points)
from lidar_layer_slicer.layer_render_core import ViewConfig, render_layer_set

# livox driver の CustomMsg は実機構成のみで使う。模擬構成では PointCloud2 を購読するため、
# 取り込めない環境でもノードが起動できるよう遅延して解決する。
try:  # pragma: no cover - 実機依存
    from livox_ros_driver2.msg import CustomMsg
except ImportError:  # pragma: no cover - 模擬・単体環境
    CustomMsg = None


class LidarLayerSlicerNode(Node):
    """点群と scan から水平スライス層の観測結果と可視化画像を配信する."""

    def __init__(self) -> None:
        super().__init__('lidar_layer_slicer')

        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('base_frame', 'base_link')
        # 床面から base_link 原点までの高さ [m]。車輪半径に相当する。
        self.declare_parameter('floor_to_base_link_m', 0.148248)

        # topURG から作る層。scan は単一平面であり厚さを持たない。
        self.declare_parameter('scan_layer_name', 'urg')
        self.declare_parameter('scan_layer_height_m', 0.25)
        self.declare_parameter('scan_half_fov_deg', 90.0)

        # MID-360 から切り出す層。名前・高さ・厚さを同じ並び順で与える。
        self.declare_parameter('cloud_layer_names', ['low', 'mid'])
        self.declare_parameter('cloud_layer_heights_m', [0.35, 0.50])
        self.declare_parameter('cloud_layer_thickness_m', [0.10, 0.10])
        # 入力の型。実機は livox の CustomMsg、模擬は PointCloud2 を使う。
        self.declare_parameter('cloud_message_type', 'custom')
        self.declare_parameter('accumulate_frames', 3)
        self.declare_parameter('accumulate_window_sec', 1.0)

        self.declare_parameter('forward_range_m', 2.0)
        self.declare_parameter('robot_width_m', 0.6)
        self.declare_parameter('side_margin_m', 0.1)
        self.declare_parameter('max_offset_m', 3.0)
        self.declare_parameter('near_x_m', 0.15)

        self.declare_parameter('view_range_x_m', 4.0)
        self.declare_parameter('view_range_y_m', 2.0)
        self.declare_parameter('view_pixel_pitch', 120)

        self._base_frame = str(self.get_parameter('base_frame').value)
        self._floor_to_base_link_m = float(self.get_parameter('floor_to_base_link_m').value)

        qos_stream = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )
        # 可視化ツールは Reliable を要求するため、画像のみ信頼型で配信する。
        qos_image = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )

        self._bridge = CvBridge()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._latest_scan: Optional[LaserScan] = None
        self._cloud_frames: Deque[Tuple[float, np.ndarray]] = deque()
        self._cloud_frame_id: str = ''
        self._transform_warned = False

        self.sub_scan = self.create_subscription(
            LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)

        message_type = str(self.get_parameter('cloud_message_type').value)
        if message_type == 'custom' and CustomMsg is not None:
            self.sub_cloud = self.create_subscription(
                CustomMsg, 'livox_lidar', self._on_custom_cloud, qos_profile_sensor_data)
        else:
            if message_type == 'custom':
                self.get_logger().warn(
                    'livox_ros_driver2 のメッセージを取り込めないため PointCloud2 を購読する')
            self.sub_cloud = self.create_subscription(
                PointCloud2, 'points', self._on_point_cloud2, qos_profile_sensor_data)

        self.pub_layers = self.create_publisher(ObstacleLayers, 'obstacle/layers', qos_stream)
        self.pub_view = self.create_publisher(Image, 'obstacle/layer_view', qos_image)

        rate = max(float(self.get_parameter('publish_rate_hz').value), 0.1)
        self.create_timer(1.0 / rate, self._on_timer)

        self._diagnostics = DiagnosticReporter(self)
        report_alive(self._diagnostics)
        self.get_logger().info(
            'lidar_layer_slicer を開始した。層定義=%s' % (self._layer_specs(),))

    # ========================= パラメータ由来の定義 =========================
    def _layer_specs(self) -> List[LayerSpec]:
        """scan 層と点群層の定義を、画像に並べる順（低い層から）で返す."""
        specs = [LayerSpec(
            name=str(self.get_parameter('scan_layer_name').value),
            source=SOURCE_SCAN,
            height_m=float(self.get_parameter('scan_layer_height_m').value),
            thickness_m=0.0)]
        names = list(self.get_parameter('cloud_layer_names').value or [])
        heights = list(self.get_parameter('cloud_layer_heights_m').value or [])
        thickness = list(self.get_parameter('cloud_layer_thickness_m').value or [])
        if not (len(names) == len(heights) == len(thickness)):
            self.get_logger().error(
                '点群層の名前・高さ・厚さの個数が一致しないため点群層を作らない')
            return specs
        for name, height, thick in zip(names, heights, thickness):
            specs.append(LayerSpec(name=str(name), source=SOURCE_POINT_CLOUD,
                                   height_m=float(height), thickness_m=float(thick)))
        return sorted(specs, key=lambda spec: spec.height_m)

    def _clearance_config(self) -> ClearanceConfig:
        return ClearanceConfig(
            forward_range_m=float(self.get_parameter('forward_range_m').value),
            robot_width_m=float(self.get_parameter('robot_width_m').value),
            side_margin_m=float(self.get_parameter('side_margin_m').value),
            max_offset_m=float(self.get_parameter('max_offset_m').value),
            near_x_m=float(self.get_parameter('near_x_m').value),
        )

    def _view_config(self) -> ViewConfig:
        return ViewConfig(
            range_x_m=float(self.get_parameter('view_range_x_m').value),
            range_y_m=float(self.get_parameter('view_range_y_m').value),
            pixel_pitch=int(self.get_parameter('view_pixel_pitch').value),
        )

    # ========================= 入力 =========================
    def _on_scan(self, msg: LaserScan) -> None:
        self._latest_scan = msg

    def _on_point_cloud2(self, msg: PointCloud2) -> None:
        points = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'),
                                                skip_nans=True)
        self._store_cloud(msg.header.frame_id, np.asarray(points, dtype=np.float32))

    def _on_custom_cloud(self, msg: 'CustomMsg') -> None:
        if not msg.points:
            return
        points = np.asarray([(p.x, p.y, p.z) for p in msg.points], dtype=np.float32)
        self._store_cloud(msg.header.frame_id, points)

    def _store_cloud(self, frame_id: str, points: np.ndarray) -> None:
        """センサ座標の点群を base_link へ変換して累積する."""
        if points.size == 0:
            return
        transformed = self._transform_to_base(frame_id, points)
        if transformed is None:
            return
        self._cloud_frame_id = frame_id
        now = self._now_sec()
        self._cloud_frames.append((now, transformed))
        limit = max(int(self.get_parameter('accumulate_frames').value), 1)
        while len(self._cloud_frames) > limit:
            self._cloud_frames.popleft()

    def _transform_to_base(self, frame_id: str, points: np.ndarray) -> Optional[np.ndarray]:
        """TF でセンサ座標の点群を base_link 系へ回転・平行移動する.

        TF が未取得の場合は None を返し、層を作らない。取付角をノード内に持たないため、
        TF が無いまま近似する選択はしない。
        """
        if not frame_id or frame_id == self._base_frame:
            return points
        try:
            transform = self._tf_buffer.lookup_transform(
                self._base_frame, frame_id, rclpy.time.Time())
        except Exception as error:  # tf2 の例外は複数種類あるため型を限定しない
            if not self._transform_warned:
                self.get_logger().warn(
                    f'{self._base_frame} <- {frame_id} の TF を取得できない: {error}')
                self._transform_warned = True
            return None
        self._transform_warned = False
        rotation = _quaternion_to_matrix(transform.transform.rotation)
        translation = np.asarray([transform.transform.translation.x,
                                  transform.transform.translation.y,
                                  transform.transform.translation.z], dtype=np.float32)
        return points @ rotation.T + translation

    # ========================= 出力 =========================
    def _on_timer(self) -> None:
        specs = self._layer_specs()
        config = self._clearance_config()
        cloud, span = self._accumulated_cloud()
        observations: List[LayerObservation] = []
        for spec in specs:
            if spec.source == SOURCE_SCAN:
                points_xy = self._scan_points()
            else:
                points_xy = slice_layer_points(cloud, spec, self._floor_to_base_link_m)
            observations.append(observe_layer(points_xy, spec, config))

        message = ObstacleLayers()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._base_frame
        message.forward_range_m = float(config.forward_range_m)
        message.robot_width_m = float(config.robot_width_m)
        message.side_margin_m = float(config.side_margin_m)
        message.accumulated_frames = int(len(self._cloud_frames))
        message.accumulated_span_s = float(span)
        for observation in observations:
            message.layers.append(_to_layer_message(observation))
        self.pub_layers.publish(message)

        image = render_layer_set(observations, config, self._view_config())
        image_message = self._bridge.cv2_to_imgmsg(image, encoding='bgr8')
        image_message.header = message.header
        self.pub_view.publish(image_message)

    def _scan_points(self) -> np.ndarray:
        """最新 scan を前方のみの xy 点へ変換する."""
        scan = self._latest_scan
        if scan is None:
            return np.empty((0, 2), dtype=np.float32)
        return scan_to_xy(scan.ranges, float(scan.angle_min), float(scan.angle_increment),
                          range_min=float(scan.range_min), range_max=float(scan.range_max),
                          half_fov_deg=float(self.get_parameter('scan_half_fov_deg').value))

    def _accumulated_cloud(self) -> Tuple[np.ndarray, float]:
        """累積窓内のフレームを連結した点群と、累積した時間幅 [s] を返す."""
        window = float(self.get_parameter('accumulate_window_sec').value)
        now = self._now_sec()
        frames = [(t, p) for t, p in self._cloud_frames if (now - t) <= window]
        self._cloud_frames = deque(frames)
        if not frames:
            return np.empty((0, 3), dtype=np.float32), 0.0
        span = frames[-1][0] - frames[0][0]
        return np.concatenate([p for _, p in frames], axis=0), span

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def _quaternion_to_matrix(quaternion) -> np.ndarray:
    """クォータニオンから 3x3 の回転行列を作る."""
    x, y, z, w = (float(quaternion.x), float(quaternion.y),
                  float(quaternion.z), float(quaternion.w))
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0:
        return np.eye(3, dtype=np.float32)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.asarray([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ], dtype=np.float32)


def _to_layer_message(observation: LayerObservation) -> ObstacleLayer:
    """観測結果を ObstacleLayer へ写す."""
    layer = ObstacleLayer()
    layer.name = observation.spec.name
    layer.source = observation.spec.source
    layer.height_m = float(observation.spec.height_m)
    layer.thickness_m = float(observation.spec.thickness_m)
    layer.point_count = int(observation.point_count)
    layer.nearest_x_m = float(observation.nearest_x_m)
    layer.required_left_offset_m = float(observation.required_left_offset_m)
    layer.required_right_offset_m = float(observation.required_right_offset_m)
    layer.available_left_offset_m = float(observation.available_left_offset_m)
    layer.available_right_offset_m = float(observation.available_right_offset_m)
    return layer


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarLayerSlicerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
