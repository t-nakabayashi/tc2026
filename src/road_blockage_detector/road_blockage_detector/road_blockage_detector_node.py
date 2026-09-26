#!/usr/bin/env python3
"""road_blockage_detector ノードの実装モジュール."""

from collections import deque
import math
from typing import Deque, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from tc_diagnostics import DiagnosticReporter, report_alive
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from std_msgs.msg import Bool
from tc_perception_msgs.msg import OverlayDetection, PerceptionOverlay
from vision_msgs.msg import Detection2D, Detection2DArray


class RoadBlockageDetector(Node):
    """YOLO 検知結果から経路封鎖を判定するノード."""

    def __init__(self) -> None:
        super().__init__('road_blockage_detector')

        self._declare_parameters()
        self._load_parameters()

        self.count_history: Deque[Tuple[int, int]] = deque()
        self.latest_pose_enu: Optional[Pose] = None
        self.latest_pose_enu_time: Optional[Time] = None
        self.blocked_positions: List[Pose] = []
        self.temporary_decision_count = 0
        self.blocked_state_started_at: Optional[float] = None
        self.road_blocked_state = False
        self.last_valid_detection_count = 0
        self.last_detection_ratio = 0.0

        qos_sensor_data = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.detections_subscriber = self.create_subscription(
            Detection2DArray,
            self.detections_topic,
            self._detections_callback,
            qos_sensor_data,
        )
        self.overlay_publisher = self.create_publisher(
            PerceptionOverlay,
            self.overlay_topic,
            qos_sensor_data,
        )
        self.pose_enu_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            self.pose_enu_topic,
            self._pose_enu_callback,
            10,
        )
        self.road_blocked_publisher = self.create_publisher(Bool, self.road_blocked_topic, 10)

        # 自己申告診断。生存はreporterのタイマーが回ることで表される。
        self._diagnostics = DiagnosticReporter(self)
        report_alive(self._diagnostics)

        self.get_logger().info(
            'road_blockage_detector を起動しました。'
            f' detections={self.detections_topic}, pose_enu={self.pose_enu_topic}, '
            f'road_blocked={self.road_blocked_topic}, '
            f'overlay={self.overlay_topic}'
        )

    def _declare_parameters(self) -> None:
        """ノードが使用するパラメータを宣言する."""

        self.declare_parameter('target_class_id', 0)
        self.declare_parameter('detections_topic', '/perception/road_blockage/detections')
        self.declare_parameter('pose_enu_topic', '/localization/pose_enu')
        self.declare_parameter('road_blocked_topic', '/road_blocked')
        self.declare_parameter('overlay_topic', '/perception/road_blockage/overlay')
        self.declare_parameter('score_threshold', 0.5)
        self.declare_parameter('bbox_width_min', -1.0)
        self.declare_parameter('bbox_width_max', -1.0)
        self.declare_parameter('bbox_height_min', -1.0)
        self.declare_parameter('bbox_height_max', -1.0)
        self.declare_parameter('bbox_bottom_max', -1.0)
        self.declare_parameter('decision_duration', 3.0)
        self.declare_parameter('decision_frame_ratio', 50.0)
        self.declare_parameter('confirmation_duration', 10.0)
        self.declare_parameter('multi_detection_suppression_range', 10.0)

    def _load_parameters(self) -> None:
        """宣言済みパラメータを読み込む."""

        self.target_class_id = self._get_int_parameter('target_class_id')
        self.detections_topic = self._get_string_parameter('detections_topic')
        self.pose_enu_topic = self._get_string_parameter('pose_enu_topic')
        self.road_blocked_topic = self._get_string_parameter('road_blocked_topic')
        self.overlay_topic = self._get_string_parameter('overlay_topic')
        self.score_threshold = self._get_double_parameter('score_threshold')
        self.bbox_width_min = self._get_double_parameter('bbox_width_min')
        self.bbox_width_max = self._get_double_parameter('bbox_width_max')
        self.bbox_height_min = self._get_double_parameter('bbox_height_min')
        self.bbox_height_max = self._get_double_parameter('bbox_height_max')
        self.bbox_bottom_max = self._get_double_parameter('bbox_bottom_max')
        self.decision_duration = self._get_double_parameter('decision_duration')
        self.decision_frame_ratio = self._get_double_parameter('decision_frame_ratio')
        self.confirmation_duration = self._get_double_parameter('confirmation_duration')
        self.multi_detection_suppression_range = self._get_double_parameter(
            'multi_detection_suppression_range'
        )

    def _get_int_parameter(self, name: str) -> int:
        """整数値パラメータを取得するヘルパー."""

        return self.get_parameter(name).get_parameter_value().integer_value

    def _get_double_parameter(self, name: str) -> float:
        """浮動小数点パラメータを取得するヘルパー."""

        return self.get_parameter(name).get_parameter_value().double_value

    def _get_string_parameter(self, name: str) -> str:
        """文字列パラメータを取得するヘルパー."""

        return self.get_parameter(name).get_parameter_value().string_value

    def _pose_enu_callback(self, msg: PoseWithCovarianceStamped) -> None:
        """最新の pose_enu をキャッシュする."""

        self.latest_pose_enu = msg.pose.pose
        self.latest_pose_enu_time = Time.from_msg(msg.header.stamp)

    def _detections_callback(self, msg: Detection2DArray) -> None:
        """Detection2DArray を受信し、判定処理を行う."""

        detection_time = Time.from_msg(msg.header.stamp)
        pose = self._lookup_pose(detection_time)
        if pose is None:
            self.get_logger().warn(
                '自己位置が取得できないため検知をスキップします。'
            )
            self._record_count(detection_time, 0)
            self.last_valid_detection_count = 0
            self.last_detection_ratio = self._compute_detection_ratio()
            self._publish_overlay(msg, [], 'no_pose')
            return

        self._maybe_clear_blocked_state(pose)
        if self._suppress_near_blocked_position(pose, detection_time):
            self.last_valid_detection_count = 0
            self.last_detection_ratio = self._compute_detection_ratio()
            self._publish_overlay(msg, [], 'suppressed')
            return

        valid_detections = self._extract_valid_detections(msg.detections)
        valid_count = len(valid_detections)
        self._record_count(detection_time, valid_count)
        self.last_valid_detection_count = valid_count
        self._evaluate_decision(detection_time, pose)
        self.last_detection_ratio = self._compute_detection_ratio()
        self._publish_overlay(msg, valid_detections, '')

    def _count_valid_detections(self, detections: List[Detection2D]) -> int:
        """要求仕様に合致する検知数を数える."""

        return len(self._extract_valid_detections(detections))

    def _extract_valid_detections(self, detections: List[Detection2D]) -> List[Detection2D]:
        """要求仕様に合致する検知を抽出する."""

        if not detections:
            return []

        valid_detections: List[Detection2D] = []
        for detection in detections:
            best_result = self._extract_best_result(detection)
            if best_result is None:
                continue

            if best_result[1] < self.score_threshold:
                continue
            if best_result[0] != self.target_class_id:
                continue

            if not self._is_bbox_within_threshold(detection):
                continue

            valid_detections.append(detection)

        return valid_detections

    def _extract_best_result(self, detection: Detection2D) -> Optional[Tuple[int, float]]:
        """Detection2D.results からスコア最大の (class_id, score) を返す."""

        best_pair: Optional[Tuple[int, float]] = None
        for result in detection.results:
            try:
                class_id = int(result.hypothesis.class_id)
                score = float(result.hypothesis.score)
            except (AttributeError, TypeError, ValueError):
                continue
            if best_pair is None or score > best_pair[1]:
                best_pair = (class_id, score)
        return best_pair

    def _is_bbox_within_threshold(self, detection: Detection2D) -> bool:
        """バウンディングボックスの閾値判定を行う."""

        width = detection.bbox.size_x
        height = detection.bbox.size_y
        bottom_from_top = detection.bbox.center.position.y + (detection.bbox.size_y / 2.0)
        bottom_distance = self._compute_bottom_distance(detection, bottom_from_top)

        if self.bbox_width_min >= 0 and width < self.bbox_width_min:
            return False
        if self.bbox_width_max >= 0 and width > self.bbox_width_max:
            return False
        if self.bbox_height_min >= 0 and height < self.bbox_height_min:
            return False
        if self.bbox_height_max >= 0 and height > self.bbox_height_max:
            return False
        if self.bbox_bottom_max >= 0:
            if bottom_distance is None:
                self.get_logger().warn(
                    '画像高さ情報が無いため bbox_bottom_max 判定を'
                    'スキップします。',
                )
            elif bottom_distance > self.bbox_bottom_max:
                return False

        return True

    def _compute_bottom_distance(
        self, detection: Detection2D, bottom_from_top: float
    ) -> Optional[float]:
        """バウンディングボックス下端と画像下端の距離を算出する."""

        image_height = getattr(detection, 'image_height', None)
        if image_height is None:
            return bottom_from_top

        return max(float(image_height) - bottom_from_top, 0.0)

    def _record_count(self, stamp: Time, count: int) -> None:
        """秒単位のバケットに判定カウントを記録する."""

        bucket_start = math.floor(stamp.nanoseconds / 1_000_000_000)
        if self.count_history and self.count_history[-1][0] == bucket_start:
            prev_bucket = self.count_history.pop()
            self.count_history.append((bucket_start, prev_bucket[1] + count))
        else:
            self.count_history.append((bucket_start, count))

        threshold = bucket_start - math.ceil(self.decision_duration)
        while self.count_history and self.count_history[0][0] < threshold:
            self.count_history.popleft()

    def _evaluate_decision(self, stamp: Time, pose: Optional[Pose]) -> None:
        """履歴から road_blocked の状態遷移と封鎖確定判定を行う."""

        ratio = self._compute_detection_ratio()
        if ratio >= self.decision_frame_ratio:
            previous = self.temporary_decision_count
            self.temporary_decision_count += 1
            if previous == 0:
                self.blocked_state_started_at = self.get_clock().now().nanoseconds / 1e9
                self._publish_road_blocked(True, force=True)
                self.get_logger().info(
                    '封鎖を仮判定しました。走行を停止します。'
                )
        else:
            if self.temporary_decision_count > 0:
                elapsed = None
                if self.blocked_state_started_at is not None:
                    elapsed = (
                        self.get_clock().now().nanoseconds / 1e9 - self.blocked_state_started_at
                    )
                self.temporary_decision_count = 0
                self.blocked_state_started_at = None
                self._publish_road_blocked(False)
                if elapsed is not None:
                    self.get_logger().info(
                        f'封鎖判定を解除しました。継続時間: {elapsed:.2f} 秒'
                    )
                else:
                    self.get_logger().info('封鎖判定を解除しました。')
        self._handle_confirmation(pose)

    def _compute_detection_ratio(self) -> float:
        """count_history から閾値割合 [%] を算出する."""

        if not self.count_history:
            return 0.0

        active_buckets = sum(1 for _, value in self.count_history if value >= 1)
        ratio = (active_buckets / len(self.count_history)) * 100.0
        return ratio

    def _publish_overlay(
        self,
        detection_msg: Detection2DArray,
        valid_detections: List[Detection2D],
        status_note: str,
    ) -> None:
        """重畳表示用の認識結果を publish する.

        本ノードは画像を購読・配信しない。`header` は判定に使用した
        `Detection2DArray` のものをそのまま引き継ぎ、`yolo_detector` が複製した
        元画像フレームの stamp/frame_id を表示側へ伝える。

        `valid_detections` に含まれない検出も `adopted=False` として残し、
        判定に採用されなかった検出を表示側で描き分けられるようにする。

        Args:
            detection_msg (Detection2DArray): 判定に使用した検出結果.
            valid_detections (List[Detection2D]): 判定に採用した検出.
            status_note (str): 判定できなかった理由。正常時は空文字.
        """

        adopted_ids = {id(detection) for detection in valid_detections}

        overlay = PerceptionOverlay()
        overlay.header = detection_msg.header
        overlay.source = 'road_blockage'
        overlay.detections = [
            self._to_overlay_detection(detection, id(detection) in adopted_ids)
            for detection in detection_msg.detections
        ]
        overlay.decision = 1 if self.road_blocked_state else 0
        overlay.decision_text = self._decision_state_label()
        overlay.status_note = status_note
        self.overlay_publisher.publish(overlay)

    def _to_overlay_detection(
        self, detection: Detection2D, adopted: bool
    ) -> OverlayDetection:
        """Detection2D を表示用の OverlayDetection へ変換する.

        Args:
            detection (Detection2D): 変換元の検出.
            adopted (bool): 判定に採用された検出か.

        Returns:
            OverlayDetection: 表示用の検出 1 件.
        """

        entry = OverlayDetection()
        entry.center_x = float(detection.bbox.center.position.x)
        entry.center_y = float(detection.bbox.center.position.y)
        entry.size_x = float(detection.bbox.size_x)
        entry.size_y = float(detection.bbox.size_y)
        entry.label = 'road_blockage'
        best = self._extract_best_result(detection)
        entry.score = float(best[1]) if best is not None else 0.0
        entry.adopted = adopted
        return entry

    def _decision_state_label(self) -> str:
        """画像表示用の封鎖判定状態名を返す."""

        if self.road_blocked_state and self.temporary_decision_count == 0:
            return 'confirmed'
        if self.temporary_decision_count > 0:
            return 'judging'
        return 'clear'

    def _handle_confirmation(self, pose: Optional[Pose]) -> None:
        """confirmation_duration を超えた場合に封鎖確定処理を行う."""

        if self.temporary_decision_count <= 0 or self.blocked_state_started_at is None:
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec - self.blocked_state_started_at < self.confirmation_duration:
            return

        target_stamp = self.latest_pose_enu_time or Time()
        confirmation_pose = pose or self._lookup_pose(target_stamp)
        if confirmation_pose is None:
            self.get_logger().warn(
                '封鎖確定時に自己位置を取得できませんでした。'
                '記録をスキップします。'
            )
            return

        self.blocked_positions.append(self._copy_pose(confirmation_pose))
        self.get_logger().info(
            '封鎖を確定しました。位置を記録します: '
            f"x={confirmation_pose.position.x:.2f}, y={confirmation_pose.position.y:.2f}"
        )

        self._reset_detection_history()
        self._reset_temporary_decision_state()

    def _lookup_pose(self, stamp: Time) -> Optional[Pose]:
        """最新の pose_enu から現在位置を取得する."""

        if self.latest_pose_enu is None or self.latest_pose_enu_time is None:
            return None

        time_diff_sec = abs(self.latest_pose_enu_time.nanoseconds - stamp.nanoseconds) / 1e9
        if time_diff_sec >= 3.0:
            self.get_logger().warn(
                'Detection と /localization/pose_enu のタイムスタンプに'
                '3秒以上の差があります。'
            )

        return self._copy_pose(self.latest_pose_enu)

    def _is_within_blocked_positions(self, pose: Pose) -> bool:
        """過去の封鎖位置近傍にいるかを判定する."""

        for blocked_pose in self.blocked_positions:
            distance = math.hypot(
                blocked_pose.position.x - pose.position.x,
                blocked_pose.position.y - pose.position.y,
            )
            if distance < self.multi_detection_suppression_range:
                return True
        return False

    def _suppress_near_blocked_position(self, pose: Pose, stamp: Time) -> bool:
        """確定済み封鎖地点近傍での多重検知を抑制する."""

        if not self._is_within_blocked_positions(pose):
            return False

        self._record_count(stamp, 0)
        self._reset_detection_history()
        self._reset_temporary_decision_state()
        return True

    def _reset_temporary_decision_state(self) -> None:
        """仮判定関連の状態を初期化する."""

        self.temporary_decision_count = 0
        self.blocked_state_started_at = None

    def _reset_detection_history(self) -> None:
        """検知履歴をリセットする."""

        self.count_history.clear()

    def _maybe_clear_blocked_state(self, pose: Pose) -> None:
        """抑制範囲外に移動した場合に封鎖状態を解除する."""

        if not self.road_blocked_state:
            return
        if self.temporary_decision_count > 0 or self.blocked_state_started_at is not None:
            return
        if self._is_within_blocked_positions(pose):
            return

        self._publish_road_blocked(False)
        self.get_logger().info(
            '多重検知抑止範囲を離脱したため road_blocked を解除します。'
        )

    def _publish_road_blocked(self, is_blocked: bool, force: bool = False) -> None:
        """road_blocked を Publish する.

        同じブール値であっても内部状態の変化に応じて force=True で
        再通知を行う。force=False の場合は従来どおり状態変化時のみ
        通知する。
        """

        if not force and self.road_blocked_state == is_blocked:
            return

        self.road_blocked_state = is_blocked
        self.road_blocked_publisher.publish(Bool(data=is_blocked))
        state_text = 'true' if is_blocked else 'false'
        self.get_logger().info(f'road_blocked を {state_text} で通知しました。')

    @staticmethod
    def _copy_pose(pose: Pose) -> Pose:
        """Pose をディープコピーする."""

        copied = Pose()
        copied.position.x = pose.position.x
        copied.position.y = pose.position.y
        copied.position.z = pose.position.z
        copied.orientation.x = pose.orientation.x
        copied.orientation.y = pose.orientation.y
        copied.orientation.z = pose.orientation.z
        copied.orientation.w = pose.orientation.w
        return copied


def main(args=None) -> None:
    rclpy.init(args=args)

    node = RoadBlockageDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
