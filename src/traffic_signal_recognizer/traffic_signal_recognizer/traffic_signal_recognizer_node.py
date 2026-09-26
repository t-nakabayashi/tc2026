#!/usr/bin/env python3
"""traffic_signal_recognizer ノードの実装モジュール."""

from typing import List, Optional, Sequence, Tuple

import rclpy
from rclpy.node import Node
from tc_diagnostics import DiagnosticReporter, report_alive
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Int32
from tc_perception_msgs.msg import OverlayDetection, PerceptionOverlay
from vision_msgs.msg import Detection2D, Detection2DArray

from traffic_signal_recognizer.signal_recognition_core import (
    DetectionCandidate,
    SignalDecision,
    TrafficSignalRecognitionCore,
)


class TrafficSignalRecognizerNode(Node):
    """YOLO 検出結果から信号の GO/STOP を判定するノード."""

    def __init__(self) -> None:
        super().__init__('traffic_signal_recognizer')

        self._declare_parameters()
        self._load_parameters()

        self.core = TrafficSignalRecognitionCore(
            confidence_threshold=self.confidence_threshold,
            judge_count=self.judge_count,
            go_status=self.go_status,
            stop_status=self.stop_status,
            unknown_class_id=self.unknown_class_id,
            green_class_ids=self.green_class_ids,
            red_class_ids=self.red_class_ids,
            green_class_names=self.green_class_names,
            red_class_names=self.red_class_names,
            hold_go=self.hold_go,
        )

        self.enabled = False

        self.create_subscription(Int32, self.recog_flag_topic, self._recog_flag_callback, 10)
        self.create_subscription(
            Detection2DArray,
            self.detections_topic,
            self._detections_callback,
            10,
        )

        self.sig_recog_publisher = self.create_publisher(Int32, self.sig_recog_topic, 10)
        # 表示用の重畳データは取りこぼしを許容し、road_blockage_detector 側の
        # overlay と QoS を揃える（購読側が両者を同一設定で扱えるようにする）。
        # 制御経路へ渡す判定値は sig_recog_topic 側で RELIABLE に配信する。
        self.overlay_publisher = self.create_publisher(
            PerceptionOverlay,
            self.overlay_topic,
            QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT),
        )

        # 自己申告診断。生存はreporterのタイマーが回ることで表される。
        self._diagnostics = DiagnosticReporter(self)
        report_alive(self._diagnostics)

        self.get_logger().info(
            'traffic_signal_recognizer を起動しました。'
            f' recog_flag={self.recog_flag_topic}, detections={self.detections_topic}, '
            f'sig_recog={self.sig_recog_topic}, overlay={self.overlay_topic}'
        )

    def _declare_parameters(self) -> None:
        """ノードが使用するパラメータを宣言する."""

        self.declare_parameter('recog_flag_topic', '/recog_flag')
        self.declare_parameter('detections_topic', '/perception/traffic_signal/detections')
        self.declare_parameter('sig_recog_topic', '/sig_recog')
        self.declare_parameter('overlay_topic', '/perception/traffic_signal/overlay')
        self.declare_parameter('confidence_threshold', 0.8)
        self.declare_parameter('judge_count', 3)
        self.declare_parameter('go_status', 1)
        self.declare_parameter('stop_status', 2)
        self.declare_parameter('unknown_class_id', 99)
        self.declare_parameter('green_class_ids', [1])
        self.declare_parameter('red_class_ids', [0])
        self.declare_parameter('green_class_names', ['green'])
        self.declare_parameter('red_class_names', ['red'])
        self.declare_parameter('class_names', ['red', 'green'])
        self.declare_parameter('hold_go', False)
        self.declare_parameter('publish_stop_when_disabled', False)

    def _load_parameters(self) -> None:
        """宣言済みパラメータを読み込む."""

        self.recog_flag_topic = self._get_string_parameter('recog_flag_topic')
        self.detections_topic = self._get_string_parameter('detections_topic')
        self.sig_recog_topic = self._get_string_parameter('sig_recog_topic')
        self.overlay_topic = self._get_string_parameter('overlay_topic')
        self.confidence_threshold = self._get_double_parameter('confidence_threshold')
        self.judge_count = self._get_int_parameter('judge_count')
        self.go_status = self._get_int_parameter('go_status')
        self.stop_status = self._get_int_parameter('stop_status')
        self.unknown_class_id = self._get_int_parameter('unknown_class_id')
        self.green_class_ids = self._get_int_array_parameter('green_class_ids')
        self.red_class_ids = self._get_int_array_parameter('red_class_ids')
        self.green_class_names = self._get_string_array_parameter('green_class_names')
        self.red_class_names = self._get_string_array_parameter('red_class_names')
        self.class_names = self._get_string_array_parameter('class_names')
        self.hold_go = self._get_bool_parameter('hold_go')
        self.publish_stop_when_disabled = self._get_bool_parameter('publish_stop_when_disabled')

    def _get_string_parameter(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _get_double_parameter(self, name: str) -> float:
        return self.get_parameter(name).get_parameter_value().double_value

    def _get_int_parameter(self, name: str) -> int:
        return self.get_parameter(name).get_parameter_value().integer_value

    def _get_bool_parameter(self, name: str) -> bool:
        return self.get_parameter(name).get_parameter_value().bool_value

    def _get_int_array_parameter(self, name: str) -> List[int]:
        return [int(value) for value in self.get_parameter(name).value]

    def _get_string_array_parameter(self, name: str) -> List[str]:
        return [str(value) for value in self.get_parameter(name).value]

    def _recog_flag_callback(self, msg: Int32) -> None:
        """recog_flag の最新値に応じて判定有効状態を更新する."""

        next_enabled = int(msg.data) == 1
        if self.enabled == next_enabled:
            return

        self.enabled = next_enabled
        self.core.reset()
        state_text = '有効' if self.enabled else '無効'
        self.get_logger().info(
            f'信号認識を{state_text}にしました。recog_flag={msg.data}'
        )

        if not self.enabled and self.publish_stop_when_disabled:
            self._publish_sig_recog(self.stop_status)

    def _detections_callback(self, msg: Detection2DArray) -> None:
        """Detection2DArray を受信し、信号判定を行う."""

        if not self.enabled:
            return

        candidates = self._extract_candidates(msg.detections)
        decision = self.core.update(candidates)
        self._publish_sig_recog(decision.status)
        self._publish_overlay(msg, decision)

        self.get_logger().debug(
            '信号認識: '
            f'status={decision.status}, class={decision.selected_class_name}, '
            f'score={decision.selected_score:.2f}, history={decision.history}'
        )

    def _extract_candidates(self, detections: Sequence[Detection2D]) -> List[DetectionCandidate]:
        """Detection2D から判定候補を抽出する."""

        candidates: List[DetectionCandidate] = []
        for detection in detections:
            best = self._extract_best_result(detection)
            if best is None:
                continue
            class_id, score = best
            candidates.append(
                DetectionCandidate(
                    class_id=class_id,
                    class_name=self._resolve_class_name(class_id),
                    score=score,
                )
            )
        return candidates

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

    def _resolve_class_name(self, class_id: int) -> str:
        """class id から class name を取得する."""

        if 0 <= class_id < len(self.class_names):
            return self.class_names[class_id]
        return f'class_{class_id}'

    def _publish_sig_recog(self, status: int) -> None:
        """sig_recog を publish する."""

        self.sig_recog_publisher.publish(Int32(data=int(status)))

    def _publish_overlay(self, msg: Detection2DArray, decision: SignalDecision) -> None:
        """重畳表示用の認識結果を publish する.

        本ノードは画像を購読・配信しない。`header` は判定根拠となった
        `Detection2DArray` のものをそのまま引き継ぎ、`yolo_detector` が複製した
        元画像フレームの stamp/frame_id を表示側へ伝える。

        Args:
            msg (Detection2DArray): 判定に使用した検出結果.
            decision (SignalDecision): 判定結果.
        """

        overlay = PerceptionOverlay()
        overlay.header = msg.header
        overlay.source = 'traffic_signal'
        overlay.detections = [
            self._to_overlay_detection(detection) for detection in msg.detections
        ]
        overlay.decision = int(decision.status)
        overlay.decision_text = 'GO' if decision.status == self.go_status else 'STOP'
        overlay.status_note = ''
        self.overlay_publisher.publish(overlay)

    def _to_overlay_detection(self, detection: Detection2D) -> OverlayDetection:
        """Detection2D を表示用の OverlayDetection へ変換する.

        confidence_threshold 未満の検出も `adopted=False` として残し、判定に
        採用されなかった検出も表示側で確認できるようにする。

        Args:
            detection (Detection2D): 変換元の検出.

        Returns:
            OverlayDetection: 表示用の検出 1 件.
        """

        entry = OverlayDetection()
        entry.center_x = float(detection.bbox.center.position.x)
        entry.center_y = float(detection.bbox.center.position.y)
        entry.size_x = float(detection.bbox.size_x)
        entry.size_y = float(detection.bbox.size_y)

        best = self._extract_best_result(detection)
        if best is None:
            entry.label = 'unknown'
            entry.score = 0.0
            entry.adopted = False
            return entry

        class_id, score = best
        entry.label = self._resolve_class_name(class_id)
        entry.score = float(score)
        entry.adopted = score >= self.confidence_threshold
        return entry


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)

    node = TrafficSignalRecognizerNode()
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
