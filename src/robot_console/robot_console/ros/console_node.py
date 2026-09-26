"""robot_console 次期UI向けの ROS 2 Node実装。

`RobotConsoleNode` はROS 2 topic購読・publishのみを担当し、受け取った
メッセージをそのまま `ConsoleCore` の `update_*()` へ委譲する薄いラッパーで
ある（`robot_console_gui_architecture_design.md` 3.1節）。業務ロジック・
状態集約は一切持たない。手動操作系publisherは `ConsoleCore.bind_publishers()`
経由で送信関数として登録し、`ConsoleCore` 自体はROSに依存しない。
"""

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Any, Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message
from rtk_gps_um982_msgs.msg import RtkStatus
from sensor_msgs.msg import Image as ImageMsg
from std_msgs.msg import Bool, Int32, String
from tc_geo_msgs.msg import FusionState, GeoPoseWithQuality
from tc_perception_msgs.msg import PerceptionOverlay
from tc_route_msgs.msg import (
    ActiveTargetLlh,
    DriveModeStatus,
    FollowerState,
    ManagerStatus,
    ObstacleAvoidanceHint,
    Route,
    RouteState,
)

from ..core.camera_overlay import OverlayDetectionView, PerceptionOverlayView
from ..core.console_core import ConsoleCore

DEFAULT_NODE_NAME = 'robot_console_gui'
# フロントカメラの生画像。重畳済み画像ではなくこれを唯一の画像入力とする。
# 先頭にスラッシュを付けないことで launch からの remap を可能にする。
CAMERA_IMAGE_TOPIC = 'usb_cam/image_raw'


def _stamp_to_seconds(stamp: Any) -> Optional[float]:
    """`builtin_interfaces/Time` を秒へ変換する。

    認識結果 (`PerceptionOverlay.header.stamp`) と生画像フレームの対応付けに使う。

    Args:
        stamp (Any): `sec` / `nanosec` を持つ時刻.

    Returns:
        Optional[float]: 秒単位の時刻. 変換できない場合は None.
    """

    try:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9
    except (AttributeError, TypeError, ValueError):
        return None


def _overlay_view_from_msg(msg: PerceptionOverlay) -> PerceptionOverlayView:
    """`PerceptionOverlay` をROS非依存のViewへ変換する。

    `ConsoleCore` はROSメッセージ型に依存しないため、変換は本モジュールで行う
    （architecture_design.md 3.1節）。

    Args:
        msg (PerceptionOverlay): 受信したメッセージ.

    Returns:
        PerceptionOverlayView: `ConsoleCore` へ渡すView.
    """

    return PerceptionOverlayView(
        source=msg.source,
        detections=[
            OverlayDetectionView(
                center_x=detection.center_x,
                center_y=detection.center_y,
                size_x=detection.size_x,
                size_y=detection.size_y,
                label=detection.label,
                score=detection.score,
                adopted=detection.adopted,
            )
            for detection in msg.detections
        ],
        decision=msg.decision,
        decision_text=msg.decision_text,
        status_note=msg.status_note,
        frame_stamp=_stamp_to_seconds(msg.header.stamp),
    )

# route_manager の /active_route はTransient Local（ラッチ配信）で配信される
# （route_manager_node.py の qos_tl()）。購読側が既定のVOLATILEのままだと、
# route_manager の publish 完了より購読の確立が遅れた場合に初回のRouteを
# 二度と受け取れない（DDSのdurability replayはVOLATILE購読には行われない）。
# 起動順に依存せず必ず現在のrouteを受け取れるよう、publisher側と同じQoSで購読する。
_QOS_ACTIVE_ROUTE = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# obstacle_monitor は obstacle_avoidance_hint を BEST_EFFORT で配信する
# （obstacle_monitor_node.py の qos_be_volatile）。購読側が既定のRELIABLEのままだと
# QoS非互換となり接続自体が成立せず、hintを一切受信できない。BEST_EFFORT購読は
# RELIABLE配信元とも互換であるため、購読側をBEST_EFFORTに合わせる。
_QOS_BEST_EFFORT = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# 表示系トピックは配信側のQoSがノードごとに異なる（obstacle_monitor の sensor_viewer と
# usb_cam の画像は RELIABLE、認識ノードの overlay は BEST_EFFORT）。BEST_EFFORT購読は
# どちらとも互換であり、表示用途では取りこぼしも許容できるため一律 BEST_EFFORT とする。
_QOS_IMAGE = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# 健全性監視用の購読QoS。配信様式 stream の既定であり、BEST_EFFORT購読は
# RELIABLE配信元とも互換なため、配信側の設定を調べずに接続が成立する
# （docs/トピック通信規約.md 4.1節）。
_QOS_HEALTH = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class RobotConsoleNode(Node):
    """ROS 2 topicを購読し `ConsoleCore` へ橋渡しするだけのNode。"""

    def __init__(self, core: ConsoleCore, *, node_name: str = DEFAULT_NODE_NAME) -> None:
        super().__init__(node_name)
        self._core = core
        core.survey_conflict_check = lambda: set(self.get_node_names()) & {
            'ypspur_node', 'drive_cmd_mux_node', 'manual_teleop_node',
            'fastlio_mapping', 'gnss_lio_fusion', 'route_survey'}
        self._survey_pub = self.create_publisher(String, '/route_survey/command', 10)
        core.survey_publisher = lambda value: self._survey_pub.publish(String(data=value))
        self.create_subscription(String, '/route_survey/status', core.update_survey_status, 10)
        # GNSS診断は実機のUM982ドライバがnode private名で配信する。起動側が
        # `gnss_namespace` に応じてremapするため、ここでは相対名で購読する。
        self.create_subscription(String, 'rtk_gps/ntrip_status', core.update_ntrip_status, 10)
        self.create_subscription(Bool, '/fusion/gnss_dropout_active', self._core.update_gnss_dropout, 10)
        self.create_subscription(
            FusionState, '/fusion/status', self._core.update_fusion_status, 10
        )

        self.create_subscription(RouteState, 'route_state', self._core.update_route_state, 10)
        self.create_subscription(
            ManagerStatus, 'manager_status', self._core.update_manager_status, 10
        )
        self.create_subscription(
            Route, 'active_route', self._core.update_route, _QOS_ACTIVE_ROUTE
        )
        self.create_subscription(
            FollowerState, 'follower_state', self._core.update_follower_state, 10
        )
        self.create_subscription(
            ObstacleAvoidanceHint,
            'obstacle_avoidance_hint',
            self._core.update_obstacle_hint,
            _QOS_BEST_EFFORT,
        )
        self.create_subscription(
            PoseStamped, 'active_target', self._core.update_active_target, 10
        )
        # 目標の緯度経度は geo_pose_converter（route_geo_projector_node）が
        # ENUから変換して配信する。`robot_console` は変換を持たず受け取るだけとする。
        self.create_subscription(
            ActiveTargetLlh,
            'route/active_target_llh',
            self._core.update_active_target_llh,
            10,
        )
        self.create_subscription(
            DriveModeStatus, 'drive_mode_status', self._core.update_drive_mode_status, 10
        )
        self.create_subscription(Twist, 'cmd_vel', self._core.update_cmd_vel, 10)
        self.create_subscription(
            Twist, 'cmd_vel/autonomous', self._core.update_cmd_vel_autonomous, 10
        )
        odom_topic = self.resolve_topic_name('odom')
        self.create_subscription(
            Odometry, 'odom', lambda msg: core.update_odom(msg, topic=odom_topic), 10
        )
        # 自身がpublishした値もDDS経由でエコーバックされるため、GUI送信の結果確認と
        # GUI以外（信号認識ノード・road_blockage_detector等）からの送信検知の双方を
        # これらの購読で扱う。
        self.create_subscription(Bool, 'manual_start', self._core.update_manual_start, 10)
        self.create_subscription(Int32, 'sig_recog', self._core.update_sig_recog, 10)
        self.create_subscription(Bool, 'road_blocked', self._core.update_road_blocked, 10)
        self.create_subscription(
            PoseWithCovarianceStamped,
            'localization/pose_enu',
            self._core.update_pose_enu,
            10,
        )
        self.create_subscription(
            GeoPoseWithQuality,
            'localization/pose_llh',
            self._core.update_pose_llh,
            10,
        )
        self.create_subscription(
            RtkStatus, 'rtk_gps/rtk_status', self._core.update_gps_status, 10
        )
        self.create_subscription(
            ImageMsg,
            'sensor_viewer',
            lambda msg: core.update_sensor_image(
                'sensor_viewer', 'Sensor Viewer', '/sensor_viewer', msg
            ),
            _QOS_IMAGE,
        )
        # 認識ノードは重畳画像を配信しない。生画像 1 本と認識結果を受け取り、
        # 重畳は ConsoleCore が行う（core/camera_overlay.py）。
        camera_topic = self.resolve_topic_name(CAMERA_IMAGE_TOPIC)
        self.create_subscription(
            ImageMsg,
            'usb_cam/image_raw',
            lambda msg: core.update_camera_image(
                camera_topic, msg, _stamp_to_seconds(msg.header.stamp)
            ),
            _QOS_IMAGE,
        )
        self.create_subscription(
            PerceptionOverlay,
            'perception/traffic_signal/overlay',
            lambda msg: core.update_perception_overlay(_overlay_view_from_msg(msg)),
            _QOS_IMAGE,
        )
        self.create_subscription(
            PerceptionOverlay,
            'perception/road_blockage/overlay',
            lambda msg: core.update_perception_overlay(_overlay_view_from_msg(msg)),
            _QOS_IMAGE,
        )

        self.create_subscription(
            DiagnosticArray, '/diagnostics', core.update_diagnostics, _QOS_HEALTH
        )
        self._subscribe_health_topics(core)

        # 先頭にスラッシュを付けないことで launch からの remap を可能にする。
        self._manual_pub = self.create_publisher(Bool, 'manual_start', 10)
        self._sig_pub = self.create_publisher(Int32, 'sig_recog', 10)
        self._road_pub = self.create_publisher(Bool, 'road_blocked', 10)
        self._obstacle_hint_pub = self.create_publisher(
            ObstacleAvoidanceHint, 'obstacle_avoidance_hint', 10
        )
        self._frame_image_path_pub = self.create_publisher(String, '/frame_image_path', 10)

        core.bind_publishers(
            manual_start=lambda value: self._manual_pub.publish(Bool(data=value)),
            sig_recog=lambda value: self._sig_pub.publish(Int32(data=value)),
            road_blocked=lambda value: self._road_pub.publish(Bool(data=value)),
            obstacle_hint=lambda active, clearance, left, right: self._obstacle_hint_pub.publish(
                ObstacleAvoidanceHint(
                    front_blocked=active,
                    front_clearance_m=clearance,
                    left_offset_m=left,
                    right_offset_m=right,
                )
            ),
            frame_image=lambda path: self._frame_image_path_pub.publish(String(data=path)),
        )


    def _subscribe_health_topics(self, core: ConsoleCore) -> None:
        """`health_topics` を健全性判定専用に購読する。

        内容は解釈せず受信事実だけを記録するため `raw=True` で購読し、
        デシリアライズを回避する。`/scan` のような高レートtopicでも負荷を
        無視できる（`docs/ノード健全性監視設計.md` 3.4節）。

        表示のために既に購読しているtopicにも、健全性用の購読を別に張る。
        表示側の購読は remap 対象の相対名で、鮮度キーも表示都合の名前
        （`obstacle_hint`、`image.sensor_viewer` など）であり、profile定義の
        絶対topic名とは一対一に対応しないためである。購読を共用するには
        表示側の全経路に健全性キーの記録を追加する必要があり、記録漏れが
        そのまま誤った異常判定になる。重複する購読は raw・BEST_EFFORT・
        depth 1 であり、コストよりも判定の確実性を優先する。
        """

        for health_topic in core.health_topic_definitions():
            try:
                message_type = get_message(health_topic.type)
            except (AttributeError, ModuleNotFoundError, ValueError) as exc:
                # 型を解決できないtopicは購読を作れない。起動は継続し、当該topicは
                # 未受信（UNKNOWN）のまま扱う。
                self.get_logger().error(
                    f'health_topic の型を解決できません: {health_topic.type} ({exc})'
                )
                continue
            self.create_subscription(
                message_type,
                health_topic.topic,
                lambda _raw, topic=health_topic.topic: core.mark_health_topic_received(topic),
                _QOS_HEALTH,
                raw=True,
            )


@dataclass
class RosHandle:
    """バックグラウンドで動かしているROS 2 Nodeへのハンドル。"""

    node: RobotConsoleNode
    thread: threading.Thread

    def stop(self) -> None:
        """Nodeを破棄しROSスレッドを安全に停止する。"""

        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self.thread.join(timeout=5.0)


def start_ros_thread(
    core: ConsoleCore,
    *,
    node_name: str = DEFAULT_NODE_NAME,
    ros_args: Optional[list[str]] = None,
) -> RosHandle:
    """`RobotConsoleNode` を生成し、別スレッドでexecutorを回す。

    Qtイベントループ（`ui_qt_main.py`）やHTTPサーバスレッド（`web_main.py`）と
    rclpy executorを分離するために用いる（architecture_design.md 14.1節）。
    `ros_args` はROS初期化時に渡す引数列。省略時は従来どおりsys.argvを使う。
    """

    if not rclpy.ok():
        rclpy.init(args=ros_args)
    node = RobotConsoleNode(core, node_name=node_name)

    def _spin() -> None:
        try:
            rclpy.spin(node)
        except Exception:  # pragma: no cover - shutdown時の例外は無視する
            pass

    thread = threading.Thread(target=_spin, daemon=True)
    thread.start()
    return RosHandle(node=node, thread=thread)
