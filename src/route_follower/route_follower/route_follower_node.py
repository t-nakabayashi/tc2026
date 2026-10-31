#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""route_follower_node（ROS2ラッパー層）

follower_core.FollowerCore を内部に保持し、
ROS2のPublisher / Subscriber / Timer / ServiceClient を仲介する。

役割：
    - ROSメッセージを Core の内部構造体（Pose, Waypoint, Route, HintSample）へ変換。
    - Core.tick() で得た出力を ROSメッセージ（PoseStamped, FollowerState）としてpublish。
    - /report_stuck サービスの呼び出しを管理（CoreからWAITING_REROUTE遷移時に発火）。
"""

from __future__ import annotations

import time
import sys
import math
import threading
from pathlib import Path
from typing import Optional, Tuple
import rclpy
from rclpy.node import Node
from tc_diagnostics import DiagnosticReporter, report_alive
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose, Quaternion
from std_msgs.msg import Header, Bool, Int32
from tc_route_msgs.msg import (  # type: ignore
    AvoidanceAdvice,
    FollowerState,
    ObstacleAvoidanceHint,
    Route,
    Waypoint,
)
from tc_route_msgs.action import RequestAvoidanceAdvice  # type: ignore
from tc_route_msgs.srv import ReportStuck  # type: ignore
from tc_perception_msgs.msg import ObstacleLayers  # type: ignore
from rclpy.action import ActionClient

# 可変ルート探索（外部モジュール）
_THIS_DIR = Path(__file__).resolve().parent
if str(_THIS_DIR) not in sys.path:
    sys.path.append(str(_THIS_DIR))

from follower_core import (
    AdviceDecision,
    FollowerCore,
    Pose as CorePose,
    Waypoint as CoreWaypoint,
    Route as CoreRoute,
    HintSample,
    FollowerStatus,
)


# ============================================================
# QoS設定ヘルパ
# ============================================================

def qos_transient_local(depth: int = 1) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def qos_volatile(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def qos_best_effort(depth: int = 5) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


# ============================================================
# メインノード
# ============================================================

class RouteFollowerNode(Node):
    """route_follower（ROS2ラッパー層）."""

    def __init__(self) -> None:
        super().__init__("route_follower")

        # パラメータ宣言
        self.declare_parameter("arrival_threshold", 0.6)
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("resend_interval_sec", 1.0)
        self.declare_parameter("start_immediately", True)
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("avoid_back_offset_m", 0.5)
        self.declare_parameter("road_blocked_confirmation_sec", 5.0)

        # AI 助言。既定は無効で、有効化しない限り従来どおりの回避だけで走る。
        self.declare_parameter("use_ai_advice", False)
        self.declare_parameter("advice_trigger_hold_sec", 3.0)
        self.declare_parameter("advice_wait_limit_sec", 3.0)
        self.declare_parameter("advice_cooldown_sec", 20.0)
        self.declare_parameter("advice_cooldown_m", 3.0)
        self.declare_parameter("advice_budget_per_min", 4)
        self.declare_parameter("advice_budget_total", 30)
        self.declare_parameter("max_obstacle_wait_sec", 30.0)
        self.declare_parameter("layer_clearance_timeout_sec", 2.0)

        arrival_threshold = float(self.get_parameter("arrival_threshold").value)
        control_rate = float(self.get_parameter("control_rate_hz").value)
        if control_rate <= 0.0:
            self.get_logger().warn("control_rate_hz が0以下のため20Hzを使用します。")
            control_rate = 20.0
        resend_interval = float(self.get_parameter("resend_interval_sec").value)
        if resend_interval <= 0.0:
            self.get_logger().warn("resend_interval_sec が0以下のため1.0秒を使用します。")
            resend_interval = 1.0
        self.start_immediately: bool = bool(self.get_parameter("start_immediately").value)
        self.target_frame: str = str(self.get_parameter("target_frame").value)
        avoid_back_offset = float(self.get_parameter("avoid_back_offset_m").value)
        if avoid_back_offset < 0.0:
            self.get_logger().warn("avoid_back_offset_m が負のため0.0mを使用します。")
            avoid_back_offset = 0.0
        road_blocked_confirmation = float(
            self.get_parameter("road_blocked_confirmation_sec").value
        )
        if road_blocked_confirmation < 0.0:
            self.get_logger().warn(
                "road_blocked_confirmation_sec が負のため0.0秒を使用します。",
            )
            road_blocked_confirmation = 0.0

        self.core = FollowerCore(self.get_logger())
        self.core.arrival_threshold = arrival_threshold
        self.core.control_rate_hz = control_rate
        self.core.republish_target_hz = 1.0 / resend_interval
        self.core.start_immediately = self.start_immediately
        self.core.avoid_back_offset_m = avoid_back_offset
        self.core.road_blocked_confirmation_sec = road_blocked_confirmation
        self.core.use_ai_advice = bool(self.get_parameter("use_ai_advice").value)
        self.core.advice_trigger_hold_sec = float(
            self.get_parameter("advice_trigger_hold_sec").value)
        self.core.advice_wait_limit_sec = float(
            self.get_parameter("advice_wait_limit_sec").value)
        self.core.advice_cooldown_sec = float(
            self.get_parameter("advice_cooldown_sec").value)
        self.core.advice_cooldown_m = float(self.get_parameter("advice_cooldown_m").value)
        self.core.advice_budget_per_min = int(
            self.get_parameter("advice_budget_per_min").value)
        self.core.advice_budget_total = int(
            self.get_parameter("advice_budget_total").value)
        self.core.max_obstacle_wait_sec = float(
            self.get_parameter("max_obstacle_wait_sec").value)
        self.core.layer_clearance_timeout_sec = float(
            self.get_parameter("layer_clearance_timeout_sec").value)
        # 自己申告診断。生存はreporterのタイマーが回ることで表される。
        self._diagnostics = DiagnosticReporter(self)
        report_alive(self._diagnostics)
        self.get_logger().info("route_follower_node started.")

        # QoS設定
        self.qos_tl = qos_transient_local()
        self.qos_vol = qos_volatile()
        self.qos_be = qos_best_effort()

        # Pub/Sub設定
        active_route_topic = 'active_route'
        pose_enu_topic = 'localization/pose_enu'
        obstacle_hint_topic = 'obstacle_avoidance_hint'
        manual_start_topic = 'manual_start'
        signal_recognition_topic = 'sig_recog'
        recog_flag_topic = 'recog_flag'
        road_block_topic = 'road_blocked'
        active_target_topic = 'active_target'
        follower_state_topic = 'follower_state'
        report_stuck_service = 'report_stuck'
        obstacle_layers_topic = 'obstacle/layers'
        advice_action_name = 'request_avoidance_advice'

        self.sub_route = self.create_subscription(Route, active_route_topic, self._on_route, self.qos_tl)
        self.sub_pose = self.create_subscription(
            PoseWithCovarianceStamped, pose_enu_topic, self._on_pose, self.qos_vol
        )
        self.sub_hint = self.create_subscription(
            ObstacleAvoidanceHint, obstacle_hint_topic, self._on_hint, self.qos_be
        )
        self.sub_manual = self.create_subscription(Bool, manual_start_topic, self._on_manual_start, self.qos_vol)
        self.sub_sig = self.create_subscription(Int32, signal_recognition_topic, self._on_sig_recog, self.qos_vol)
        # road_blockedトピックは運用ツールなど複数の発行元から送信されるが、
        # いずれもデフォルトのVolatile QoSで運用されている。
        # 本ノードがTransient Localを要求するとQoS不整合で購読できないため、
        # ここではVolatileを使用しCore側でラッチ相当の保持を行う。
        self.sub_road_block = self.create_subscription(
            Bool, road_block_topic, self._on_road_blocked, self.qos_vol
        )

        # 層スライスの観測値は回避幅の上限に使う。未受信でも従来どおり動作する。
        self.sub_layers = self.create_subscription(
            ObstacleLayers, obstacle_layers_topic, self._on_obstacle_layers, self.qos_be
        )

        self.pub_target = self.create_publisher(PoseStamped, active_target_topic, self.qos_vol)
        self.pub_state = self.create_publisher(FollowerState, follower_state_topic, self.qos_vol)
        # recog_flag は値変化時のみ送るため、標準のVolatile QoSを使用する。
        self.pub_recog_flag = self.create_publisher(Int32, recog_flag_topic, self.qos_vol)

        self.cli_report_stuck = self.create_client(ReportStuck, report_stuck_service)
        self.advice_client = ActionClient(self, RequestAvoidanceAdvice, advice_action_name)

        # リマップ適用後の名称をログ用に保存
        self.active_route_topic = self._resolve_topic_name(active_route_topic)
        self.pose_enu_topic = self._resolve_topic_name(pose_enu_topic)
        self.obstacle_hint_topic = self._resolve_topic_name(obstacle_hint_topic)
        self.manual_start_topic = self._resolve_topic_name(manual_start_topic)
        self.signal_recognition_topic = self._resolve_topic_name(signal_recognition_topic)
        self.recog_flag_topic = self._resolve_topic_name(recog_flag_topic)
        self.road_block_topic = self._resolve_topic_name(road_block_topic)
        self.active_target_topic = self._resolve_topic_name(active_target_topic)
        self.follower_state_topic = self._resolve_topic_name(follower_state_topic)
        self.report_stuck_service_name = self._resolve_service_name(report_stuck_service)
        self.obstacle_layers_topic = self._resolve_topic_name(obstacle_layers_topic)
        self.advice_action_name = advice_action_name

        # 助言要求の進行管理。結果は callback で受け取り、次の tick で Core へ渡す。
        self._advice_lock = threading.Lock()
        self._advice_goal_handle = None
        self._advice_result: Optional[AdviceDecision] = None
        self._advice_reason: str = ""
        self._advice_result_ready: bool = False
        self._advice_deadline: Optional[float] = None

        self._report_lock = threading.Lock()
        self._report_stuck_pending: bool = False
        self._report_stuck_result: Optional[ReportStuck.Response] = None
        self._report_stuck_result_ready: bool = False

        # Timer
        self._last_pub_target_pose = None
        self._last_pub_time = 0.0
        self._last_pub_stamp: Optional[Time] = None
        self._last_pub_target_identity: Optional[Tuple[int, int, str]] = None
        self._republish_interval = resend_interval
        timer_period = 1.0 / control_rate
        self.timer = self.create_timer(timer_period, self._on_timer)

        # follower_stateの周期ログ用タイムスタンプを初期化する。
        self._last_state_log_time: float = 0.0
        self._latest_pose_stamped: Optional[PoseStamped] = None
        self._last_recog_flag: Optional[int] = None
        self._publish_recog_flag(0, log=False, force=True)

    def _resolve_topic_name(self, name: str) -> str:
        """リマップ適用後のトピック名を取得する。"""
        try:
            return self.resolve_topic_name(name)
        except AttributeError:
            return name

    def _resolve_service_name(self, name: str) -> str:
        """リマップ適用後のサービス名を取得する。"""
        try:
            return self.resolve_service_name(name)
        except AttributeError:
            return name

    # ========================================================
    # Callback群
    # ========================================================

    def _on_route(self, msg: Route) -> None:
        """経路トピック受信時の処理."""
        if not msg.waypoints:
            self.get_logger().warn(f"{self.active_route_topic}: 空のrouteを無視します。")
            return

        wp_list = []
        for w in msg.waypoints:
            wp_list.append(
                CoreWaypoint(
                    label=w.label,
                    pose=CorePose(
                        w.pose.position.x,
                        w.pose.position.y,
                        self._yaw_from_quat(w.pose.orientation),
                        orientation_x=float(getattr(w.pose.orientation, "x", 0.0)),
                        orientation_y=float(getattr(w.pose.orientation, "y", 0.0)),
                        orientation_z=float(getattr(w.pose.orientation, "z", 0.0)),
                        orientation_w=float(getattr(w.pose.orientation, "w", 1.0)),
                    ),
                    line_stop=bool(getattr(w, "line_stop", False)),
                    signal_stop=bool(getattr(w, "signal_stop", False)),
                    left_open=float(getattr(w, "left_open", 0.0)),
                    right_open=float(getattr(w, "right_open", 0.0)),
                )
            )

        route = CoreRoute(
            version=int(getattr(msg, "version", -1)),
            waypoints=wp_list,
            start_index=int(getattr(msg, "start_index", 0)),
            start_waypoint_label=str(getattr(msg, "start_waypoint_label", "")),
        )
        self.core.update_route(route)
        if self.start_immediately:
            self.get_logger().info("start_immediately が有効のため自動開始を指示します。")
            self.core.update_control_inputs(manual_start=True)

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        """現在位置Poseトピック受信時の処理."""
        # PoseWithCovarianceStampedをPoseStampedに変換
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        self._latest_pose_stamped = self._copy_pose_stamped(pose_stamped)
        pose = CorePose(
            pose_stamped.pose.position.x,
            pose_stamped.pose.position.y,
            self._yaw_from_quat(pose_stamped.pose.orientation),
            orientation_x=float(getattr(pose_stamped.pose.orientation, "x", 0.0)),
            orientation_y=float(getattr(pose_stamped.pose.orientation, "y", 0.0)),
            orientation_z=float(getattr(pose_stamped.pose.orientation, "z", 0.0)),
            orientation_w=float(getattr(pose_stamped.pose.orientation, "w", 1.0)),
        )
        self.core.update_pose(pose)

    def _on_hint(self, msg: ObstacleAvoidanceHint) -> None:
        """障害物回避ヒントトピック受信時の処理."""
        sample = HintSample(
            t=time.time(),
            front_blocked=bool(msg.front_blocked),
            front_clearance=float(msg.front_clearance_m),
            left_offset=float(msg.left_offset_m),
            right_offset=float(msg.right_offset_m),
        )
        self.core.update_hint(sample)

    def _on_manual_start(self, msg: Bool) -> None:
        """manual_start(Bool) の最新値を Core に渡す。"""
        self.core.update_control_inputs(manual_start=bool(msg.data))

    def _on_sig_recog(self, msg: Int32) -> None:
        """sig_recog(Int32) の最新値を Core に渡す"""
        self.core.update_control_inputs(sig_recog=int(msg.data))

    def _on_road_blocked(self, msg: Bool) -> None:
        """road_blocked(Bool) の最新値を Core に渡す。"""
        self.core.update_control_inputs(road_blocked=bool(msg.data))

    # ========================================================
    # 周期処理
    # ========================================================

    def _on_timer(self) -> None:
        """設定された制御周期ごとに Core.tick() を実行する."""
        previous_status = self.core.status
        output = self.core.tick()

        # active_target 出力
        self._handle_target_publish(output)

        # follower_state 出力
        self._handle_state_publish(output)

        # recog_flag 出力
        self._update_recog_flag(previous_status)

        # AI助言の結果反映と要求送信
        self._process_advice_result()
        self._maybe_request_advice()

        # /report_stuck応答の反映
        self._process_report_stuck_result()

        # stuck報告判定
        if self.core.status == self.core.status.WAITING_REROUTE:
            self._handle_stuck_report()

    def _on_obstacle_layers(self, msg: ObstacleLayers) -> None:
        """層スライスの観測値から、全層で共通に使える左右の余裕を Core へ渡す.

        点が 1 点も無い層は未観測であり、障害物が無いことを意味しないため最小値の
        算出から外す。全層が未観測の場合は余裕なしとして扱わず、Core 側が採取幅と
        大域上限だけで判断できるよう、値を更新しない。
        """
        observed = [layer for layer in msg.layers if layer.point_count > 0]
        if not observed:
            return
        left = min(float(layer.available_left_offset_m) for layer in observed)
        right = min(float(layer.available_right_offset_m) for layer in observed)
        self.core.update_layer_clearance(left, right)

    def _maybe_request_advice(self) -> None:
        """Core が助言を求めている場合に action の goal を送る."""
        request = self.core.take_advice_request()
        if request is None:
            return
        if not self.advice_client.server_is_ready():
            self.get_logger().warn("avoidance_advisor が応答しないため従来の回避を使う。")
            self.core.submit_advice(None, "server_unavailable")
            return

        goal = RequestAvoidanceAdvice.Goal()
        goal.waypoint_label = request.waypoint_label
        goal.waypoint_left_open_m = float(request.waypoint_left_open_m)
        goal.waypoint_right_open_m = float(request.waypoint_right_open_m)
        goal.front_clearance_m = float(request.front_clearance_m)
        goal.hint_left_offset_m = float(request.hint_left_offset_m)
        goal.hint_right_offset_m = float(request.hint_right_offset_m)
        goal.min_offset_m = float(request.min_offset_m)
        goal.max_offset_m = float(request.max_offset_m)

        with self._advice_lock:
            self._advice_deadline = time.time() + self.core.advice_wait_limit_sec
        future = self.advice_client.send_goal_async(goal)
        future.add_done_callback(self._on_advice_goal_response)

    def _on_advice_goal_response(self, future) -> None:
        """goal の受理・拒否を受け取る."""
        try:
            goal_handle = future.result()
        except Exception as error:  # 通信失敗は従来の回避で続行する
            self._store_advice_result(None, f"send_failed:{error}")
            return
        if not goal_handle.accepted:
            self._store_advice_result(None, "rejected")
            return
        with self._advice_lock:
            self._advice_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self._on_advice_result)

    def _on_advice_result(self, future) -> None:
        """助言の結果を受け取り、次の tick で Core へ渡せるよう保持する."""
        try:
            result = future.result().result
        except Exception as error:
            self._store_advice_result(None, f"result_failed:{error}")
            return
        if not result.advice_valid:
            self._store_advice_result(None, result.failure_reason or "invalid")
            return
        self._store_advice_result(self._to_advice_decision(result.advice), "")

    def _to_advice_decision(self, advice: AvoidanceAdvice) -> AdviceDecision:
        """ROS メッセージを Core の助言表現へ写す."""
        if advice.action == AvoidanceAdvice.ACTION_WAIT:
            action = "wait"
        elif advice.action == AvoidanceAdvice.ACTION_AVOID:
            action = "avoid"
        else:
            action = ""
        if advice.side == AvoidanceAdvice.SIDE_LEFT:
            side = "L"
        elif advice.side == AvoidanceAdvice.SIDE_RIGHT:
            side = "R"
        else:
            side = ""
        return AdviceDecision(
            action=action,
            side=side,
            offset_m=float(advice.offset_m),
            wait_sec=float(advice.wait_sec),
            is_moving=bool(advice.is_moving),
            confidence=float(advice.confidence),
            obstacle_class=str(advice.obstacle_class),
            request_id=str(advice.request_id),
        )

    def _store_advice_result(self, decision: Optional[AdviceDecision],
                             reason: str) -> None:
        with self._advice_lock:
            self._advice_result = decision
            self._advice_reason = reason
            self._advice_result_ready = True
            self._advice_goal_handle = None
            self._advice_deadline = None

    def _process_advice_result(self) -> None:
        """受領済みの助言を Core へ渡す。待機上限を過ぎた要求は取り消す."""
        with self._advice_lock:
            ready = self._advice_result_ready
            decision = self._advice_result
            reason = self._advice_reason
            deadline = self._advice_deadline
            goal_handle = self._advice_goal_handle
            if ready:
                self._advice_result = None
                self._advice_reason = ""
                self._advice_result_ready = False
        if ready:
            self.core.submit_advice(decision, reason)
            return
        if deadline is None or time.time() < deadline:
            return
        # 待機上限を過ぎた。goal を取り消して従来の回避へ退避する。
        self.get_logger().warn("AI助言が待機上限内に返らないため従来の回避を使う。")
        if goal_handle is not None:
            goal_handle.cancel_goal_async()
        with self._advice_lock:
            self._advice_goal_handle = None
            self._advice_deadline = None
            self._advice_result = None
            self._advice_result_ready = False
        self.core.submit_advice(None, "timeout")

    def _process_report_stuck_result(self) -> None:
        with self._report_lock:
            if not self._report_stuck_result_ready:
                return
            res = self._report_stuck_result
            self._report_stuck_result = None
            self._report_stuck_result_ready = False

        if res is None:
            self.get_logger().warn("report_stuck call returned no response.")
            return

        self.get_logger().info(
            f"{self.report_stuck_service_name} result: decision_code={res.decision_code}, "
            f"note='{res.note}', offset_hint={res.offset_hint:.2f}"
        )

        if res.decision_code == ReportStuck.Response.DECISION_FAILED:
            note = res.note or "avoidance_failed"
            self.core.notify_reroute_failed(note)

    def _handle_target_publish(self, output):
        """active_targetのデバウンス付きpublish"""
        now_ros = self.get_clock().now()
        now_sec = now_ros.seconds_nanoseconds()[0] + now_ros.seconds_nanoseconds()[1] * 1e-9
        pose = output.target_pose
        if pose is None:
            return
        state = output.state or {}
        route_version = int(
            state.get("route_version", getattr(self.core, "route_version", -1))
        )
        active_index = int(
            state.get("active_waypoint_index", getattr(self.core, "index", -1))
        )
        default_label = self.core.get_current_waypoint_label()
        active_label = str(state.get("active_waypoint_label", default_label))
        target_identity: Tuple[int, int, str] = (route_version, active_index, active_label)
        identity_changed = self._last_pub_target_identity != target_identity
        pose_changed = (
            self._last_pub_target_pose is None
            or self._euclid_diff(pose, self._last_pub_target_pose) > 1e-6
        )
        is_new_directive = identity_changed or pose_changed
        time_elapsed = (now_sec - self._last_pub_time) >= self._republish_interval
        if not is_new_directive and not time_elapsed:
            return
        pose_msg = PoseStamped()
        pose_msg.header = Header()
        if is_new_directive:
            stamp_msg = now_ros.to_msg()
        else:
            # 同じターゲットを周期再送する場合は、header.stampを更新しない。
            stamp_msg = self._last_pub_stamp or now_ros.to_msg()
        pose_msg.header.stamp = stamp_msg
        pose_msg.header.frame_id = self.target_frame
        pose_msg.pose = self._pose_to_msg(pose)
        self.pub_target.publish(pose_msg)
        self._last_pub_target_pose = pose
        self._last_pub_time = now_sec
        self._last_pub_stamp = stamp_msg
        self._last_pub_target_identity = target_identity

    def _handle_state_publish(self, output):
        """FollowerState メッセージを生成・publish"""
        if output.state is None:
            return

        state = output.state
        msg = FollowerState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.route_version = int(state["route_version"])
        msg.state = str(state["status"])
        msg.active_waypoint_index = int(state["active_waypoint_index"])
        msg.active_waypoint_label = str(state.get("active_waypoint_label", ""))
        msg.front_blocked = bool(state.get("front_blocked", False))
        msg.front_clearance_m = float(state.get("front_clearance_m", 0.0))
        msg.left_offset_m = float(state.get("left_offset_m", 0.0))
        msg.right_offset_m = float(state.get("right_offset_m", 0.0))
        msg.avoidance_attempt_count = int(state["avoid_count"])
        msg.last_stagnation_reason = str(state["reason"])
        msg.segment_length_m = float(state.get("segment_length_m", 0.0))
        msg.active_target_distance_m = float(self._compute_active_target_distance(output))
        self.pub_state.publish(msg)

        now_sec = time.monotonic()
        if now_sec - self._last_state_log_time >= 1.0:
            self.get_logger().info(
                f"[Node] publish {self.follower_state_topic}: state={msg.state}, index={msg.active_waypoint_index}, "
                f"route_ver={msg.route_version}, avoid_count={msg.avoidance_attempt_count}"
            )
            self._last_state_log_time = now_sec

    def _update_recog_flag(self, previous_status: FollowerStatus) -> None:
        """現在の状態とWP属性に応じてrecog_flagをpublishする。"""
        if (
            previous_status == FollowerStatus.IDLE
            and self.core.status == FollowerStatus.RUNNING
        ):
            # RUNNINGへ移行したタイミングで起動直後の0を再送する。
            self._publish_recog_flag(0, force=True)
        desired_flag = self._compute_recog_flag()
        self._publish_recog_flag(desired_flag)

    def _compute_recog_flag(self) -> int:
        """信号停止WP待機中のみ1を返し、それ以外は0を返す。"""
        wp = self.core.get_current_waypoint()
        if (
            wp is not None
            and self.core.status == FollowerStatus.WAITING_STOP
            and wp.signal_stop
        ):
            return 1
        return 0

    def _publish_recog_flag(self, flag: int, log: bool = True, force: bool = False) -> None:
        """recog_flagの変化時のみ、もしくはforce指定時に発行する。"""
        if not force and self._last_recog_flag == flag:
            return
        msg = Int32()
        msg.data = int(flag)
        self.pub_recog_flag.publish(msg)
        self._last_recog_flag = int(flag)
        if log:
            self.get_logger().info(
                f"[Node] publish {self.recog_flag_topic}: flag={flag}"
            )

    # ========================================================
    # stuck報告
    # ========================================================

    def _handle_stuck_report(self) -> None:
        """WAITING_REROUTE遷移時に report_stuck サービスを呼び出す."""
        if not self.cli_report_stuck.service_is_ready():
            self.get_logger().warn(f"{self.report_stuck_service_name} not ready.")
            return

        with self._report_lock:
            if self._report_stuck_pending:
                return
            self._report_stuck_pending = True

        req = ReportStuck.Request()
        req.route_version = int(self.core.route_version)
        req.current_index = int(self.core.index)
        req.current_wp_label = str(self.core.get_current_waypoint_label())
        pose = self.core.get_current_pose()
        if self._latest_pose_stamped is not None:
            req.current_pose = self._copy_pose_stamped(self._latest_pose_stamped)
        elif pose is not None:
            stamped_pose = self._core_pose_to_stamped(pose)
            req.current_pose = stamped_pose
        reason_label = str(self.core.last_stagnation_reason)
        normalized_reason = self._normalize_reason_label(reason_label)
        req.reason_code = int(self._convert_reason_code(normalized_reason))
        req.reason_detail = normalized_reason or reason_label
        req.avoid_trial_count = int(self.core.avoid_attempt_count)
        req.last_hint_blocked = bool(self.core.get_hint_front_blocked())
        req.last_applied_offset_m = float(self.core.last_applied_offset_m)

        future = self.cli_report_stuck.call_async(req)

        def _cb_done(fut):
            try:
                if fut.cancelled() or not fut.done():
                    raise RuntimeError("report_stuck timeout or cancelled")
                res: ReportStuck.Response = fut.result()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"{self.report_stuck_service_name} call failed: {exc}")
                res = None
            with self._report_lock:
                self._report_stuck_pending = False
                self._report_stuck_result = res
                self._report_stuck_result_ready = True

        future.add_done_callback(_cb_done)

    # ========================================================
    # ユーティリティ
    # ========================================================

    def _normalize_reason_label(self, label: str) -> str:
        """滞留理由ラベルを manager と共有可能な正規化表現へ変換する。"""
        normalized = (label or "").strip().lower()
        if normalized.startswith("road_blocked"):
            return "road_blocked"
        return normalized

    def _convert_reason_code(self, label: str) -> int:
        """滞留理由ラベルを ReportStuck の列挙値へ変換する。"""
        normalized = self._normalize_reason_label(label)
        mapping = {
            'front_blocked': ReportStuck.Request.REASON_FRONT_BLOCKED,
            'road_blocked': ReportStuck.Request.REASON_ROAD_BLOCKED,
            'no_hint': ReportStuck.Request.REASON_NO_HINT,
            'no_space': ReportStuck.Request.REASON_NO_SPACE,
            'avoidance_failed': ReportStuck.Request.REASON_AVOIDANCE_FAILED,
        }
        return mapping.get(normalized, ReportStuck.Request.REASON_UNKNOWN)

    def _compute_active_target_distance(self, output) -> float:
        """現在位置とアクティブターゲット間の距離を算出する。"""
        current_pose = self.core.get_current_pose()
        target_pose = output.target_pose or self.core.last_target or self._last_pub_target_pose
        if current_pose is None or target_pose is None:
            return 0.0
        return float(self._euclid_diff(current_pose, target_pose))

    def _yaw_from_quat(self, q: Quaternion) -> float:
        """クォータニオンからyaw(平面角)のみを抽出する。

        FollowerCoreは全ての姿勢を2D平面上のyawで扱い、目標生成や回避計算
        (_yaw_betweenやavoidanceサブゴール)で直接使用する。そのため受信時に
        一度だけyawへ正規化し、以降のロジックをクォータニオンに依存させない
        ようにする。
        """
        x, y, z, w = q.x, q.y, q.z, q.w
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _euclid_diff(self, a: CorePose, b: CorePose) -> float:
        """Pose間の位置差を返す（m単位）"""
        return math.hypot(a.x - b.x, a.y - b.y)

    def _pose_to_msg(self, pose: CorePose) -> Pose:
        p = Pose()
        p.position.x = pose.x
        p.position.y = pose.y
        p.position.z = 0.0
        q = Quaternion()
        orientation = (
            getattr(pose, "orientation_x", None),
            getattr(pose, "orientation_y", None),
            getattr(pose, "orientation_z", None),
            getattr(pose, "orientation_w", None),
        )
        if all(v is not None for v in orientation):
            q.x, q.y, q.z, q.w = (float(v) for v in orientation)
        else:
            q.x, q.y = 0.0, 0.0
            q.z = math.sin(pose.yaw / 2.0)
            q.w = math.cos(pose.yaw / 2.0)
        p.orientation = q
        return p

    @staticmethod
    def _copy_pose_stamped(src: PoseStamped) -> PoseStamped:
        """PoseStampedを新しいインスタンスへ複製する。"""

        copied = PoseStamped()
        copied.header = Header()
        if hasattr(src, "header") and hasattr(src.header, "stamp"):
            copied.header.stamp.sec = int(getattr(src.header.stamp, "sec", 0))
            copied.header.stamp.nanosec = int(getattr(src.header.stamp, "nanosec", 0))
        copied.header.frame_id = str(getattr(src.header, "frame_id", ""))
        fallback_pose = Pose()
        pose_field = getattr(src, "pose", fallback_pose)
        position = getattr(pose_field, "position", fallback_pose.position)
        orientation = getattr(pose_field, "orientation", fallback_pose.orientation)
        copied.pose.position.x = float(getattr(position, "x", 0.0))
        copied.pose.position.y = float(getattr(position, "y", 0.0))
        copied.pose.position.z = float(getattr(position, "z", 0.0))
        copied.pose.orientation.x = float(getattr(orientation, "x", 0.0))
        copied.pose.orientation.y = float(getattr(orientation, "y", 0.0))
        copied.pose.orientation.z = float(getattr(orientation, "z", 0.0))
        copied.pose.orientation.w = float(getattr(orientation, "w", 1.0))
        return copied

    def _core_pose_to_stamped(self, pose: CorePose) -> PoseStamped:
        """CorePoseからPoseStampedを生成する。"""

        stamped = PoseStamped()
        stamped.header = Header()
        stamped.header.frame_id = self.target_frame
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.pose = self._pose_to_msg(pose)
        return stamped


# ============================================================
# main
# ============================================================

def main(args=None) -> None:
    rclpy.init(args=args)
    node = RouteFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
