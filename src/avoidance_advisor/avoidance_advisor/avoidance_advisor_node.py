# -*- coding: utf-8 -*-
"""停止中の回避について、AI エージェントへ助言を求める action server.

推論は機上 PC で `claude -p` 相当の CLI を起動して行う。応答は数秒から十数秒かかるため、
goal の実行中も購読を止めないよう、callback を並行実行できる構成で動かす。

助言は提案値であり、採用の可否は要求側（route_follower）が決める。本ノードは検証と
クランプを行い、採用できない場合は理由を付けて `advice_valid=false` を返す。
"""

from pathlib import Path
from typing import List, Optional
import os
import tempfile
import time

import cv2
import numpy as np

import rclpy
from cv_bridge import CvBridge
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from tc_diagnostics import DiagnosticReporter, report_alive
from tc_perception_msgs.msg import ObstacleLayers
from tc_route_msgs.action import RequestAvoidanceAdvice
from tc_route_msgs.msg import AvoidanceAdvice

from avoidance_advisor.advice_core import (ACTION_UNKNOWN, AdvicePolicy, AdviceRequest,
                                           build_prompt, parse_advice)
from avoidance_advisor.cli_core import (PROMPT_PLACEHOLDER, STATUS_CANCELED,
                                        STATUS_DONE, STATUS_TIMEOUT, CliConfig,
                                        CliRunner)

LAYER_IMAGE_NAME = 'layers.png'
CAMERA_IMAGE_NAME = 'camera.png'

# 不採用の理由のうち、CLI の実行そのものに起因するもの。
REASON_BUSY = 'busy'
REASON_DISABLED = 'disabled'
REASON_TIMEOUT = 'timeout'
REASON_CLI_FAILED = 'cli_failed'
REASON_CANCELED = 'canceled'


class AvoidanceAdvisorNode(Node):
    """回避の助言を返す action server."""

    def __init__(self) -> None:
        super().__init__('avoidance_advisor')

        self.declare_parameter('enabled', True)
        # CLI のコマンド列。`{prompt}` を要求文へ置換する。実行環境に合わせて変更する。
        self.declare_parameter('cli_command',
                               ['claude', '-p', '--output-format', 'text',
                                '--allowedTools', 'Read', PROMPT_PLACEHOLDER])
        self.declare_parameter('cli_timeout_sec', 20.0)
        # 作業ディレクトリの親。空ならシステムの一時ディレクトリ配下を使う。
        # リポジトリ内で CLI を起動すると、リポジトリの設定や指示を読み込むため避ける。
        self.declare_parameter('workspace_root', '')
        self.declare_parameter('keep_workspace', True)

        self.declare_parameter('min_confidence', 0.5)
        self.declare_parameter('default_wait_sec', 10.0)
        self.declare_parameter('max_wait_sec', 30.0)
        # 入力画像をこの時間より古い場合は要求文へ含めない。
        self.declare_parameter('image_max_age_sec', 5.0)

        self._bridge = CvBridge()
        self._layer_image: Optional[Image] = None
        self._camera_image: Optional[Image] = None
        self._layers: Optional[ObstacleLayers] = None

        qos_stream = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                durability=QoSDurabilityPolicy.VOLATILE, depth=1)
        qos_event = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                               durability=QoSDurabilityPolicy.VOLATILE, depth=10)
        callback_group = ReentrantCallbackGroup()

        self.sub_layer_view = self.create_subscription(
            Image, 'obstacle/layer_view', self._on_layer_image, qos_stream,
            callback_group=callback_group)
        self.sub_camera = self.create_subscription(
            Image, 'camera/image_raw', self._on_camera_image, qos_stream,
            callback_group=callback_group)
        self.sub_layers = self.create_subscription(
            ObstacleLayers, 'obstacle/layers', self._on_layers, qos_stream,
            callback_group=callback_group)

        self.pub_advice = self.create_publisher(AvoidanceAdvice, 'avoidance_advice',
                                                qos_event)

        self._runner = CliRunner(self._cli_config())
        self._server = ActionServer(
            self, RequestAvoidanceAdvice, 'request_avoidance_advice',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=callback_group)

        self._diagnostics = DiagnosticReporter(self)
        report_alive(self._diagnostics)
        self.get_logger().info(
            'avoidance_advisor を開始した。作業ディレクトリ=%s' % self._workspace_root())

    # ========================= 設定 =========================
    def _workspace_root(self) -> Path:
        configured = str(self.get_parameter('workspace_root').value or '')
        if configured:
            return Path(os.path.expanduser(configured))
        return Path(tempfile.gettempdir()) / 'avoidance_advisor'

    def _cli_config(self) -> CliConfig:
        return CliConfig(
            command=[str(part) for part in self.get_parameter('cli_command').value],
            timeout_sec=float(self.get_parameter('cli_timeout_sec').value),
            workspace_root=self._workspace_root(),
            keep_workspace=bool(self.get_parameter('keep_workspace').value),
        )

    def _policy(self) -> AdvicePolicy:
        return AdvicePolicy(
            min_confidence=float(self.get_parameter('min_confidence').value),
            default_wait_sec=float(self.get_parameter('default_wait_sec').value),
            max_wait_sec=float(self.get_parameter('max_wait_sec').value),
        )

    # ========================= 入力 =========================
    def _on_layer_image(self, msg: Image) -> None:
        self._layer_image = msg

    def _on_camera_image(self, msg: Image) -> None:
        self._camera_image = msg

    def _on_layers(self, msg: ObstacleLayers) -> None:
        self._layers = msg

    # ========================= action =========================
    def _on_goal(self, goal_request) -> GoalResponse:
        if not bool(self.get_parameter('enabled').value):
            self.get_logger().warn('助言は無効化されている。要求を拒否する。')
            return GoalResponse.REJECT
        if self._runner.busy:
            self.get_logger().warn('前の問い合わせが実行中のため要求を拒否する。')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle) -> CancelResponse:
        self.get_logger().info('助言の要求が取り消された。CLI を終了する。')
        self._runner.cancel()
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        goal = goal_handle.request
        request = self._build_request(goal)
        started = time.monotonic()

        files = {}
        layer_name = self._encode_image(self._layer_image, files, LAYER_IMAGE_NAME)
        camera_name = self._encode_image(self._camera_image, files, CAMERA_IMAGE_NAME)
        prompt = build_prompt(request, layer_name, camera_name)

        self._publish_feedback(goal_handle, 'invoking', started)
        invocation = self._runner.start(prompt, files=files)
        if invocation is None:
            return self._finish(goal_handle, None, REASON_BUSY, '')

        while not invocation.wait(timeout=0.2):
            if goal_handle.is_cancel_requested:
                invocation.cancel()
                invocation.wait(timeout=5.0)
                return self._finish(goal_handle, None, REASON_CANCELED,
                                    invocation.request_id, canceled=True)
            self._publish_feedback(goal_handle, 'invoking', started)

        result = invocation.result
        self.get_logger().info(
            '助言の問い合わせが終了した。id=%s status=%s elapsed=%.1fs'
            % (invocation.request_id, result.status, result.elapsed_s))
        self._save_output(invocation.workspace, result.stdout, result.stderr)

        if result.status == STATUS_TIMEOUT:
            return self._finish(goal_handle, None, REASON_TIMEOUT, invocation.request_id)
        if result.status == STATUS_CANCELED:
            return self._finish(goal_handle, None, REASON_CANCELED,
                                invocation.request_id, canceled=True)
        if result.status != STATUS_DONE:
            self.get_logger().warn('CLI が異常終了した: %s' % result.stderr.strip()[:200])
            return self._finish(goal_handle, None, REASON_CLI_FAILED,
                                invocation.request_id)

        self._publish_feedback(goal_handle, 'parsing', started)
        advice, reason = parse_advice(result.stdout, request, self._policy())
        if advice is None:
            self.get_logger().warn('助言を採用しない: %s' % reason)
        return self._finish(goal_handle, advice, reason, invocation.request_id)

    # ========================= 補助 =========================
    def _build_request(self, goal) -> AdviceRequest:
        return AdviceRequest(
            waypoint_label=str(goal.waypoint_label),
            front_clearance_m=float(goal.front_clearance_m),
            hint_left_offset_m=float(goal.hint_left_offset_m),
            hint_right_offset_m=float(goal.hint_right_offset_m),
            min_offset_m=float(goal.min_offset_m),
            max_left_offset_m=min(float(goal.waypoint_left_open_m),
                                  float(goal.max_offset_m)),
            max_right_offset_m=min(float(goal.waypoint_right_open_m),
                                   float(goal.max_offset_m)),
            context_lines=self._context_lines(),
        )

    def _context_lines(self) -> List[str]:
        """購読した層の観測値を、要求文へ入れる行へ整える.

        書式は lidar_layer_slicer の構造化テキストに合わせる。層が未受信の場合は
        その旨を 1 行で伝え、AI が観測値なしで判断していることを分かるようにする。
        """
        layers = self._layers
        if layers is None or not layers.layers:
            return ['layers=unavailable']
        lines = [f'robot_width_m={layers.robot_width_m:.2f} '
                 f'side_margin_m={layers.side_margin_m:.2f} '
                 f'forward_range_m={layers.forward_range_m:.2f} '
                 f'accumulated_frames={layers.accumulated_frames}']
        for layer in layers.layers:
            lines.append(
                f'layer={layer.name} height_m={layer.height_m:.2f} '
                f'source={layer.source} points={layer.point_count} '
                f'nearest_x_m={_fmt(layer.nearest_x_m)} '
                f'required_left_m={_fmt(layer.required_left_offset_m)} '
                f'available_left_m={_fmt(layer.available_left_offset_m)} '
                f'required_right_m={_fmt(layer.required_right_offset_m)} '
                f'available_right_m={_fmt(layer.available_right_offset_m)}')
        return lines

    def _encode_image(self, message: Optional[Image], files: dict,
                      name: str) -> Optional[str]:
        """画像を PNG へ符号化して作業ディレクトリへ置くファイルに加える.

        受信していない、または古い画像は要求文へ含めない。古い画像を渡すと、
        現在の状況と異なる場面で判断させることになる。
        """
        if message is None:
            return None
        age = self._message_age_sec(message)
        if age > float(self.get_parameter('image_max_age_sec').value):
            self.get_logger().warn(f'{name} が古いため要求文へ含めない: {age:.1f}s')
            return None
        try:
            image = self._bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        except Exception as error:
            self.get_logger().warn(f'{name} を変換できない: {error}')
            return None
        success, buffer = cv2.imencode('.png', np.asarray(image))
        if not success:
            return None
        files[name] = buffer.tobytes()
        return name

    def _message_age_sec(self, message) -> float:
        stamp = message.header.stamp
        sent = stamp.sec + stamp.nanosec * 1e-9
        if sent <= 0.0:
            return 0.0
        return max(self.get_clock().now().nanoseconds * 1e-9 - sent, 0.0)

    def _save_output(self, workspace: Path, stdout: str, stderr: str) -> None:
        """CLI の出力を作業ディレクトリへ残す（事後の確認用）."""
        try:
            (workspace / 'stdout.txt').write_text(stdout or '', encoding='utf-8')
            if stderr:
                (workspace / 'stderr.txt').write_text(stderr, encoding='utf-8')
        except OSError as error:
            self.get_logger().warn(f'CLI の出力を保存できない: {error}')

    def _publish_feedback(self, goal_handle, phase: str, started: float) -> None:
        feedback = RequestAvoidanceAdvice.Feedback()
        feedback.phase = phase
        feedback.elapsed_s = float(time.monotonic() - started)
        goal_handle.publish_feedback(feedback)

    def _finish(self, goal_handle, advice, reason: str, request_id: str,
                canceled: bool = False):
        """action の result を組み立て、助言を topic へも配信する."""
        result = RequestAvoidanceAdvice.Result()
        message = AvoidanceAdvice()
        message.header.stamp = self.get_clock().now().to_msg()
        message.request_id = request_id
        if advice is not None:
            message.action = int(advice.action)
            message.side = int(advice.side)
            message.offset_m = float(advice.offset_m)
            message.wait_sec = float(advice.wait_sec)
            message.is_moving = bool(advice.is_moving)
            message.confidence = float(advice.confidence)
            message.obstacle_class = advice.obstacle_class
            message.rationale = advice.rationale
        else:
            message.action = ACTION_UNKNOWN
        result.advice = message
        result.advice_valid = advice is not None
        result.failure_reason = '' if advice is not None else reason
        self.pub_advice.publish(message)
        if canceled:
            goal_handle.canceled()
        else:
            goal_handle.succeed()
        return result


def _fmt(value: float) -> str:
    """inf を文字列として扱えるよう整える."""
    return 'inf' if not np.isfinite(value) else f'{value:.2f}'


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AvoidanceAdvisorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
