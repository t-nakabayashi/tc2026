"""自律 cmd と手動 cmd を排他的に選択する ROS 2 ノード。"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy

from drive_mode_manager.drive_mode_core import (
    CommandSnapshot,
    DriveModeConfig,
    DriveModeCore,
    DriveModeOutput,
    JoySnapshot,
)
from tc_route_msgs.msg import DriveModeStatus


class DriveCmdMuxNode(Node):
    """最終 /cmd_vel の唯一の publish 元になる mux ノード。"""

    def __init__(self) -> None:
        super().__init__('drive_cmd_mux_node')
        config = self._load_config()
        self._core = DriveModeCore(config)
        self._latest_joy: JoySnapshot | None = None
        self._latest_auto: CommandSnapshot | None = None
        self._latest_manual: CommandSnapshot | None = None
        self._last_reason = ''
        self._last_mode = self._core.mode

        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self._status_pub = self.create_publisher(DriveModeStatus, 'drive_mode_status', 10)
        self.create_subscription(Twist, 'cmd_vel/autonomous', self._on_autonomous_cmd, 1)
        self.create_subscription(Twist, 'cmd_vel/manual', self._on_manual_cmd, 1)
        self.create_subscription(Joy, 'joy', self._on_joy, 10)
        period_s = 1.0 / max(float(self.get_parameter('publish_rate_hz').value), 1.0)
        self.create_timer(period_s, self._on_timer)
        self.get_logger().info('drive_cmd_mux_node started: /cmd_vel is managed by this node')

    def _load_config(self) -> DriveModeConfig:
        self.declare_parameter('initial_mode', 'autonomous')
        self.declare_parameter('manual_transition_trigger', 'l1_ps_button_hold')
        self.declare_parameter('manual_transition_hold_s', 2.0)
        self.declare_parameter('manual_to_auto_l1_released_s', 1.0)
        self.declare_parameter('auto_resume_delay_s', 5.0)
        self.declare_parameter('autonomous_cmd_timeout_s', 0.5)
        self.declare_parameter('manual_cmd_timeout_s', 0.3)
        self.declare_parameter('joy_timeout_s', 0.5)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('l1_button_index', 4)
        self.declare_parameter('ps_button_index', 5)
        trigger = str(self.get_parameter('manual_transition_trigger').value)
        if trigger != 'l1_ps_button_hold':
            self.get_logger().warn(
                'manual_transition_trigger は l1_ps_button_hold のみ実装済みです。'
                'l1_ps_button_hold として扱います。'
            )
        return DriveModeConfig(
            initial_mode=str(self.get_parameter('initial_mode').value),
            manual_transition_hold_s=float(self.get_parameter('manual_transition_hold_s').value),
            manual_to_auto_l1_released_s=float(
                self.get_parameter('manual_to_auto_l1_released_s').value
            ),
            auto_resume_delay_s=float(self.get_parameter('auto_resume_delay_s').value),
            autonomous_cmd_timeout_s=float(self.get_parameter('autonomous_cmd_timeout_s').value),
            manual_cmd_timeout_s=float(self.get_parameter('manual_cmd_timeout_s').value),
            joy_timeout_s=float(self.get_parameter('joy_timeout_s').value),
            l1_button_index=int(self.get_parameter('l1_button_index').value),
            ps_button_index=int(self.get_parameter('ps_button_index').value),
        )

    def _on_joy(self, msg: Joy) -> None:
        self._latest_joy = JoySnapshot(tuple(msg.buttons), self._now_s())

    def _on_autonomous_cmd(self, msg: Twist) -> None:
        self._latest_auto = self._to_snapshot(msg)

    def _on_manual_cmd(self, msg: Twist) -> None:
        self._latest_manual = self._to_snapshot(msg)

    def _to_snapshot(self, msg: Twist) -> CommandSnapshot:
        return CommandSnapshot(
            linear_x=float(msg.linear.x),
            linear_y=float(msg.linear.y),
            angular_z=float(msg.angular.z),
            stamp_s=self._now_s(),
        )

    def _on_timer(self) -> None:
        output = self._core.update(
            now_s=self._now_s(),
            joy=self._latest_joy,
            autonomous_cmd=self._latest_auto,
            manual_cmd=self._latest_manual,
        )
        self._publish_cmd(output)
        self._publish_status(output)
        if output.mode != self._last_mode:
            self._last_mode = output.mode
            self.get_logger().info(f'drive mode changed: mode={output.mode} reason={output.reason}')
        if output.reason != self._last_reason:
            self._last_reason = output.reason
            self.get_logger().debug(f'drive mux reason={output.reason}')

    def _publish_cmd(self, output: DriveModeOutput) -> None:
        msg = Twist()
        msg.linear.x = output.linear_x
        msg.linear.y = output.linear_y
        msg.angular.z = output.angular_z
        self._cmd_pub.publish(msg)

    def _publish_status(self, output: DriveModeOutput) -> None:
        msg = DriveModeStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.mode = output.mode
        msg.output_source = output.output_source
        msg.joy_available = output.joy_available
        msg.l1_pressed = output.l1_pressed
        msg.ps_button_pressed = output.ps_button_pressed
        msg.ps_hold_progress_s = float(output.ps_hold_progress_s)
        msg.manual_input_active = output.manual_input_active
        msg.manual_cmd_alive = output.manual_cmd_alive
        msg.autonomous_cmd_alive = output.autonomous_cmd_alive
        msg.auto_resume_pending = output.auto_resume_pending
        msg.auto_resume_remaining_s = float(output.auto_resume_remaining_s)
        msg.pending_autonomous_linear_x = float(output.pending_autonomous_linear_x)
        msg.pending_autonomous_angular_z = float(output.pending_autonomous_angular_z)
        msg.output_linear_x = float(output.linear_x)
        msg.output_angular_z = float(output.angular_z)
        msg.reason = output.reason
        self._status_pub.publish(msg)

    def destroy_node(self) -> bool:
        if rclpy.ok():
            try:
                self._cmd_pub.publish(Twist())
            except Exception:
                pass
        return super().destroy_node()

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main() -> None:
    rclpy.init()
    node = DriveCmdMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


if __name__ == '__main__':
    main()
