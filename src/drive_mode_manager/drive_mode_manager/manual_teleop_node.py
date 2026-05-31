"""/joy から /cmd_vel/manual を生成する ROS 2 ノード。"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy

from drive_mode_manager.manual_teleop_core import ManualTeleopConfig, ManualTeleopCore


class ManualTeleopNode(Node):
    """L1 deadman 中だけ手動 Twist を publish する。"""

    def __init__(self) -> None:
        super().__init__('manual_teleop_node')
        config = self._load_config()
        self._core = ManualTeleopCore(config)
        self._last_reason = ''

        self._publisher = self.create_publisher(Twist, 'cmd_vel/manual', 1)
        self.create_subscription(Joy, 'joy', self._on_joy, 10)
        period_s = 1.0 / max(float(self.get_parameter('publish_rate_hz').value), 1.0)
        self.create_timer(period_s, self._on_timer)
        self.get_logger().info('manual_teleop_node started: /joy -> /cmd_vel/manual')

    def _load_config(self) -> ManualTeleopConfig:
        self.declare_parameter('linear_axis', 1)
        self.declare_parameter('angular_axis', 0)
        self.declare_parameter('linear_y_axis', -1)
        self.declare_parameter('linear_scale', 1.2)
        self.declare_parameter('angular_scale', 1.5)
        self.declare_parameter('linear_y_scale', 0.5)
        self.declare_parameter('deadzone', 0.05)
        self.declare_parameter('linear_axis_invert', False)
        self.declare_parameter('angular_axis_invert', False)
        self.declare_parameter('enable_button', 4)
        self.declare_parameter('turbo_button', 6)
        self.declare_parameter('turbo_ratio', 1.5)
        self.declare_parameter('joy_timeout_s', 0.5)
        self.declare_parameter('publish_rate_hz', 20.0)
        return ManualTeleopConfig(
            linear_axis=int(self.get_parameter('linear_axis').value),
            angular_axis=int(self.get_parameter('angular_axis').value),
            linear_y_axis=int(self.get_parameter('linear_y_axis').value),
            linear_scale=float(self.get_parameter('linear_scale').value),
            angular_scale=float(self.get_parameter('angular_scale').value),
            linear_y_scale=float(self.get_parameter('linear_y_scale').value),
            deadzone=float(self.get_parameter('deadzone').value),
            linear_axis_invert=bool(self.get_parameter('linear_axis_invert').value),
            angular_axis_invert=bool(self.get_parameter('angular_axis_invert').value),
            enable_button=int(self.get_parameter('enable_button').value),
            turbo_button=int(self.get_parameter('turbo_button').value),
            turbo_ratio=float(self.get_parameter('turbo_ratio').value),
            joy_timeout_s=float(self.get_parameter('joy_timeout_s').value),
        )

    def _on_joy(self, msg: Joy) -> None:
        self._core.update_joy(msg.axes, msg.buttons, self._now_s())

    def _on_timer(self) -> None:
        result = self._core.compute(self._now_s())
        msg = Twist()
        msg.linear.x = result.linear_x
        msg.linear.y = result.linear_y
        msg.angular.z = result.angular_z
        self._publisher.publish(msg)
        if result.reason != self._last_reason:
            self._last_reason = result.reason
            self.get_logger().debug(f'manual teleop reason={result.reason}')

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main() -> None:
    rclpy.init()
    node = ManualTeleopNode()
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
