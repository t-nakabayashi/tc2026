import os
import subprocess
import sys
from types import SimpleNamespace
from gnss_lio_fusion.gnss_dropout import GnssDropoutHold


def test_hold_release_timeout_and_short_joy():
    gate = GnssDropoutHold()
    assert not gate.active(0.)
    gate.update([0]*5+[1], 1.)
    assert gate.active(1.49)
    assert not gate.active(1.5)
    gate.update([0]*5+[1], 2.)
    gate.update([0]*6, 2.1)
    assert not gate.active(2.1)
    gate.update([0]*5+[1], 3.)
    gate.update([], 3.1)
    assert not gate.active(3.1)


def test_fusion_drops_pending_and_live_gnss_then_accepts_fresh(monkeypatch):
    from gnss_lio_fusion.fusion_node import FusionNode
    import gnss_lio_fusion.fusion_node as module
    t = [1.]
    monkeypatch.setattr(module.time, 'monotonic', lambda: t[0])
    node = SimpleNamespace(gnss_dropout=GnssDropoutHold(), gnss_dropout_active=False,
        events=[(0., 'gps', None), (0., 'lio', None)], statuses={0.: None},
        get_logger=lambda: SimpleNamespace(warning=lambda _: None))
    node.alignment_available = lambda: True
    node.refresh_dropout = lambda: FusionNode.refresh_dropout(node)
    msg = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0)))
    node.gnss_dropout.update([0]*5+[1], 1.)
    FusionNode.on_fix(node, msg); FusionNode.on_status(node, msg)
    assert node.events == [(0., 'lio', None)] and not node.statuses
    # No further Joy messages: timeout restores input even if ROS time is paused.
    t[0] = 1.6
    FusionNode.on_status(node, msg); FusionNode.on_fix(node, msg)
    assert node.statuses == {1.: msg}
    assert node.events[-1] == (1., 'gps', msg)
    assert not node.gnss_dropout_active


def test_joy_ros_transport_releases_even_with_paused_sim_clock():
    """隔離ROS domainで、実際のJoy配送経路と停止時計での解除を検証する。

    ROS contextの初期化はDDSの内部スレッドを生成し、`rclpy.shutdown()` 後も
    それらは残留する。pytestの親プロセスでDDSを初期化すると、以降に
    `pytest-forked` が `os.fork()` する子プロセスがDDSの内部ロックを保持した
    まま複製され、子側の `rclpy.init()` がデッドロックする。そのため本テストは
    親プロセスではなく別プロセスでROSを起動する（`conftest.py` の
    `pytest_runtest_setup()` がこの規則を検査する）。
    """
    script = r'''
import time
import rclpy
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from gnss_lio_fusion.fusion_node import FusionNode
rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'], domain_id=196)
fusion = FusionNode()
source = rclpy.create_node('dropout_test_source')
publisher = source.create_publisher(Joy, '/joy', 10)
states = []
source.create_subscription(Bool, '/fusion/gnss_dropout_active', lambda m: states.append(m.data), 10)
executor = SingleThreadedExecutor()
executor.add_node(fusion); executor.add_node(source)
def spin_for(duration, buttons=None):
    end = time.monotonic()+duration
    while time.monotonic() < end:
        if buttons is not None: publisher.publish(Joy(buttons=buttons))
        executor.spin_once(timeout_sec=.01)
try:
    spin_for(.8, [0]*6)
    spin_for(.3, [0]*5+[1])
    assert fusion.gnss_dropout_active and states[-1]
    spin_for(.2, [0]*6)
    assert not fusion.gnss_dropout_active and not states[-1]
    spin_for(.3, [0]*5+[1])
    assert fusion.gnss_dropout_active
    spin_for(.8)
    assert not fusion.gnss_dropout_active and not states[-1]
    assert fusion.get_clock().now().nanoseconds == 0
finally:
    executor.shutdown(); source.destroy_node(); fusion.destroy_node(); rclpy.shutdown()
'''
    env = dict(os.environ, ROS_DOMAIN_ID='196', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    subprocess.run([sys.executable, '-c', script], env=env, check=True, timeout=30)
