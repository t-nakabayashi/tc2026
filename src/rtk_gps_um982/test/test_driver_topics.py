"""driver_node が名前空間付きの公開トピックへ配信することを確認する。

ROS context を初期化するため、pytest の親プロセスではなく subprocess で実行する。
`rclpy.init()` が生成する DDS の内部スレッドは `rclpy.shutdown()` 後も残留し、
親プロセスに残ると後続の forked テストが `os.fork()` 後にデッドロックする。
規則と検査はワークスペース直下の `conftest.py` を参照する。

実機を開かないよう UM982Client を差し替えて起動する。
"""

import os
import subprocess
import sys

# 子プロセスで実行する検証本体。実機デバイスは開かない。
SCRIPT = r'''
import rclpy
from rtk_gps_um982 import driver_node


class Receiver:
    """実機を開かない UM982Client の代替。"""

    def __init__(self, **kwargs):
        self._ntrip_client = None

    def set_position_callback(self, callback):
        pass

    def start(self):
        pass

    def stop(self):
        pass


driver_node.UM982Client = Receiver
rclpy.init(args=['--ros-args', '-r', '__ns:=/rtk_gps'])
node = driver_node.Um982DriverNode()
try:
    assert node._pub_fix.topic_name == '/rtk_gps/fix'
    assert node._pub_imu.topic_name == '/rtk_gps/heading'
    assert node._pub_status.topic_name == '/rtk_gps/rtk_status'
    assert node._pub_ntrip.topic_name == '/rtk_gps/ntrip_status'
finally:
    node.destroy_node()
    rclpy.shutdown()
'''


def test_receiver_uses_namespace_public_topics_without_opening_hardware() -> None:
    """名前空間 `/rtk_gps` 配下の公開名で配信することを確認する."""
    env = dict(os.environ, ROS_DOMAIN_ID='197', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    subprocess.run([sys.executable, '-c', SCRIPT], env=env, check=True, timeout=60)
