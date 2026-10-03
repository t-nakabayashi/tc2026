import pytest


@pytest.mark.forked
def test_receiver_uses_namespace_public_topics_without_opening_hardware(monkeypatch):
    import rclpy
    from rtk_gps_um982 import driver_node
    class Receiver:
        def __init__(self, **kwargs): self._ntrip_client = None
        def set_position_callback(self, callback): pass
        def start(self): pass
        def stop(self): pass
    monkeypatch.setattr(driver_node, 'UM982Client', Receiver)
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
