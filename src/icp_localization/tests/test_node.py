"""Execute only in an isolated DDS/network namespace; never launch robot drivers."""
import json
import time

import numpy as np
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header, String
from rtk_gps_um982_msgs.msg import RtkStatus
from icp_localization.node import Localizer
from test_registration import fixture_map, PROJECTION


def test_pose_interface_without_driving_side_effects(tmp_path):
    import socket
    assert set(name for _, name in socket.if_nameindex()) == {'lo'}, 'Run with network isolation'
    points, _, manifest = fixture_map(tmp_path)
    context = Context()
    rclpy.init(context=context)
    node = peer = executor = None
    try:
        values = dict(PROJECTION, map_manifest=str(manifest), base_imu_xyz=[0., 0., 0.],
                      base_imu_rpy_deg=[0., 0., 0.], antenna_xyz=[0., 0., 0.], publish_base_tf=False)
        node = Localizer(context=context, parameter_overrides=[Parameter(k, value=v) for k, v in values.items()])
        peer = Node('icp_test_source', context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node); executor.add_node(peer)
        pubs = {k: peer.create_publisher(cls, topic, 10) for k, cls, topic in (
            ('odom', Odometry, '/lio/odometry_raw'), ('cloud', PointCloud2, '/cloud_registered_body'),
            ('fix', NavSatFix, '/rtk_gps/fix'), ('status', RtkStatus, '/rtk_gps/rtk_status'),
            ('initial', PoseWithCovarianceStamped, '/initialpose'))}
        poses, statuses = [], []
        actual_x = [0.]
        peer.create_subscription(PoseWithCovarianceStamped, '/localization/pose_enu', poses.append, 10)
        peer.create_subscription(String, '/localization/icp_status', lambda m: statuses.append(json.loads(m.data)), 10)

        def spin(duration, *, x=0., gnss=None, cloud=True, bad=False, odom=True):
            end, next_send = time.monotonic()+duration, 0.
            while time.monotonic() < end:
                if time.monotonic() >= next_send:
                    next_send = time.monotonic()+.05
                    h = Header(); h.stamp = peer.get_clock().now().to_msg(); h.frame_id = 'body'
                    actual_x[0] += float(np.clip(x-actual_x[0], -.02, .02))
                    measured_x = actual_x[0]
                    if odom:
                        m = Odometry(); m.header.stamp = h.stamp; m.header.frame_id = 'camera_init'
                        m.child_frame_id = 'body'; m.pose.pose.orientation.w = 1.; m.pose.pose.position.x = measured_x
                        pubs['odom'].publish(m)
                    if cloud:
                        p = np.ones((300, 3))*30 if bad else points-[measured_x, 0., 1.]
                        pubs['cloud'].publish(create_cloud_xyz32(h, p.astype(np.float32)))
                    if gnss is not None:
                        f = NavSatFix(); f.header.stamp = h.stamp; f.latitude = 36.+gnss; f.longitude = 140.
                        f.status.status = 2; f.position_covariance_type = 2
                        f.position_covariance = [0.01, 0., 0., 0., .01, 0., 0., 0., .01]
                        s = RtkStatus(); s.header.stamp = h.stamp; s.rtk_state = 4
                        s.heading_deg = 270.; s.heading_stddev_deg = 1.
                        pubs['fix'].publish(f); pubs['status'].publish(s)
                executor.spin_once(timeout_sec=.005)

        spin(1.5, gnss=0.)
        assert node.core.localized and len(poses) > 3
        assert poses[-1].header.frame_id == 'map'
        assert abs(poses[-1].pose.pose.position.x) < .04
        assert abs(poses[-1].pose.pose.position.z-1.) < .04
        # After initial map alignment, a wrong FIX cannot move the estimate.
        spin(.6, x=.1, gnss=.001)
        assert abs(poses[-1].pose.pose.position.x-.1) < .05
        # No accepted map correction -> LIO prediction, no reset or movement command.
        n = len(poses)
        spin(.8, x=.1, bad=True, gnss=.001)
        assert len(poses) > n and node.core.localized
        assert statuses[-1]['state'] == 'LIO_PREDICTION'
        assert abs(poses[-1].pose.pose.position.x-.1) < .05
        last_good, diagnostic_start = node.core.last_accepted, len(statuses)
        spin(.6, x=.1)
        assert node.core.last_accepted > last_good
        assert any(s['state'] == 'ICP_TRACKING' for s in statuses[diagnostic_start:])
        advertised = dict(peer.get_publisher_names_and_types_by_node('icp_localization', '/'))
        assert '/localization/pose_enu' in advertised
        assert not any('Twist' in t or 'Bool' in t for types in advertised.values() for t in types)
        # Sensor discontinuity does not activate automatic GNSS reinitialization.
        spin(.6, x=.1, cloud=False, odom=False)
        n = len(poses)
        spin(.3, x=.1, cloud=False, odom=False)
        assert len(poses) == n
        spin(.5, x=.1, gnss=0.)
        assert not node.core.localized and node.core.initialized_once
        initial = PoseWithCovarianceStamped(); initial.header.frame_id = 'map'
        initial.pose.pose.position.x = .1; initial.pose.pose.position.z = 1.
        initial.pose.pose.orientation.w = 1.
        pubs['initial'].publish(initial)
        spin(.7, x=.1)
        assert node.core.localized
    finally:
        if executor:
            executor.shutdown()
        if node:
            node.destroy_node()
        if peer:
            peer.destroy_node()
        context.shutdown()
