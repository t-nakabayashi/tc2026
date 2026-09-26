#!/usr/bin/env python3
"""隔離ROS graphの人工観測でLIO途絶中の車輪継続と全入力途絶停止を検証する."""
import json
import os
import time

os.environ['ROS_DOMAIN_ID'] = '87'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped
from rtk_gps_um982_msgs.msg import RtkStatus
from tc_geo_msgs.msg import FusionState

from geo_pose_converter.geo_core import EnuPoint, ProjectionConfig, enu_to_llh
from gnss_lio_fusion.fusion_node import FusionNode


def main() -> None:
    """実機トピックに接続しない。truthは購読せず、既知の人工観測を配信する."""
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true', '-p', 'buffer_s:=0.1'])
    fusion = FusionNode()
    source = Node('fusion_dropout_test')
    clock = source.create_publisher(Clock, '/clock', 10)
    wheel = source.create_publisher(Odometry, '/ypspur_ros/odom', 10)
    lio = source.create_publisher(Odometry, '/lio/odometry', 10)
    fix = source.create_publisher(NavSatFix, '/rtk_gps/fix', 10)
    status = source.create_publisher(RtkStatus, '/rtk_gps/rtk_status', 10)
    command = source.create_publisher(Twist, '/cmd_vel/autonomous', 10)
    poses, commands, modes = [], [], []
    source.create_subscription(FusionState, '/fusion/status',
        lambda msg: modes.append(msg.mode), 10)
    current_time = [0.]
    source.create_subscription(PoseWithCovarianceStamped, '/localization/pose_enu',
        lambda msg: poses.append(msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9), 10)
    source.create_subscription(Twist, '/cmd_vel/fusion_limited',
        lambda msg: commands.append((current_time[0], msg.linear.x)), 10)
    executor = SingleThreadedExecutor()
    executor.add_node(fusion)
    executor.add_node(source)
    try:
        for _ in range(30):
            executor.spin_once(timeout_sec=.01)
        for i in range(1, 81):
            t = i*.1
            current_time[0] = t
            tick = Clock()
            tick.clock.sec = int(t)
            tick.clock.nanosec = int(round((t-int(t))*1e9))
            clock.publish(tick)
            if t <= 4.:
                odom = Odometry()
                odom.header.stamp = tick.clock
                odom.pose.pose.position.x = .1*t
                odom.pose.pose.orientation.w = 1.
                wheel.publish(odom)
                if t < 1.5 or t > 3.5:
                    odom.pose.pose.position.z = .6
                    lio.publish(odom)
                point = enu_to_llh(EnuPoint(.1*t, 0., .7), ProjectionConfig(0., 0., 0.))
                gps = NavSatFix()
                gps.header.stamp = tick.clock
                gps.latitude, gps.longitude, gps.altitude = point.latitude, point.longitude, point.altitude
                gps.position_covariance[0] = gps.position_covariance[4] = .0004
                quality = RtkStatus()
                quality.header.stamp = tick.clock
                quality.rtk_state, quality.num_satellites = 4, 20
                quality.baseline_length_m = .5
                quality.heading_deg, quality.heading_stddev_deg = 90., .5
                status.publish(quality)
                fix.publish(gps)
            cmd = Twist()
            cmd.linear.x = .2
            command.publish(cmd)
            for _ in range(24):
                executor.spin_once(timeout_sec=.001)
        continued = any(2.5 < t < 3.4 for t in poses)
        moving = any(2.5 < t < 3.4 and v > .1 for t, v in commands)
        after = [v for t, v in commands if t > 6.]
        stopped = bool(after) and all(abs(v) < 1e-9 for v in after)
        result = dict(pass_test=continued and moving and stopped and 'WAIT_INITIAL_FIX' in modes,
                      initial_wait_visible='WAIT_INITIAL_FIX' in modes,
                      pose_during_lio_dropout=continued, command_during_lio_dropout=moving,
                      stopped_after_both_inputs_lost=stopped, pose_count=len(poses),
                      wheel_fallback_count=fusion.motion.fallback_count)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        if not result['pass_test']:
            raise RuntimeError('車輪退避・途絶停止試験が不合格')
    finally:
        executor.shutdown()
        fusion.destroy_node()
        source.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
