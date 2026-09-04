#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""GNSS LLH/ENU poseをpublishするROS 2ノード."""

from __future__ import annotations

import copy
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Header

from rtk_gps_um982_msgs.msg import RtkStatus
from tc_geo_msgs.msg import GeoPoseWithQuality, MapProjection

from geo_pose_converter.geo_core import ProjectionConfig
from geo_pose_converter.message_utils import (
    llh_to_pose_with_covariance,
    make_geo_pose,
    make_geo_pose_quality,
    navsatfix_to_llh,
    projection_to_msg,
)


class GeoPoseConverterNode(Node):
    """GNSS raw topicをLLH/ENU poseへ正規化するノード."""

    def __init__(self) -> None:
        super().__init__('geo_pose_converter')

        self.declare_parameter('projection_id', 'default')
        self.declare_parameter('datum', 'WGS84')
        self.declare_parameter('map_frame_id', 'map')
        self.declare_parameter('earth_frame_id', 'earth')
        self.declare_parameter('child_frame_id', 'gps_link')
        self.declare_parameter('origin_latitude', 0.0)
        self.declare_parameter('origin_longitude', 0.0)
        self.declare_parameter('origin_altitude', 0.0)
        self.declare_parameter('map_yaw_offset_rad', 0.0)

        self.projection = ProjectionConfig(
            origin_latitude=float(self.get_parameter('origin_latitude').value),
            origin_longitude=float(self.get_parameter('origin_longitude').value),
            origin_altitude=float(self.get_parameter('origin_altitude').value),
            map_yaw_offset_rad=float(self.get_parameter('map_yaw_offset_rad').value),
            projection_id=str(self.get_parameter('projection_id').value),
            datum=str(self.get_parameter('datum').value),
            map_frame_id=str(self.get_parameter('map_frame_id').value),
            earth_frame_id=str(self.get_parameter('earth_frame_id').value),
        )
        self.child_frame_id = str(self.get_parameter('child_frame_id').value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pub_gnss_llh = self.create_publisher(GeoPoseWithQuality, 'gnss/pose_llh', qos)
        self.pub_gnss_enu = self.create_publisher(
            PoseWithCovarianceStamped,
            'gnss/pose_enu',
            qos,
        )
        self.pub_projection = self.create_publisher(
            MapProjection,
            'geo/map_projection',
            qos,
        )

        self.latest_heading: Optional[Imu] = None
        self.latest_status: Optional[RtkStatus] = None
        self.create_subscription(NavSatFix, 'rtk_gps/fix', self._on_fix, qos)
        self.create_subscription(Imu, 'rtk_gps/heading', self._on_heading, qos)
        self.create_subscription(RtkStatus, 'rtk_gps/rtk_status', self._on_status, qos)

        self.projection_timer = self.create_timer(1.0, self._publish_projection)
        self.get_logger().info('geo_pose_converter node started.')

    def _publish_projection(self) -> None:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.projection.earth_frame_id
        self.pub_projection.publish(projection_to_msg(self.projection, header))

    def _on_heading(self, msg: Imu) -> None:
        self.latest_heading = copy.deepcopy(msg)

    def _on_status(self, msg: RtkStatus) -> None:
        self.latest_status = copy.deepcopy(msg)

    def _on_fix(self, msg: NavSatFix) -> None:
        point = navsatfix_to_llh(msg)
        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = self.projection.earth_frame_id

        # heading_deg=0.0 は真北を向く有効値なので、値そのものではなく
        # RtkStatus の受信有無で heading の有効性を判断する。
        has_heading = self.latest_status is not None
        heading = float(self.latest_status.heading_deg) if self.latest_status is not None else 0.0
        geo_pose = make_geo_pose(
            header,
            point,
            heading,
            has_heading,
            self.child_frame_id,
            has_altitude=True,
        )
        quality = make_geo_pose_quality(
            header,
            geo_pose,
            self.latest_status,
            GeoPoseWithQuality.SOURCE_GNSS,
            GeoPoseWithQuality.FUSION_OK,
        )
        enu_header = Header()
        enu_header.stamp = msg.header.stamp
        enu_header.frame_id = self.projection.map_frame_id
        enu_pose = llh_to_pose_with_covariance(
            enu_header,
            point,
            heading,
            has_heading,
            self.projection,
        )
        if msg.position_covariance:
            enu_pose.pose.covariance[0] = msg.position_covariance[0]
            enu_pose.pose.covariance[7] = msg.position_covariance[4]
            enu_pose.pose.covariance[14] = msg.position_covariance[8]

        self.pub_gnss_llh.publish(quality)
        self.pub_gnss_enu.publish(enu_pose)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GeoPoseConverterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
