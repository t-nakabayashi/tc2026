#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""ENU自己位置とactive targetをLLH表示用topicへ投影するROS 2ノード."""

from __future__ import annotations

import copy
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Header
from tc_geo_msgs.msg import GeoPoseWithQuality
from tc_route_msgs.msg import ActiveTargetLlh, FollowerState, Route, Waypoint

from geo_pose_converter.geo_core import ProjectionConfig
from geo_pose_converter.message_utils import (
    make_active_target_llh,
    make_geo_pose_quality,
    pose_to_llh_pose,
    projection_from_msg,
)


class RouteGeoProjectorNode(Node):
    """ENU poseをLLH表示用pose/targetへ変換するノード."""

    def __init__(self) -> None:
        super().__init__('route_geo_projector')

        self.declare_parameter('projection_id', 'default')
        self.declare_parameter('datum', 'WGS84')
        self.declare_parameter('map_frame_id', 'map')
        self.declare_parameter('earth_frame_id', 'earth')
        self.declare_parameter('origin_latitude', 0.0)
        self.declare_parameter('origin_longitude', 0.0)
        self.declare_parameter('origin_altitude', 0.0)
        self.declare_parameter('map_yaw_offset_rad', 0.0)
        self.declare_parameter('pose_enu_topic', '/localization/pose_enu')
        self.declare_parameter('pose_llh_topic', '/localization/pose_llh')
        self.declare_parameter('pose_child_frame_id', 'base_link')

        self.configured_projection = ProjectionConfig(
            origin_latitude=float(self.get_parameter('origin_latitude').value),
            origin_longitude=float(self.get_parameter('origin_longitude').value),
            origin_altitude=float(self.get_parameter('origin_altitude').value),
            map_yaw_offset_rad=float(self.get_parameter('map_yaw_offset_rad').value),
            projection_id=str(self.get_parameter('projection_id').value),
            datum=str(self.get_parameter('datum').value),
            map_frame_id=str(self.get_parameter('map_frame_id').value),
            earth_frame_id=str(self.get_parameter('earth_frame_id').value),
        )
        self.projection = self.configured_projection
        self._projection_mismatch_logged = False
        self._pose_frame_mismatch_logged = False
        self._target_frame_mismatch_logged = False

        qos_stream = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_route = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pose_enu_topic = str(self.get_parameter('pose_enu_topic').value)
        self.pose_llh_topic = str(self.get_parameter('pose_llh_topic').value)
        self.pose_child_frame_id = str(self.get_parameter('pose_child_frame_id').value)

        self.pub_pose_llh = self.create_publisher(
            GeoPoseWithQuality,
            self.pose_llh_topic,
            qos_stream,
        )
        self.pub_target_llh = self.create_publisher(
            ActiveTargetLlh,
            'route/active_target_llh',
            qos_stream,
        )

        self.active_route: Optional[Route] = None
        self.active_target: Optional[PoseStamped] = None
        self.follower_state: Optional[FollowerState] = None
        self.pose_enu: Optional[PoseWithCovarianceStamped] = None
        self.current_geo_pose = None

        self.create_subscription(Route, 'active_route', self._on_route, qos_route)
        self.create_subscription(PoseStamped, 'active_target', self._on_target, qos_stream)
        self.create_subscription(FollowerState, 'follower_state', self._on_follower_state, qos_stream)
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.pose_enu_topic,
            self._on_pose_enu,
            qos_stream,
        )
        self.get_logger().info(
            'route_geo_projector node started: '
            f'pose_enu_topic={self.pose_enu_topic}, pose_llh_topic={self.pose_llh_topic}'
        )

    def _projection_diff_messages(
        self,
        expected: ProjectionConfig,
        actual: ProjectionConfig,
    ) -> list[str]:
        """projection設定差分をログ用文字列として返す."""

        diffs: list[str] = []

        def _check_str(name: str, left: str, right: str) -> None:
            if str(left) != str(right):
                diffs.append(f"{name}: configured={left}, route={right}")

        def _check_float(name: str, left: float, right: float, tolerance: float) -> None:
            if abs(float(left) - float(right)) > tolerance:
                diffs.append(f"{name}: configured={left}, route={right}")

        _check_str('projection_id', expected.projection_id, actual.projection_id)
        _check_str('datum', expected.datum, actual.datum)
        _check_str('map_frame_id', expected.map_frame_id, actual.map_frame_id)
        _check_str('earth_frame_id', expected.earth_frame_id, actual.earth_frame_id)
        _check_float(
            'origin_latitude',
            expected.origin_latitude,
            actual.origin_latitude,
            1.0e-9,
        )
        _check_float(
            'origin_longitude',
            expected.origin_longitude,
            actual.origin_longitude,
            1.0e-9,
        )
        _check_float(
            'origin_altitude',
            expected.origin_altitude,
            actual.origin_altitude,
            1.0e-3,
        )
        _check_float(
            'map_yaw_offset_rad',
            expected.map_yaw_offset_rad,
            actual.map_yaw_offset_rad,
            1.0e-9,
        )
        return diffs

    def _warn_frame_mismatch(self, kind: str, frame_id: str) -> None:
        """入力poseのframe_idがprojectionのmap frameと異なる場合に警告する."""

        expected = self.projection.map_frame_id
        if not frame_id or frame_id == expected:
            return
        if kind == 'pose_enu' and self._pose_frame_mismatch_logged:
            return
        if kind == 'active_target' and self._target_frame_mismatch_logged:
            return
        self.get_logger().warn(
            f'{kind} frame_id mismatch: '
            f'frame_id={frame_id}, projection.map_frame_id={expected}'
        )
        if kind == 'pose_enu':
            self._pose_frame_mismatch_logged = True
        elif kind == 'active_target':
            self._target_frame_mismatch_logged = True

    def _on_route(self, msg: Route) -> None:
        self.active_route = copy.deepcopy(msg)
        if msg.projection.projection_id:
            route_projection = projection_from_msg(msg.projection)
            diffs = self._projection_diff_messages(
                self.configured_projection,
                route_projection,
            )
            if diffs and not self._projection_mismatch_logged:
                self.get_logger().error(
                    'Route.projection differs from route_geo_projector parameters. '
                    'Using Route.projection for route display conversion. '
                    + '; '.join(diffs)
                )
                self._projection_mismatch_logged = True
            self.projection = route_projection

    def _on_follower_state(self, msg: FollowerState) -> None:
        self.follower_state = copy.deepcopy(msg)
        self._publish_if_ready()

    def _on_pose_enu(self, msg: PoseWithCovarianceStamped) -> None:
        self.pose_enu = copy.deepcopy(msg)
        self._warn_frame_mismatch('pose_enu', msg.header.frame_id)
        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = self.projection.earth_frame_id
        self.current_geo_pose = pose_to_llh_pose(
            header,
            msg.pose.pose,
            self.projection,
            self.pose_child_frame_id,
        )
        quality = make_geo_pose_quality(
            header,
            self.current_geo_pose,
            None,
            GeoPoseWithQuality.SOURCE_UNKNOWN,
            GeoPoseWithQuality.FUSION_OK,
        )
        quality.status_text = 'projected_from_enu'
        self.pub_pose_llh.publish(quality)
        self._publish_if_ready()

    def _on_target(self, msg: PoseStamped) -> None:
        self.active_target = copy.deepcopy(msg)
        self._warn_frame_mismatch('active_target', msg.header.frame_id)
        self._publish_if_ready()

    def _find_route_waypoint(self, index: int, label: str) -> Optional[Waypoint]:
        if self.active_route is None:
            return None
        if 0 <= index < len(self.active_route.waypoints):
            wp = self.active_route.waypoints[index]
            if not label or wp.label == label:
                return wp
        if label:
            for wp in self.active_route.waypoints:
                if wp.label == label:
                    return wp
        return None

    def _current_geo_pose(self):
        return self.current_geo_pose

    def _publish_if_ready(self) -> None:
        if self.active_target is None:
            return
        route_version = -1
        target_index = -1
        target_label = ''
        if self.follower_state is not None:
            route_version = int(self.follower_state.route_version)
            target_index = int(self.follower_state.active_waypoint_index)
            target_label = str(self.follower_state.active_waypoint_label)
        elif self.active_route is not None:
            route_version = int(self.active_route.version)

        header = Header()
        header.stamp = self.active_target.header.stamp
        header.frame_id = self.projection.earth_frame_id
        waypoint = self._find_route_waypoint(target_index, target_label)
        if waypoint is not None and waypoint.has_geo_pose:
            target_pose = copy.deepcopy(waypoint.geo_pose)
            target_pose.header = header
        else:
            target_pose = pose_to_llh_pose(
                header,
                self.active_target.pose,
                self.projection,
                'active_target',
            )

        msg = make_active_target_llh(
            header,
            route_version,
            target_index,
            target_label,
            target_pose,
            self._current_geo_pose(),
            self.projection,
            is_avoidance_subgoal=target_index < 0,
        )
        self.pub_target_llh.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RouteGeoProjectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
