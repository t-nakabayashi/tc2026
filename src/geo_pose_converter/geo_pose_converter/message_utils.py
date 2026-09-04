#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""tc_geo_msgs/tc_route_msgs 生成の補助関数."""

from __future__ import annotations

from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header

from tc_route_msgs.msg import ActiveTargetLlh
from rtk_gps_um982_msgs.msg import RtkStatus
from tc_geo_msgs.msg import GeoPoint, GeoPose, GeoPoseWithQuality, MapProjection

from geo_pose_converter.geo_core import (
    LlhPoint,
    ProjectionConfig,
    bearing_from_map_delta,
    enu_to_llh_on_ground,
    heading_deg_to_yaw_enu_rad,
    llh_to_enu,
    quaternion_to_yaw,
    yaw_enu_rad_to_heading_deg,
    yaw_to_quaternion,
)


def projection_to_msg(config: ProjectionConfig, header: Header | None = None) -> MapProjection:
    """ProjectionConfigをMapProjection msgへ変換する."""

    msg = MapProjection()
    if header is not None:
        msg.header = header
    msg.projection_type = MapProjection.PROJECTION_LOCAL_TANGENT_PLANE
    msg.projection_id = config.projection_id
    msg.datum = config.datum
    msg.map_frame_id = config.map_frame_id
    msg.earth_frame_id = config.earth_frame_id
    msg.origin_latitude = float(config.origin_latitude)
    msg.origin_longitude = float(config.origin_longitude)
    msg.origin_altitude = float(config.origin_altitude)
    msg.map_yaw_offset_rad = float(config.map_yaw_offset_rad)
    msg.utm_zone = ""
    msg.utm_north = True
    return msg


def projection_from_msg(msg: MapProjection) -> ProjectionConfig:
    """MapProjection msgをProjectionConfigへ変換する."""

    return ProjectionConfig(
        origin_latitude=float(msg.origin_latitude),
        origin_longitude=float(msg.origin_longitude),
        origin_altitude=float(msg.origin_altitude),
        map_yaw_offset_rad=float(msg.map_yaw_offset_rad),
        projection_id=str(msg.projection_id),
        datum=str(msg.datum) or "WGS84",
        map_frame_id=str(msg.map_frame_id) or "map",
        earth_frame_id=str(msg.earth_frame_id) or "earth",
    )


def make_geo_point(point: LlhPoint, has_altitude: bool = True) -> GeoPoint:
    """LlhPointからGeoPoint msgを生成する."""

    msg = GeoPoint()
    msg.latitude = float(point.latitude)
    msg.longitude = float(point.longitude)
    msg.altitude = float(point.altitude)
    msg.has_altitude = bool(has_altitude)
    return msg


def make_geo_pose(
    header: Header,
    point: LlhPoint,
    heading_deg: float = 0.0,
    has_heading: bool = False,
    child_frame_id: str = "",
    has_altitude: bool = True,
) -> GeoPose:
    """GeoPose msgを生成する."""

    msg = GeoPose()
    msg.header = header
    msg.point = make_geo_point(point, has_altitude)
    msg.heading_deg = float(heading_deg)
    msg.has_heading = bool(has_heading)
    msg.yaw_enu_rad = float(heading_deg_to_yaw_enu_rad(heading_deg)) if has_heading else 0.0
    msg.has_yaw_enu = bool(has_heading)
    msg.child_frame_id = str(child_frame_id)
    return msg


def rtk_state_to_fix_quality(rtk_state: int) -> int:
    """RtkStatus.rtk_stateをGeoPoseWithQuality.fix_qualityへ変換する."""

    mapping = {
        RtkStatus.STATE_UNKNOWN: GeoPoseWithQuality.FIX_UNKNOWN,
        RtkStatus.STATE_STANDALONE: GeoPoseWithQuality.FIX_STANDALONE,
        RtkStatus.STATE_DGPS: GeoPoseWithQuality.FIX_DGPS,
        RtkStatus.STATE_RTK_FLOAT: GeoPoseWithQuality.FIX_RTK_FLOAT,
        RtkStatus.STATE_RTK_FIX: GeoPoseWithQuality.FIX_RTK_FIX,
    }
    return mapping.get(int(rtk_state), GeoPoseWithQuality.FIX_UNKNOWN)


def make_geo_pose_quality(
    header: Header,
    geo_pose: GeoPose,
    status: RtkStatus | None,
    source: int,
    fusion_status: int,
) -> GeoPoseWithQuality:
    """GeoPoseWithQuality msgを生成する."""

    msg = GeoPoseWithQuality()
    msg.header = header
    msg.pose = geo_pose
    msg.source = int(source)
    msg.fusion_status = int(fusion_status)
    if status is not None:
        msg.fix_quality = rtk_state_to_fix_quality(status.rtk_state)
        msg.num_satellites = int(status.num_satellites)
        msg.hdop = float(status.hdop)
        msg.correction_age_s = float(status.correction_age_s)
        msg.rtcm_bytes_received = int(status.rtcm_bytes_received)
        msg.heading_accuracy_deg = float(status.heading_stddev_deg)
        msg.status_text = str(status.rtk_state_raw)
    else:
        msg.fix_quality = GeoPoseWithQuality.FIX_UNKNOWN
        msg.num_satellites = 0
        msg.hdop = 0.0
        msg.correction_age_s = -1.0
        msg.rtcm_bytes_received = 0
        msg.heading_accuracy_deg = 0.0
        msg.status_text = ""
    msg.horizontal_accuracy_m = float(msg.hdop) if msg.hdop > 0.0 else 0.0
    msg.vertical_accuracy_m = msg.horizontal_accuracy_m * 2.0 if msg.horizontal_accuracy_m > 0.0 else 0.0
    return msg


def navsatfix_to_llh(msg: NavSatFix) -> LlhPoint:
    """NavSatFixからLlhPointを生成する."""

    return LlhPoint(float(msg.latitude), float(msg.longitude), float(msg.altitude))


def llh_to_pose_with_covariance(
    header: Header,
    point: LlhPoint,
    heading_deg: float,
    has_heading: bool,
    projection: ProjectionConfig,
) -> PoseWithCovarianceStamped:
    """LLH poseをmap ENUのPoseWithCovarianceStampedへ変換する.

    本プロジェクトでは走行用ENU poseを2Dとして扱うため、zは高度から復元せず0.0に
    正規化する。GNSS高度はLLH系topic側で保持する。

    水平座標(x, y)の算出にもGNSS実高度ではなく `origin_altitude` を用いる。
    local tangent planeでは同一の緯度経度でも高度が変わると水平座標がずれるため
    (原点距離d[m]に対し 高度差 × d/R)、高度規約が経路ごとに異なると同じ地点が
    別のENU座標になる。route waypointは `origin_altitude` 基準でENU化される
    (`route_builder._set_pose_from_llh()`) ため、自己位置側も同じ基準へ揃える。
    東京駅原点・つくば(52.7km)では、高度62mの実測値をそのまま使うと約0.48mの
    系統誤差になる。
    """

    ground_point = LlhPoint(point.latitude, point.longitude, projection.origin_altitude)
    enu = llh_to_enu(ground_point, projection)
    msg = PoseWithCovarianceStamped()
    msg.header = header
    msg.header.frame_id = projection.map_frame_id
    msg.pose.pose.position.x = enu.x
    msg.pose.pose.position.y = enu.y
    msg.pose.pose.position.z = 0.0
    yaw = heading_deg_to_yaw_enu_rad(heading_deg) - projection.map_yaw_offset_rad if has_heading else 0.0
    qx, qy, qz, qw = yaw_to_quaternion(yaw)
    msg.pose.pose.orientation.x = qx
    msg.pose.pose.orientation.y = qy
    msg.pose.pose.orientation.z = qz
    msg.pose.pose.orientation.w = qw
    return msg


def pose_to_llh_pose(
    header: Header,
    pose: Pose,
    projection: ProjectionConfig,
    child_frame_id: str = "",
    has_altitude: bool = False,
) -> GeoPose:
    """map ENU PoseをGeoPoseへ変換する.

    ENU poseは走行用2D座標として扱うため、既定では逆投影した高度を有効値としない。
    zも `llh_to_enu_pose()` が0.0へ正規化した値が入るため、そのまま逆投影すると
    原点から離れた地点で水平位置に誤差が出る(原点52.7km先で約1.8m)。
    水平座標(x, y)を地表点として扱う `enu_to_llh_on_ground()` を用いることで、
    `llh_to_enu_pose()` との往復を一致させる。基準高度は他経路（waypoint生成・
    GNSS自己位置）と同じ `origin_altitude` を用い、ENU⇔LLH変換の高度規約を統一する。
    """

    point = enu_to_llh_on_ground(
        float(pose.position.x),
        float(pose.position.y),
        projection,
        ground_altitude=projection.origin_altitude,
    )
    yaw_map = quaternion_to_yaw(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )
    heading = yaw_enu_rad_to_heading_deg(yaw_map + projection.map_yaw_offset_rad)
    return make_geo_pose(header, point, heading, True, child_frame_id, has_altitude)


def make_active_target_llh(
    header: Header,
    route_version: int,
    target_index: int,
    target_label: str,
    target_pose: GeoPose,
    current_pose: GeoPose | None,
    projection: ProjectionConfig,
    is_avoidance_subgoal: bool = False,
) -> ActiveTargetLlh:
    """ActiveTargetLlh msgを生成する."""

    msg = ActiveTargetLlh()
    msg.header = header
    msg.route_version = int(route_version)
    msg.target_index = int(target_index)
    msg.target_label = str(target_label)
    msg.target_pose = target_pose
    msg.is_avoidance_subgoal = bool(is_avoidance_subgoal)
    msg.target_kind = "avoidance_subgoal" if is_avoidance_subgoal else "route_waypoint"
    if current_pose is not None:
        current_altitude = (
            current_pose.point.altitude
            if current_pose.point.has_altitude
            else projection.origin_altitude
        )
        target_altitude = (
            target_pose.point.altitude
            if target_pose.point.has_altitude
            else projection.origin_altitude
        )
        cur = llh_to_enu(
            LlhPoint(
                current_pose.point.latitude,
                current_pose.point.longitude,
                current_altitude,
            ),
            projection,
        )
        tgt = llh_to_enu(
            LlhPoint(
                target_pose.point.latitude,
                target_pose.point.longitude,
                target_altitude,
            ),
            projection,
        )
        dx = tgt.x - cur.x
        dy = tgt.y - cur.y
        msg.distance_m = float((dx * dx + dy * dy) ** 0.5)
        msg.bearing_deg = float(bearing_from_map_delta(dx, dy, projection))
    else:
        msg.distance_m = 0.0
        msg.bearing_deg = 0.0
    return msg
