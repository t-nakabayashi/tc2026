#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""geo_pose_converterノード群を起動するlaunchファイル."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('geo_pose_converter'),
        'params',
        'default.yaml',
    )
    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='geo_pose_converter parameter YAML path',
    )
    enable_converter_arg = DeclareLaunchArgument(
        'enable_geo_pose_converter',
        default_value='true',
        description='GNSS raw topic を LLH/ENU に変換する geo_pose_converter_node を起動する',
    )
    gnss_pose_llh_topic_arg = DeclareLaunchArgument(
        'gnss_pose_llh_topic',
        default_value='/gnss/pose_llh',
        description='geo_pose_converter_node が publish する GNSS 単独 LLH pose topic',
    )
    gnss_pose_enu_topic_arg = DeclareLaunchArgument(
        'gnss_pose_enu_topic',
        default_value='/localization/pose_enu',
        description=(
            'geo_pose_converter_node が publish する GNSS 単独 ENU pose topic。'
            'localization_fusion 実装前は /localization/pose_enu、'
            'localization_fusion 実装後は /gnss/pose_enu を指定する'
        ),
    )
    pose_enu_topic_arg = DeclareLaunchArgument(
        'pose_enu_topic',
        default_value='/localization/pose_enu',
        description=(
            'route_geo_projector_node が購読する ENU 自己位置 topic。'
            '既定では /localization/pose_enu を指定する'
        ),
    )
    pose_llh_topic_arg = DeclareLaunchArgument(
        'pose_llh_topic',
        default_value='/localization/pose_llh',
        description='route_geo_projector_node が publish する LLH 自己位置 topic',
    )
    active_target_llh_topic_arg = DeclareLaunchArgument(
        'active_target_llh_topic',
        default_value='/route/active_target_llh',
        description='route_geo_projector_node が publish する active target LLH topic',
    )
    map_projection_topic_arg = DeclareLaunchArgument(
        'map_projection_topic',
        default_value='/geo/map_projection',
        description='geo_pose_converter_node が publish する map projection topic',
    )
    enable_llh_osm_viewer_arg = DeclareLaunchArgument(
        'enable_llh_osm_viewer',
        default_value='false',
        description='LLH自己位置とrouteをOSM上に表示する llh_osm_viewer_node を起動する',
    )
    llh_osm_viewer_host_arg = DeclareLaunchArgument(
        'llh_osm_viewer_host',
        default_value='127.0.0.1',
        description='llh_osm_viewer_node のHTTP listen host',
    )
    llh_osm_viewer_port_arg = DeclareLaunchArgument(
        'llh_osm_viewer_port',
        default_value='8765',
        description='llh_osm_viewer_node のHTTP listen port',
    )
    llh_osm_viewer_open_browser_arg = DeclareLaunchArgument(
        'llh_osm_viewer_open_browser',
        default_value='true',
        description='llh_osm_viewer_node 起動時にブラウザを開くか',
    )

    config = LaunchConfiguration('config')
    enable_converter = LaunchConfiguration('enable_geo_pose_converter')
    gnss_pose_llh_topic = LaunchConfiguration('gnss_pose_llh_topic')
    gnss_pose_enu_topic = LaunchConfiguration('gnss_pose_enu_topic')
    pose_enu_topic = LaunchConfiguration('pose_enu_topic')
    pose_llh_topic = LaunchConfiguration('pose_llh_topic')
    active_target_llh_topic = LaunchConfiguration('active_target_llh_topic')
    map_projection_topic = LaunchConfiguration('map_projection_topic')
    enable_llh_osm_viewer = LaunchConfiguration('enable_llh_osm_viewer')
    llh_osm_viewer_host = LaunchConfiguration('llh_osm_viewer_host')
    llh_osm_viewer_port = LaunchConfiguration('llh_osm_viewer_port')
    llh_osm_viewer_open_browser = LaunchConfiguration('llh_osm_viewer_open_browser')

    return LaunchDescription([
        config_arg,
        enable_converter_arg,
        gnss_pose_llh_topic_arg,
        gnss_pose_enu_topic_arg,
        pose_enu_topic_arg,
        pose_llh_topic_arg,
        active_target_llh_topic_arg,
        map_projection_topic_arg,
        enable_llh_osm_viewer_arg,
        llh_osm_viewer_host_arg,
        llh_osm_viewer_port_arg,
        llh_osm_viewer_open_browser_arg,
        Node(
            package='geo_pose_converter',
            executable='geo_pose_converter_node',
            name='geo_pose_converter',
            output='screen',
            parameters=[config],
            remappings=[
                ('rtk_gps/fix', '/rtk_gps/fix'),
                ('rtk_gps/heading', '/rtk_gps/heading'),
                ('rtk_gps/rtk_status', '/rtk_gps/rtk_status'),
                ('gnss/pose_llh', gnss_pose_llh_topic),
                ('gnss/pose_enu', gnss_pose_enu_topic),
                ('geo/map_projection', map_projection_topic),
            ],
            condition=IfCondition(enable_converter),
        ),
        Node(
            package='geo_pose_converter',
            executable='route_geo_projector_node',
            name='route_geo_projector',
            output='screen',
            parameters=[
                config,
                {
                    'pose_enu_topic': pose_enu_topic,
                    'pose_llh_topic': pose_llh_topic,
                },
            ],
            remappings=[
                ('active_route', '/active_route'),
                ('active_target', '/active_target'),
                ('follower_state', '/follower_state'),
                ('route/active_target_llh', active_target_llh_topic),
            ],
        ),
        Node(
            package='geo_pose_converter',
            executable='llh_osm_viewer_node',
            name='llh_osm_viewer',
            output='screen',
            parameters=[
                {
                    'pose_llh_topic': pose_llh_topic,
                    'active_route_topic': '/active_route',
                    'active_target_llh_topic': active_target_llh_topic,
                    'http_host': llh_osm_viewer_host,
                    'http_port': llh_osm_viewer_port,
                    'open_browser': llh_osm_viewer_open_browser,
                },
            ],
            condition=IfCondition(enable_llh_osm_viewer),
        ),
    ])
