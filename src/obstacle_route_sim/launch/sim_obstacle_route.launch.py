#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""障害物付き経路追従検証用 Gazebo Sim launch."""

from __future__ import annotations

import os
import sys
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _as_bool(value: str) -> bool:
    """launch 引数文字列を bool に変換する."""

    return str(value).lower() in {"1", "true", "yes", "on"}


def _setup_simulation(context: LaunchContext, *args, **kwargs) -> list:
    """launch 引数を解決し、world 生成と起動 action を返す."""

    del args, kwargs

    pkg_share = Path(get_package_share_directory("obstacle_route_sim"))
    scripts_dir = Path(get_package_prefix("obstacle_route_sim")) / "lib" / "obstacle_route_sim"
    if str(scripts_dir) not in sys.path:
        sys.path.insert(0, str(scripts_dir))

    from generate_pylon_world import generate_world
    from road_geometry import world_template_name

    road_type = LaunchConfiguration("road_type").perform(context)
    road_width = float(LaunchConfiguration("road_width").perform(context))
    enable_pylons = _as_bool(LaunchConfiguration("enable_pylons").perform(context))
    pylon_seed = int(LaunchConfiguration("pylon_seed").perform(context))
    enable_route_blocker = _as_bool(LaunchConfiguration("enable_route_blocker").perform(context))
    route_blocker_distance = float(LaunchConfiguration("route_blocker_distance").perform(context))
    spawn_robot = _as_bool(LaunchConfiguration("spawn_robot").perform(context))
    robot_x = float(LaunchConfiguration("robot_x").perform(context))
    robot_y = float(LaunchConfiguration("robot_y").perform(context))
    robot_z = float(LaunchConfiguration("robot_z").perform(context))
    generated_world_dir = Path(LaunchConfiguration("generated_world_dir").perform(context))
    start_gazebo_gui = _as_bool(LaunchConfiguration("start_gazebo_gui").perform(context))
    use_sim_time = _as_bool(LaunchConfiguration("use_sim_time").perform(context))

    base_world = pkg_share / "worlds" / "templates" / world_template_name(road_type, road_width)
    world_name = base_world.stem
    gazebo_pose_topic = f"/world/{world_name}/dynamic_pose/info"
    generated_world = generated_world_dir / (
        f"{road_type}_w{int(road_width)}_seed{pylon_seed}.world"
    )
    generate_world(
        base_world=base_world,
        output=generated_world,
        road_type=road_type,
        road_width=road_width,
        enable_pylons=enable_pylons,
        seed=pylon_seed,
        spawn_robot=spawn_robot,
        robot_x=robot_x,
        robot_y=robot_y,
        robot_z=robot_z,
        enable_route_blocker=enable_route_blocker,
        route_blocker_distance=route_blocker_distance,
    )

    gz_args = f"-r {generated_world}"
    if not start_gazebo_gui:
        gz_args = f"-r -s {generated_world}"

    ros_gz_sim_launch = PathJoinSubstitutionCompat(
        FindPackageShare("ros_gz_sim"),
        "launch",
        "gz_sim.launch.py",
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ros_gz_sim_launch.as_path()),
        launch_arguments={"gz_args": gz_args}.items(),
    )

    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="obstacle_route_sim_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/ypspur_ros/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/mid360/livox/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            # AI 助言の入力に使うフロントカメラ。実機と同じ topic 名で配信する。
            "/usb_cam/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            f"{gazebo_pose_topic}@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        ],
        remappings=[(gazebo_pose_topic, "/gazebo/dynamic_pose_info")],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    fake_localization_node = Node(
        package="obstacle_route_sim",
        executable="fake_localization_pose.py",
        name="fake_localization_pose",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "pose_topic": "/gazebo/dynamic_pose_info",
                "pose_enu_topic": "/localization/pose_enu",
                "frame_id": "map",
                "target_pose_index": 0,
            }
        ],
    )

    odom_tf_node = Node(
        package="obstacle_route_sim",
        executable="odom_tf_broadcaster.py",
        name="odom_tf_broadcaster",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "odom_topic": "/ypspur_ros/odom",
                "parent_frame_id": "odom",
                "child_frame_id": "base_link",
            }
        ],
    )

    map_to_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_map_to_odom",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        parameters=[{"use_sim_time": use_sim_time}],
    )
    # センサの静的 TF は base_link（車軸中心、床上 0.15 m）からの相対値で与える。
    # models/robot/model.sdf の link pose は model frame（床面）基準であり、車輪半径の
    # 分だけ値が異なる。両者を取り違えると層の切り出し高さが 0.15 m ずれる。
    base_to_laser_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_to_laser",
        arguments=["0.075", "0", "0.10", "0", "0", "0", "base_link", "laser"],
        parameters=[{"use_sim_time": use_sim_time}],
    )
    base_to_mid360_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_to_mid360",
        arguments=["0", "0", "0.412", "-0.010472", "0.469494", "0",
                   "base_link", "mid360_frame"],
        parameters=[{"use_sim_time": use_sim_time}],
    )
    base_to_camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_to_camera",
        arguments=["0.115", "0", "0.35", "0", "0", "0", "base_link", "camera_link"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return [
        SetEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            os.pathsep.join(
                [
                    str(pkg_share),
                    str(pkg_share / "models"),
                    os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
                ]
            ),
        ),
        gazebo_launch,
        bridge_node,
        fake_localization_node,
        odom_tf_node,
        map_to_odom_tf,
        base_to_laser_tf,
        base_to_mid360_tf,
        base_to_camera_tf,
    ]


class PathJoinSubstitutionCompat:
    """PythonLaunchDescriptionSource へ渡す実パスを遅延解決する薄い互換クラス."""

    def __init__(self, package_share, *parts: str) -> None:
        """substitution と相対パスを保持する."""

        self.package_share = package_share
        self.parts = parts

    def as_path(self):
        """launch substitution 配列として返す."""

        return [self.package_share, *[f"/{part}" for part in self.parts]]


def generate_launch_description() -> LaunchDescription:
    """LaunchDescription を生成する."""

    return LaunchDescription(
        [
            DeclareLaunchArgument("road_type", default_value="crank"),
            DeclareLaunchArgument("road_width", default_value="5.0"),
            DeclareLaunchArgument("enable_pylons", default_value="true"),
            DeclareLaunchArgument("pylon_seed", default_value="0"),
            DeclareLaunchArgument("enable_route_blocker", default_value="false"),
            DeclareLaunchArgument("route_blocker_distance", default_value="8.0"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("spawn_robot", default_value="true"),
            DeclareLaunchArgument("robot_x", default_value="1.0"),
            DeclareLaunchArgument("robot_y", default_value="0.0"),
            DeclareLaunchArgument("robot_z", default_value="0.16"),
            DeclareLaunchArgument("generated_world_dir", default_value="/tmp/obstacle_route_sim"),
            DeclareLaunchArgument("keep_generated_world", default_value="true"),
            DeclareLaunchArgument("use_compat_remap", default_value="true"),
            DeclareLaunchArgument("start_gazebo_gui", default_value="true"),
            OpaqueFunction(function=_setup_simulation),
        ]
    )
