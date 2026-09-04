"""robot_console を起動するための launch スクリプト。"""

import os
from pathlib import Path
from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_log_directory(context: LaunchContext) -> str:
    """ROS2 の規約に従い、ログディレクトリを決定する。"""

    for key in ('log_dir', 'ros_log_dir'):
        value = context.launch_configurations.get(key)
        if value:
            return str(Path(value).expanduser())
    env_log_dir = os.environ.get('ROS_LOG_DIR')
    if env_log_dir:
        return str(Path(env_log_dir).expanduser())
    ros_home = os.environ.get('ROS_HOME', str(Path.home() / '.ros'))
    return str(Path(ros_home).expanduser() / 'log')


_TOPIC_CONFIGS = [
    ('manual_start_topic', 'manual_start', '/manual_start', '手動開始トピック'),
    ('sig_recog_topic', 'sig_recog', '/sig_recog', '信号認識トピック'),
    ('road_blocked_topic', 'road_blocked', '/road_blocked', '通行止めトピック'),
    (
        'obstacle_hint_topic',
        'obstacle_avoidance_hint',
        '/obstacle_avoidance_hint',
        '障害物ヒントトピック',
    ),
    ('route_state_topic', 'route_state', '/route_state', '経路状態トピック'),
    (
        'manager_status_topic',
        'manager_status',
        '/manager_status',
        'マネージャ状態トピック',
    ),
    ('active_route_topic', 'active_route', '/active_route', '経路情報トピック'),
    ('follower_state_topic', 'follower_state', '/follower_state', '追従状態トピック'),
    ('sensor_viewer_topic', 'sensor_viewer', '/sensor_viewer', 'センサビューアトピック'),
    (
        'drive_camera_topic',
        'perception/road_blockage/decision_image',
        '/perception/road_blockage/decision_image',
        '道路封鎖判定画像トピック',
    ),
    (
        'signal_camera_topic',
        'perception/traffic_signal/decision_image',
        '/perception/traffic_signal/decision_image',
        '信号判定画像トピック',
    ),
    ('active_target_topic', 'active_target', '/active_target', 'ターゲット姿勢トピック'),
    ('pose_enu_topic', 'localization/pose_enu', '/localization/pose_enu', 'ENU自己位置トピック'),
    ('cmd_vel_topic', 'cmd_vel', '/cmd_vel', '速度指令トピック'),
    (
        'cmd_vel_autonomous_topic',
        'cmd_vel/autonomous',
        '/cmd_vel/autonomous',
        '自律速度指令トピック',
    ),
    (
        'drive_mode_status_topic',
        'drive_mode_status',
        '/drive_mode_status',
        '走行モード状態トピック',
    ),
    (
        'odom_topic',
        'odom',
        '/ypspur_ros/odom',
        'オドメトリトピック（実機のypspur_ros2、Gazebo bridge、robot_simulatorの'
        'いずれも既定で/ypspur_ros/odomへ publish する）',
    ),
]


def _launch_setup(context: LaunchContext, *args, **kwargs) -> List[Node]:
    """ノード定義を生成する内部関数。"""

    log_dir = _resolve_log_directory(context)
    remappings = [
        (from_name, LaunchConfiguration(arg_name)) for arg_name, from_name, *_ in _TOPIC_CONFIGS
    ]
    # 正式UIはPyQt5版（robot_console_qt）である。旧tkinter版（robot_console）は
    # entry pointとしては残すが、本launchからは起動しない。
    # 新UIはログ保存先をROSパラメータではなくCLI引数で受け取る（ConsoleCoreを
    # Node生成前に構築するため）。
    node = Node(
        package='robot_console',
        executable='robot_console_qt',
        name='robot_console_qt',
        output='screen',
        arguments=['--console-log-directory', log_dir],
        remappings=remappings,
    )
    return [node]


def generate_launch_description() -> LaunchDescription:
    """robot_console ノードを単独起動する。"""

    topic_arguments = [
        DeclareLaunchArgument(
            arg_name,
            default_value=default,
            description=description,
        )
        for arg_name, _, default, description in _TOPIC_CONFIGS
    ]

    return LaunchDescription(topic_arguments + [OpaqueFunction(function=_launch_setup)])
