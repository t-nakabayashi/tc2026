"""ypspur_node 起動用 launch file."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('ypspur_ros2')
    default_config = os.path.join(pkg_share, 'config', 'default.yaml')

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='Path to parameter YAML file',
    )

    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='cmd_vel',
        description='ypspur_node が購読する Twist topic',
    )
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/ypspur_ros/odom',
        description=(
            'ypspur_node が publish する Odometry topic。'
            'robot_navigator・obstacle_route_sim(Gazebo bridge)・robot_simulator が'
            '/ypspur_ros/odom を前提としているため、既定でこれに合わせる'
        ),
    )
    start_coordinator_arg = DeclareLaunchArgument(
        'start_coordinator',
        default_value='false',
        description='true の場合 ypspur-coordinator も同時起動する',
    )
    coordinator_device_arg = DeclareLaunchArgument(
        'coordinator_device',
        default_value='/dev/ttyACM0',
        description='ypspur-coordinator に渡す device path',
    )
    coordinator_param_arg = DeclareLaunchArgument(
        'coordinator_param',
        default_value='',
        description='ypspur-coordinator に渡す robot parameter file path',
    )

    coordinator = ExecuteProcess(
        cmd=[
            'ypspur-coordinator',
            '-d',
            LaunchConfiguration('coordinator_device'),
            '-p',
            LaunchConfiguration('coordinator_param'),
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_coordinator')),
    )
    node = Node(
        package='ypspur_ros2',
        executable='ypspur_node',
        name='ypspur_node',
        parameters=[LaunchConfiguration('config')],
        remappings=[
            ('cmd_vel', LaunchConfiguration('cmd_vel_topic')),
            ('odom', LaunchConfiguration('odom_topic')),
        ],
        output='screen',
        emulate_tty=True,
        condition=UnlessCondition(LaunchConfiguration('start_coordinator')),
    )

    delayed_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='ypspur_ros2',
                executable='ypspur_node',
                name='ypspur_node',
                parameters=[LaunchConfiguration('config')],
                remappings=[
                    ('cmd_vel', LaunchConfiguration('cmd_vel_topic')),
                    ('odom', LaunchConfiguration('odom_topic')),
                ],
                output='screen',
                emulate_tty=True,
            ),
        ],
        condition=IfCondition(LaunchConfiguration('start_coordinator')),
    )

    return LaunchDescription([
        config_arg,
        cmd_vel_topic_arg,
        odom_topic_arg,
        start_coordinator_arg,
        coordinator_device_arg,
        coordinator_param_arg,
        coordinator,
        node,
        delayed_node,
    ])
