"""共通位置推定・UIを使ったJoy経路採取。初期モードはmanualとする."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from icart_bringup.session_core import load_session


def setup(context):
    environment = LaunchConfiguration('environment').perform(context)
    session = LaunchConfiguration('session').perform(context)
    data = load_session(Path(session), environment)
    gps = '/rtk_gps' if environment == 'simulation' else data.get(
        'gnss_namespace', '/rtk_gps')
    joy_input = LaunchConfiguration('joy_input').perform(context)
    actions = [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
        Path(get_package_share_directory('icart_bringup'))/'launch/bringup.launch.py')),
        launch_arguments={'environment': environment, 'session': session,
                          'start_ui': context.launch_configurations.get('start_ui', 'true'), 'initial_drive_mode': 'manual',
                          'allow_auto_resume': 'false', 'start_teleop': 'true',
                          'joy_input': joy_input}.items()),
        Node(package='route_survey', executable='recorder', output='screen', parameters=[
            LaunchConfiguration('recorder_params').perform(context),
            *([data['recorder_params']] if data.get('recorder_params') else []),
            {'output_directory': LaunchConfiguration('output').perform(context),
             'projection_config': data['projection_params'],
             'gnss_fix_topic': gps+'/fix', 'gnss_status_topic': gps+'/rtk_status',
             'use_sim_time': environment=='simulation'}])]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('environment', default_value='simulation', choices=['simulation','real']),
        DeclareLaunchArgument('start_ui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('session'), DeclareLaunchArgument('output'),
        DeclareLaunchArgument('recorder_params', default_value=str(
            Path(get_package_share_directory('route_survey'))/'params/default.yaml')),
        DeclareLaunchArgument('joy_input', default_value='joy_node',
                              choices=['joy_node','external']),
        OpaqueFunction(function=setup)])
