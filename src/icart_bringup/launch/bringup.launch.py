"""共通の走行・UI接口を保ち、実機とGazeboのセンサ入力だけを切り替える."""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable, RegisterEventHandler, EmitEvent)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from icart_bringup.session_core import load_session, validate_domain
from icart_bringup.hardware_core import read_yaml, validate_runtime
from icart_bringup.localization_config import localization_settings


def setup(context) -> list:
    """設定を先に検査し、シミュレーションから実機ドライバを起動させない."""
    environment = LaunchConfiguration('environment').perform(context)
    data = load_session(Path(LaunchConfiguration('session').perform(context)), environment)
    validate_domain(data, environment, int(os.environ.get('ROS_DOMAIN_ID', '0')))
    simulation = environment == 'simulation'
    share = Path(get_package_share_directory('obstacle_route_sim'))
    gps_base = '/rtk_gps' if simulation else data.get('gnss_namespace', '/rtk_gps')
    projection = data['projection_params']
    localization_mode, icp_parameters = localization_settings(
        data, context.launch_configurations.get('localization_mode', ''),
        context.launch_configurations.get('icp_map_manifest', ''),
        context.launch_configurations.get('icp_backend_python_path', ''))
    hardware = read_yaml(Path(data['hardware_config'])) if not simulation and data.get('hardware_config') else None
    if hardware:
        validate_runtime(hardware)
    actions = []
    def node(package: str, executable: str, params: list = None, remaps: list = None) -> Node:
        return Node(package=package, executable=executable, output='screen',
                    parameters=[*(params or []), {'use_sim_time': simulation}],
                    remappings=remaps or [])
    if simulation:
        directory = Path(data['trial_directory'])
        actions += [SetEnvironmentVariable('GZ_PARTITION', 'icart_session_'+str(os.getpid())),
                    SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
                    SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', str(directory)+os.pathsep+
                                           os.environ.get('GZ_SIM_RESOURCE_PATH', '')),
                    ExecuteProcess(cmd=['gz', 'sim', '-s', '-r',
                                   '--physics-engine', data.get('physics_engine', 'gz-physics-dartsim-plugin'),
                                   data.get('trial_sdf', str(directory/'trial.sdf'))],
                                   output='screen'),
                    Node(package='ros_gz_bridge', executable='parameter_bridge', arguments=[
                        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                        '/model/icart_mini/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                        '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                        '/ypspur_ros/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                        '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                        '/body_contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
                        '/sim/mid360/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                        '/mid360/livox/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'],
                         remappings=[('/model/icart_mini/pose', '/truth')], output='screen'),
                    node('obstacle_route_sim', 'lio_sensor_adapter_node.py', [{
                        'noise_profile': data.get('noise_profile', 'conservative'),
                        'noise_seed': int(data.get('noise_seed', 1))}]),
                    node('obstacle_route_sim', 'gnss_simulator_node.py', [projection, {
                        'buildings_json': str(directory/'world.json') if data.get('building_gnss', True) else '',
                        'heading_reference': 'antenna_baseline',
                        'start_fix_radius_m': float(data.get('gnss_start_fix_radius_m', 10.)),
                        'baseline_sigma_m': .002, 'baseline_float_sigma_m': .04,
                        **{key: float(data.get(key, 0.)) for key in [
                            'heading_fault_deg', 'heading_fault_after_s', 'heading_fault_duration_s']}}])]
        actions.append(node('obstacle_route_sim', 'pedestrian_simulator_node.py', [{
            'world_sdf': data.get('trial_sdf', str(directory/'trial.sdf')),
            'world_json': str(directory/'world.json'),
            'density': float(data.get('pedestrian_density', .1)),
            'seed': int(data.get('pedestrian_seed', 42))}]))
        fastlio = str(share/'params/fastlio_gazebo.yaml')
    else:
        # 接続先・デバイス設定は既存の実機launchへ委譲する。空なら別起動のドライバを使う。
        if data.get('hardware_launch'):
            actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(data['hardware_launch']),
                launch_arguments=({'hardware_config': data['hardware_config'], 'skip_clock_sensors': 'true'}.items() if hardware else [])))
        fastlio = data.get('fastlio_config') or str(
            Path(get_package_share_directory('fast_lio'))/'config/mid360.yaml')
    actions += [node('fast_lio', 'fastlio_mapping', [fastlio,
                {'pcd_save.pcd_save_en': False, 'publish.scan_publish_en': True,
                 'publish.scan_bodyframe_pub_en': True}], [('/Odometry', '/lio/odometry_raw')])]
    if localization_mode == 'icp':
        actions += [node('icp_localization', 'localization_node', [
                    *([data['icp_params']] if data.get('icp_params') else []),
                    projection, icp_parameters], [('/rtk_gps/fix', gps_base+'/fix'),
                        ('/rtk_gps/rtk_status', gps_base+'/rtk_status')])]
    else:
        actions += [node('gnss_lio_fusion', 'gravity_alignment_node', remaps=[
                    ('/mid360/livox/imu', '/sim/lio/imu')] if simulation else []),
                node('gnss_lio_fusion', 'fusion_node', [
                    str(Path(get_package_share_directory('gnss_lio_fusion'))/'params/default.yaml'),
                    projection, {'gnss_heading_offset_deg': 180.},
                    *([data['fusion_params']] if data.get('fusion_params') else []),
                    {'output_log': LaunchConfiguration('fusion_log').perform(context), 'require_gravity_alignment': True}],
                     [('/rtk_gps/fix', gps_base+'/fix'),
                                  ('/rtk_gps/rtk_status', gps_base+'/rtk_status')])]
    actions += [node('geo_pose_converter', 'geo_pose_converter_node', [projection],
                     [('gnss/pose_enu', '/gnss/pose_enu'),
                      ('rtk_gps/fix', gps_base+'/fix'), ('rtk_gps/heading', gps_base+'/heading'),
                      ('rtk_gps/rtk_status', gps_base+'/rtk_status')]),
                node('geo_pose_converter', 'route_geo_projector_node', [projection]),
                node('route_planner', 'route_planner', [{
                    'config_yaml_path': data['route_config'], 'csv_base_dir': data['csv_base_dir'],
                    'projection_config_path': projection}]),
                node('route_manager', 'route_manager', [{
                    'start_label': str(data.get('start_label', '0')),
                    'goal_label': str(data['goal_label']), 'checkpoint_labels': ['']}]),
                node('route_follower', 'route_follower', [{'start_immediately': False}]),
                node('obstacle_monitor', 'obstacle_monitor'),
                node('robot_navigator', 'robot_navigator', params=[{
                    'pose_timeout_sec': 1., 'odom_timeout_sec': 1.,
                    'require_motion_limits': not simulation,
                    'obstacle_timeout_sec': 1. if hardware else 0.}], remaps=[
                    ('odom', '/ypspur_ros/odom'), ('cmd_vel', '/cmd_vel/autonomous')]),
                node('drive_mode_manager', 'drive_cmd_mux_node', [{'initial_mode': LaunchConfiguration('initial_drive_mode').perform(context),
                    'allow_auto_resume': context.launch_configurations.get('allow_auto_resume', 'true') == 'true',
                    **({'l1_button_index': hardware['joy']['enable_button'],
                        'ps_button_index': hardware['joy']['ps_button']} if hardware else {})}],
                     [('cmd_vel/autonomous', '/cmd_vel/autonomous'
                       if localization_mode == 'icp' else '/cmd_vel/fusion_limited')])]
    if hardware or context.launch_configurations.get('start_teleop', 'false') == 'true':
        joy = hardware['joy'] if hardware else {}
        actions.append(node('drive_mode_manager', 'manual_teleop_node', [{
            key: joy[key] for key in ['linear_axis', 'angular_axis', 'enable_button',
                                     'linear_scale', 'angular_scale'] if key in joy}]))
        if context.launch_configurations.get('joy_input', 'joy_node') == 'joy_node':
            actions.append(node('joy', 'joy_node', [{
                'device_id': joy.get('device_id', 0), 'device_name': joy.get('device_name', ''),
                'deadzone': .05, 'autorepeat_rate': 20.}]))
    if LaunchConfiguration('start_ui').perform(context).lower() == 'true':
        actions.append(Node(package='robot_console', executable='robot_console_qt',
                            output='screen', parameters=[{'use_sim_time': simulation}],
                            arguments=['--business-environment',
                                       'デジタルツイン' if simulation else
                                       ('実機（ICP）' if localization_mode == 'icp' else '実機（融合）')],
                            remappings=[('odom', '/ypspur_ros/odom'),
                                        ('rtk_gps/rtk_status', gps_base+'/rtk_status'),
                                        ('rtk_gps/ntrip_status', gps_base+'/ntrip_status')]))
    # センサや制御プロセスが終了した際に、残りの起動群だけ走り続ける状態を避ける。
    actions += [RegisterEventHandler(OnProcessExit(target_action=action,
                on_exit=[EmitEvent(event=Shutdown(reason='共通走行プロセス終了'))]))
                for action in list(actions) if isinstance(action, ExecuteProcess)]
    if hardware:
        # Only GNSS/LiDAR start before synchronization. Wheel drivers, LIO,
        # fusion, navigator and joystick/mux wait for live raw-packet evidence.
        waiter = ExecuteProcess(cmd=['python3', '-m', 'icart_bringup.clock_gate', 'wait'],
                                output='screen')
        watcher = ExecuteProcess(cmd=['python3', '-m', 'icart_bringup.clock_gate', 'watch'],
                                 output='screen')
        actions += [RegisterEventHandler(OnProcessExit(target_action=watcher,
            on_exit=[EmitEvent(event=Shutdown(reason='時刻同期監視が終了'))])), watcher]
        def after_clock(event, _context):
            if event.returncode != 0:
                return [EmitEvent(event=Shutdown(reason='時刻同期が成立しないため起動中止'))]
            return actions
        sensors = IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            Path(get_package_share_directory('icart_bringup'))/'launch/clock_sensors.launch.py')),
            launch_arguments={'session_directory': str(Path(data['hardware_config']).parent)}.items())
        return [RegisterEventHandler(OnProcessExit(target_action=waiter, on_exit=after_clock)),
                sensors, waiter]
    return actions


def generate_launch_description() -> LaunchDescription:
    """DDS domainはUIを含む親シェルと一致させ、途中では変更しない."""
    return LaunchDescription([
        DeclareLaunchArgument('environment', default_value='simulation', choices=['simulation', 'real']),
        DeclareLaunchArgument('session', description='prepare_sessionで生成したsession.yaml'),
        DeclareLaunchArgument('fusion_log', default_value='', description='任意の融合JSONL保存先'),
        DeclareLaunchArgument('localization_mode', default_value='', description='空はsession設定、gnssまたはicp'),
        DeclareLaunchArgument('icp_map_manifest', default_value='', description='ENU地図のmanifest JSON'),
        DeclareLaunchArgument('icp_backend_python_path', default_value='', description='隔離small_gicpのPythonディレクトリ（任意）'),
        DeclareLaunchArgument('initial_drive_mode', default_value='autonomous',
                              choices=['autonomous', 'manual']),
        DeclareLaunchArgument('start_teleop', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('joy_input', default_value='joy_node', choices=['joy_node', 'external']),
        DeclareLaunchArgument('allow_auto_resume', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('start_ui', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=setup)])
