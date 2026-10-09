"""記録ルートをICP主体で走行する。GNSS版と同じ経路ディレクトリを選ぶ。"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    defaults = dict(route_directory='/media/nkb/TEST/autonomous_route_20261005/route',
                    icp_map_manifest='/media/nkb/TEST/icp_runtime_20261007/map_manifest.json',
                    icp_backend_python_path='/home/nkb/colcon_ws_experiments/glim_runtime/python',
                    antenna_baseline_m='0', master_forward_m='',
                    output_root='/media/nkb/TEST/icp_route_runs')
    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()],
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            Path(get_package_share_directory('icart_bringup'))/'launch/recorded_route.launch.py')),
            launch_arguments=dict({k: LaunchConfiguration(k) for k in defaults},
                                  localization_mode='icp').items())])
