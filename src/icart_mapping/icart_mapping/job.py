"""Job preparation and an isolated, cancellable subprocess supervisor."""
import argparse
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import sqlite3
import subprocess
import sys
import uuid

import yaml

DEFAULTS = dict(
    bag='/media/nkb/TEST/tc2026_20261004_full_course_manual_logging_merged',
    projection='/media/nkb/TEST/autonomous_route_20261005/route/projection.yaml',
    hardware='/media/nkb/TEST/autonomous_route_20261005/session/hardware.yaml',
    output_root='/media/nkb/TEST/maps',
    python='/home/nkb/.cache/tc2026-graph-slam-venv/bin/python3',
    backend='/home/nkb/colcon_ws_experiments/glim_runtime/python')
REQUIRED = {
    '/lio/odometry_raw': 'nav_msgs/msg/Odometry',
    '/rtk_gps/fix': 'sensor_msgs/msg/NavSatFix',
    '/rtk_gps/rtk_status': 'rtk_gps_um982_msgs/msg/RtkStatus',
    '/mid360/livox/imu': 'sensor_msgs/msg/Imu',
    '/mid360/livox/lidar': 'livox_ros_driver2/msg/CustomMsg'}


def atomic_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2))
    temporary.replace(path)


def inspect_bag(directory):
    bag = Path(directory).expanduser().resolve()
    metadata = bag/'metadata.yaml'
    info = yaml.safe_load(metadata.read_text())['rosbag2_bagfile_information']
    if info['storage_identifier'] != 'sqlite3':
        raise ValueError('SQLite3形式のROS 2 bagを選択してください')
    files = []
    found = {}
    for name in info['relative_file_paths']:
        path = (bag/name).resolve()
        if not path.is_relative_to(bag) or not path.is_file():
            raise ValueError('bag内のデータファイルが不足または不正です')
        with sqlite3.connect(path.as_uri()+'?mode=ro', uri=True) as db:
            topics = dict(db.execute('select name,type from topics'))
            for topic, expected in REQUIRED.items():
                if topic in topics and topics[topic] != expected:
                    raise ValueError(f'未対応のメッセージ型: {topic}: {topics[topic]}')
            found.update(topics)
        stat = path.stat()
        files.append(dict(path=str(path), size=stat.st_size, mtime_ns=stat.st_mtime_ns))
    missing = set(REQUIRED)-set(found)
    if missing or not files:
        raise ValueError('地図作成に必要な記録がありません: '+', '.join(sorted(missing)))
    return dict(directory=str(bag), files=files, metadata_sha256=hashlib.sha256(metadata.read_bytes()).hexdigest(),
                duration_s=info['duration']['nanoseconds']/1e9)


def create_job(settings):
    values = dict(DEFAULTS, **settings)
    bag = inspect_bag(values['bag'])
    from geo_pose_converter.geo_core import load_projection_config_from_yaml
    from icart_mapping.mount import geometry
    projection_path = Path(values['projection']).expanduser().resolve()
    hardware_path = Path(values['hardware']).expanduser().resolve()
    projection = vars(load_projection_config_from_yaml(str(projection_path)))
    if abs(projection['map_yaw_offset_rad']) > 1e-12:
        raise ValueError('地図作成はENU軸（map_yaw_offset_rad=0）に対応しています')
    hardware = yaml.safe_load(hardware_path.read_text())
    geometry(hardware)
    # Keep the venv interpreter path: resolving its symlink loses the venv.
    python = Path(values['python']).expanduser().absolute()
    backend = Path(values['backend']).expanduser().resolve()
    if not python.is_file() or not os.access(python, os.X_OK):
        raise ValueError('地図作成用Python環境が見つかりません')
    if not list(backend.glob('small_gicp*.so')):
        raise ValueError('small_gicpのPythonフォルダを確認してください')
    if not shutil.which('bwrap'):
        raise ValueError('隔離実行に必要なbwrapがありません')
    output = Path(values['output_root']).expanduser().resolve()
    if not output.is_relative_to('/media'):
        raise ValueError('地図・ログの保存先は/media内を選んでください')
    if output.is_relative_to(bag['directory']):
        raise ValueError('出力先に入力bag内を指定できません')
    output.mkdir(parents=True, exist_ok=True)
    if shutil.disk_usage(output).free < 2*1024**3:
        raise ValueError('地図作成には保存先に2 GB以上の空きが必要です')
    directory = output/('map_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8])
    directory.mkdir()
    shutil.copyfile(projection_path, directory/'projection.yaml')
    shutil.copyfile(hardware_path, directory/'hardware.yaml')
    config = dict(schema=1, directory=str(directory), bag=bag, projection=projection,
                  hardware=hardware, python=str(python), backend=str(backend),
                  sigma_xy_m=.2, map_voxel_m=.15)
    atomic_json(directory/'job.json', config)
    atomic_json(directory/'status.json', dict(state='queued', phase='準備済み', percent=0))
    return directory/'job.json'


def isolated_command(config_path):
    from ament_index_python.packages import get_package_share_directory
    path = Path(config_path).resolve()
    config = json.loads(path.read_text())
    directory = str(path.parent)
    worker = str(Path(get_package_share_directory('icart_mapping'))/'pipeline/worker.py')
    return ['bwrap', '--ro-bind', '/', '/', '--bind', directory, directory,
            '--dev', '/dev', '--proc', '/proc', '--tmpfs', '/tmp', '--tmpfs', '/run',
            '--unshare-net', '--unshare-pid', '--die-with-parent',
            '--setenv', 'HOME', '/tmp', '--setenv', 'PYTHONDONTWRITEBYTECODE', '1',
            '--setenv', 'MPLCONFIGDIR', '/tmp/matplotlib', '--setenv', 'XDG_CACHE_HOME', '/tmp/cache',
            '--setenv', 'OPENBLAS_NUM_THREADS', '1', '--setenv', 'OMP_NUM_THREADS', '2',
            '--setenv', 'ICART_MAPPING_CONFIG', str(path),
            config['python'], '-u', worker]


def run(config_path):
    path = Path(config_path).resolve()
    cancelled = False
    process = None
    def stop(*_):
        nonlocal cancelled
        cancelled = True
        if process is not None and process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
    previous = {sig: signal.signal(sig, stop) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        with (path.parent/'build.log').open('w') as log:
            process = subprocess.Popen(isolated_command(path), stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, text=True, errors='replace', start_new_session=True)
            if cancelled:
                stop()
            for line in process.stdout:
                log.write(line); log.flush()
                print(line, end='', flush=True)
            code = process.wait()
        status_path = path.parent/'status.json'
        status = json.loads(status_path.read_text())
        if cancelled or code != 0 or status.get('state') != 'complete':
            state = 'cancelled' if cancelled else 'failed'
            status.update(state=state, phase='中止しました' if cancelled else '失敗しました',
                          error=status.get('error') or f'処理終了コード: {code}')
            atomic_json(status_path, status)
            return 130 if cancelled else 1
        return 0
    except Exception as error:
        atomic_json(path.parent/'status.json', dict(state='failed', phase='起動失敗', percent=0, error=str(error)))
        raise
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
        for sig, handler in previous.items():
            signal.signal(sig, handler)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--job', type=Path, required=True)
    args = parser.parse_args()
    sys.exit(run(args.job))


if __name__ == '__main__':
    main()
