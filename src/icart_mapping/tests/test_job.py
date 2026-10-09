import json
from pathlib import Path
import sqlite3
import sys
import threading
import os
import signal

import pytest
import yaml

from icart_mapping import job


def bag_fixture(tmp_path):
    bag = tmp_path/'bag with spaces'
    bag.mkdir()
    with sqlite3.connect(bag/'data.db3') as db:
        db.execute('create table topics(name text,type text)')
        db.executemany('insert into topics values(?,?)', job.REQUIRED.items())
    (bag/'metadata.yaml').write_text(yaml.safe_dump({'rosbag2_bagfile_information': dict(
        storage_identifier='sqlite3', relative_file_paths=['data.db3'], duration={'nanoseconds':1000000000})}))
    return bag


def test_selected_bag_is_read_only_and_all_required_types_are_checked(tmp_path):
    bag = bag_fixture(tmp_path)
    before = (bag/'data.db3').read_bytes()
    summary = job.inspect_bag(bag)
    assert summary['duration_s'] == 1.
    assert summary['files'][0]['path'] == str(bag/'data.db3')
    assert (bag/'data.db3').read_bytes() == before
    with sqlite3.connect(bag/'data.db3') as db:
        db.execute('update topics set type=? where name=?', ('wrong/type', '/rtk_gps/fix'))
    with pytest.raises(ValueError, match='メッセージ型'):
        job.inspect_bag(bag)


def test_missing_topic_and_missing_partition_are_rejected(tmp_path):
    bag = bag_fixture(tmp_path)
    with sqlite3.connect(bag/'data.db3') as db:
        db.execute('delete from topics where name=?', ('/lio/odometry_raw',))
    with pytest.raises(ValueError, match='odometry_raw'):
        job.inspect_bag(bag)
    (bag/'data.db3').unlink()
    with pytest.raises(ValueError, match='ファイル'):
        job.inspect_bag(bag)


def test_path_outside_bag_is_rejected(tmp_path):
    bag = bag_fixture(tmp_path)
    (bag/'data.db3').rename(tmp_path/'data.db3')
    path = bag/'metadata.yaml'
    value = yaml.safe_load(path.read_text())
    value['rosbag2_bagfile_information']['relative_file_paths'] = ['../data.db3']
    path.write_text(yaml.safe_dump(value))
    with pytest.raises(ValueError, match='不正'):
        job.inspect_bag(bag)


def test_isolation_writes_only_job_directory_without_shell_interpolation(tmp_path):
    path = tmp_path/'job.json'
    path.write_text(json.dumps({'python': '/some venv/bin/python3'}))
    command = job.isolated_command(path)
    assert command[:4] == ['bwrap', '--ro-bind', '/', '/']
    assert command[4:7] == ['--bind', str(tmp_path), str(tmp_path)]
    assert '--unshare-net' in command and '--unshare-pid' in command and '--die-with-parent' in command
    assert command.count('--bind') == 1
    assert '/some venv/bin/python3' in command
    assert not any(x in command for x in ('bash', '-c', 'ros2'))


def test_supervisor_failure_and_cancellation_never_mark_complete(tmp_path, monkeypatch):
    path = tmp_path/'job.json'
    path.write_text('{}')
    job.atomic_json(tmp_path/'status.json', dict(state='running', phase='test', percent=10))
    monkeypatch.setattr(job, 'isolated_command', lambda _: [sys.executable, '-c', 'raise SystemExit(4)'])
    assert job.run(path) == 1
    assert json.loads((tmp_path/'status.json').read_text())['state'] == 'failed'
    job.atomic_json(tmp_path/'status.json', dict(state='running', phase='test', percent=10))
    monkeypatch.setattr(job, 'isolated_command', lambda _: [sys.executable, '-c', 'import time; time.sleep(30)'])
    timer = threading.Timer(.25, lambda: os.kill(os.getpid(), signal.SIGTERM))
    timer.start()
    try:
        assert job.run(path) == 130
    finally:
        timer.cancel()
    assert json.loads((tmp_path/'status.json').read_text())['state'] == 'cancelled'


def test_mapping_geometry_matches_recorded_vehicle_transform():
    import numpy as np
    from icart_mapping.mount import geometry
    from scipy.spatial.transform import Rotation
    hardware = dict(mid360=dict(xyz=[0, 0, .414], rpy_deg=[-.6, 26.9, 0],
                               lidar_in_imu_xyz=[-.011, -.02329, .04412]),
                    gnss=dict(master_above_lidar_m=.15, master_forward_m=-.3))
    value = geometry(hardware)
    r = Rotation.from_euler('xyz', hardware['mid360']['rpy_deg'], degrees=True).as_matrix()
    np.testing.assert_allclose(r.T @ (np.array(value['master'])-value['imu']),
                               r.T @ np.array([-.3, 0, .15])+hardware['mid360']['lidar_in_imu_xyz'])
    hardware['mid360']['xyz'][0] = float('nan')
    with pytest.raises(ValueError):
        geometry(hardware)
