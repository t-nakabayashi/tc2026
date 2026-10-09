import time
from types import SimpleNamespace
import pytest
from PyQt5 import QtWidgets
from robot_console.core.bag_recorder import BagRecorder, BagState
from robot_console.ui_qt.widgets.bag_card import BagCard


def test_card_disables_duplicate_start_and_preserves_destination():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    card = BagCard()
    card.update_state(BagState(directory='/tmp/bags'))
    card.directory.setText('/tmp/my_bags')
    calls = []
    card.start_requested.connect(calls.append)
    card.start_button.click()
    assert calls == ['/tmp/my_bags']
    card.update_state(BagState(state='RECORDING', directory='/tmp/bags'))
    assert card.directory.text() == '/tmp/my_bags'
    assert not card.start_button.isEnabled() and card.stop_button.isEnabled()
    card.update_state(BagState(state='STOPPING'))
    assert not card.start_button.isEnabled() and not card.stop_button.isEnabled()
    card.update_state(BagState(state='SAVED'))
    assert card.start_button.isEnabled()


def test_low_disk_refuses_recording_without_process(tmp_path, monkeypatch):
    monkeypatch.setattr('robot_console.core.bag_recorder.shutil.disk_usage',
                        lambda _: SimpleNamespace(free=100))
    recorder = BagRecorder(tmp_path)
    assert not recorder.start()
    assert recorder.snapshot().state == 'ERROR'
    assert '1 GiB' in recorder.snapshot().message
    assert recorder._file_lock is None


def test_missing_ros2_is_reported_and_lock_released(tmp_path, monkeypatch):
    def fail(*args, **kwargs):
        raise FileNotFoundError('ros2 is unavailable')
    monkeypatch.setattr('robot_console.core.bag_recorder.subprocess.Popen', fail)
    recorder = BagRecorder(tmp_path)
    assert not recorder.start()
    assert recorder.snapshot().state == 'ERROR'
    assert recorder._file_lock is None


def test_real_bag_records_messages_and_finalizes_on_close(tmp_path, monkeypatch):
    rclpy = pytest.importorskip('rclpy')
    from rclpy.context import Context
    from std_msgs.msg import String
    from sensor_msgs.msg import Image, PointCloud2
    from rclpy.serialization import deserialize_message
    import sqlite3
    import socket
    assert {name for _, name in socket.if_nameindex()} == {'lo'}, 'Run this ROS integration test in a private network namespace'
    monkeypatch.setenv('ROS_DOMAIN_ID', '193')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    ctx = Context()
    rclpy.init(context=ctx, domain_id=193)
    node = rclpy.create_node('bag_ui_test_source', context=ctx)
    from rclpy.executors import SingleThreadedExecutor
    executor = SingleThreadedExecutor(context=ctx)
    executor.add_node(node)
    pub = node.create_publisher(String, '/bag_ui_test', 10)
    map_pub = node.create_publisher(PointCloud2, '/Laser_map', 1)
    viewer_pub = node.create_publisher(Image, '/sensor_viewer', 10)
    live_frames = []
    node.create_subscription(Image, '/sensor_viewer', lambda msg: live_frames.append(msg.header.stamp.sec), 10)
    viewer_emitted = {}
    viewer_sequence = 0
    next_viewer = 0.
    emitted = {}
    next_map = 0.
    sequence = 0
    recorder = BagRecorder(tmp_path)
    second = BagRecorder(tmp_path)
    try:
        assert recorder.start()
        assert not recorder.start()  # 同一UIの重複操作
        assert not second.start()    # 同じ保存先を使う別UI
        assert '別のUI' in second.snapshot().message
        deadline = time.monotonic()+23.
        # Stop images before recording ends to check that no stale frame repeats.
        viewer_deadline = deadline-3.
        received_subscription = None
        while time.monotonic() < deadline:
            pub.publish(String(data='bag UI integration test'))
            if time.monotonic() >= next_map:
                sequence += 1
                cloud = PointCloud2()
                cloud.header.frame_id = 'camera_init'
                cloud.header.stamp.sec = sequence
                cloud.height = 1
                cloud.width = sequence
                cloud.point_step = 4
                cloud.row_step = sequence*4
                cloud.data = bytes([sequence])*cloud.row_step
                emitted[sequence] = bytes(cloud.data)
                map_pub.publish(cloud)
                next_map = time.monotonic()+1.
            now = time.monotonic()
            if now < viewer_deadline and now >= next_viewer:
                viewer_sequence += 1
                img = Image()
                img.header.frame_id = 'laser_scan'
                img.header.stamp.sec = viewer_sequence
                img.height = img.width = 2
                img.encoding = 'bgr8'
                img.step = 6
                img.data = bytes([viewer_sequence % 256])*12
                viewer_emitted[viewer_sequence] = bytes(img.data)
                viewer_pub.publish(img)
                next_viewer = now+.025
            executor.spin_once(timeout_sec=.01)
            if pub.get_subscription_count():
                received_subscription = received_subscription or time.monotonic()
                # Keep publishing long enough to observe two 10-second snapshots.
        assert received_subscription is not None
        recorder.close()
        state = recorder.snapshot()
        assert state.state == 'SAVED', state
        from pathlib import Path
        output = Path(state.output)
        assert (output/'metadata.yaml').is_file()
        count = 0
        maps = []
        viewers = []
        for db in output.glob('*.db3'):
            with sqlite3.connect(db) as conn:
                count += conn.execute('SELECT COUNT(*) FROM messages m JOIN topics t ON m.topic_id=t.id WHERE t.name=?',
                                      ('/bag_ui_test',)).fetchone()[0]
                assert conn.execute(
                    "SELECT COUNT(*) FROM topics WHERE name='/Laser_map'").fetchone()[0] == 0
                maps.extend(conn.execute(
                    "SELECT m.timestamp,m.data FROM messages m JOIN topics t ON m.topic_id=t.id "
                    "WHERE t.name='/Laser_map_record' ORDER BY m.timestamp").fetchall())
                assert conn.execute("SELECT COUNT(*) FROM topics WHERE name='/sensor_viewer'").fetchone()[0] == 0
                viewers.extend(conn.execute(
                    "SELECT m.timestamp,m.data FROM messages m JOIN topics t ON m.topic_id=t.id "
                    "WHERE t.name='/sensor_viewer_record' ORDER BY m.timestamp").fetchall())
        assert count > 0
        assert len(maps) == 2
        assert 9. < (maps[1][0]-maps[0][0])/1e9 < 11.
        sequences = []
        for _, data in maps:
            cloud = deserialize_message(data, PointCloud2)
            seq = cloud.header.stamp.sec
            sequences.append(seq)
            assert cloud.header.frame_id == 'camera_init'
            assert bytes(cloud.data) == emitted[seq]
        assert sequences[1]-sequences[0] >= 8
        assert 17 <= len(viewers) <= 21
        assert len(live_frames) > 15*len(viewers)  # Live display still receives high-rate images.
        viewer_stamps = []
        for _, data in viewers:
            img = deserialize_message(data, Image)
            viewer_stamps.append(img.header.stamp.sec)
            assert img.header.frame_id == 'laser_scan'
            assert (img.height, img.width, img.encoding, img.step) == (2, 2, 'bgr8', 6)
            assert bytes(img.data) == viewer_emitted[img.header.stamp.sec]
        assert len(set(viewer_stamps)) == len(viewer_stamps)
        intervals = [(b[0]-a[0])/1e9 for a, b in zip(viewers, viewers[1:])]
        assert all(.75 < dt < 1.25 for dt in intervals)
        assert (maps[-1][0]-viewers[-1][0])/1e9 > -.5
    finally:
        recorder.close()
        executor.shutdown()
        node.destroy_node()
        ctx.shutdown()
