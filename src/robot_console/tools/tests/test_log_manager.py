"""LogManager の単体テスト。"""

from robot_console.core.log_manager import LogManager, count_levels, detect_log_level, filter_levels


def test_detect_log_level_variants():
    assert detect_log_level('[INFO] [1234.5] [node]: message') == 'INFO'
    assert detect_log_level('[WARN] [node]: 遅延を検知') == 'WARN'
    assert detect_log_level('[ERROR] failure') == 'ERROR'
    assert detect_log_level('[ERR] traceback line') == 'ERROR'
    assert detect_log_level('plain text with no level') is None


def test_count_levels_works_without_log_manager_instance():
    counts = count_levels(['[INFO] a', '[WARN] b', '[WARN] c', '[ERROR] d'])
    assert counts.info == 1
    assert counts.warn == 2
    assert counts.error == 1
    assert counts.debug == 0


def test_filter_levels_keeps_only_requested_levels():
    lines = ['[INFO] a', '[WARN] b', '[ERROR] c', '[DEBUG] d']
    assert filter_levels(lines, ('WARN', 'ERROR')) == ['[WARN] b', '[ERROR] c']


def test_append_and_snapshot_are_isolated_per_profile():
    manager = LogManager()
    manager.append('route_manager', '[INFO] start\n')
    manager.append('route_follower', '[WARN] retry\n')

    assert manager.snapshot('route_manager') == ['[INFO] start\n']
    assert manager.snapshot('route_follower') == ['[WARN] retry\n']
    assert manager.snapshot('unknown') == []


def test_snapshot_all_covers_registered_profiles():
    manager = LogManager(['a', 'b'])
    manager.append('a', '[INFO] hello\n')

    snapshot = manager.snapshot_all()
    assert set(snapshot.keys()) == {'a', 'b'}
    assert snapshot['a'] == ['[INFO] hello\n']
    assert snapshot['b'] == []


def test_merged_snapshot_prefixes_profile_id():
    manager = LogManager()
    manager.append('route_manager', '[INFO] a')
    manager.append('route_follower', '[WARN] b')

    merged = manager.merged_snapshot(['route_manager', 'route_follower'])
    assert merged == ['[route_manager] [INFO] a', '[route_follower] [WARN] b']


def test_level_counts_and_warn_error_lines():
    manager = LogManager()
    for line in ['[INFO] a', '[WARN] b', '[ERROR] c', '[DEBUG] d', '[ERR] e']:
        manager.append('node', line)

    counts = manager.level_counts('node')
    assert counts.info == 1
    assert counts.warn == 1
    assert counts.error == 2  # [ERROR] と [ERR] の合計
    assert counts.debug == 1
    assert counts.fatal == 0

    warn_error = manager.warn_error_lines('node')
    assert warn_error == ['[WARN] b', '[ERROR] c', '[ERR] e']


def test_clear_removes_buffered_lines():
    manager = LogManager()
    manager.append('node', '[INFO] a')
    manager.clear('node')
    assert manager.snapshot('node') == []


def test_buffer_capacity_drops_oldest_lines():
    manager = LogManager(buffer_capacity=2)
    manager.append('node', 'line1')
    manager.append('node', 'line2')
    manager.append('node', 'line3')

    assert manager.snapshot('node') == ['line2', 'line3']
