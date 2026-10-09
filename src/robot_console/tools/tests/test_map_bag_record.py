from types import SimpleNamespace
import re
from robot_console.map_bag_record import MapRecordRelay, record_command


def test_latest_snapshot_and_no_stale_repeats():
    sent = []
    relay = SimpleNamespace(_latest=None, _publisher=SimpleNamespace(publish=sent.append))
    first, latest = object(), object()
    MapRecordRelay._publish_latest(relay)
    MapRecordRelay._receive(relay, first)
    MapRecordRelay._receive(relay, latest)
    MapRecordRelay._publish_latest(relay)
    MapRecordRelay._publish_latest(relay)
    assert sent == [latest]
    assert relay._latest is None


def test_exclusion_only_matches_original_map_and_viewer():
    cmd = record_command('/tmp/bag with spaces', 1024)
    pattern = cmd[cmd.index('--exclude-regex')+1]
    assert re.search(pattern, '/Laser_map')
    assert not re.search(pattern, '/Laser_map_record')
    assert not re.search(pattern, '/Laser_map_other')
    assert re.search(pattern, '/sensor_viewer')
    assert not re.search(pattern, '/sensor_viewer_record')
    assert not re.search(pattern, '/sensor_viewer_other')
    assert not re.search(pattern, '/usb_cam/image_raw')
    assert '--all-topics' in cmd
    assert cmd[cmd.index('--output')+1] == '/tmp/bag with spaces'
