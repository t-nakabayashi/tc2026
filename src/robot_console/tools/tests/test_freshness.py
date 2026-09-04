"""FreshnessMonitor の単体テスト。"""

from datetime import datetime, timedelta, timezone

from robot_console.core.freshness import FreshnessLevel, FreshnessMonitor


def test_evaluate_unknown_before_first_receive():
    monitor = FreshnessMonitor()
    assert monitor.evaluate('topic') == FreshnessLevel.UNKNOWN
    assert monitor.elapsed_seconds('topic') is None


def test_evaluate_ok_stale_lost_boundaries():
    monitor = FreshnessMonitor(default_stale_sec=1.0, default_lost_sec=3.0)
    base = datetime(2026, 1, 1, tzinfo=timezone.utc)
    monitor.mark_received('topic', base)

    assert monitor.evaluate('topic', now=base + timedelta(seconds=1.0)) == FreshnessLevel.OK
    assert monitor.evaluate('topic', now=base + timedelta(seconds=1.5)) == FreshnessLevel.STALE
    assert monitor.evaluate('topic', now=base + timedelta(seconds=3.0)) == FreshnessLevel.STALE
    assert monitor.evaluate('topic', now=base + timedelta(seconds=3.1)) == FreshnessLevel.LOST


def test_per_key_threshold_overrides_default():
    monitor = FreshnessMonitor(default_stale_sec=1.0, default_lost_sec=3.0)
    monitor.set_threshold('gps.status', stale_sec=3.0, lost_sec=10.0)
    base = datetime(2026, 1, 1, tzinfo=timezone.utc)
    monitor.mark_received('gps.status', base)
    monitor.mark_received('other', base)

    now = base + timedelta(seconds=2.0)
    assert monitor.evaluate('gps.status', now=now) == FreshnessLevel.OK
    assert monitor.evaluate('other', now=now) == FreshnessLevel.STALE


def test_reset_returns_to_unknown():
    monitor = FreshnessMonitor()
    monitor.mark_received('topic')
    monitor.reset('topic')
    assert monitor.evaluate('topic') == FreshnessLevel.UNKNOWN


def test_from_config_parses_dotted_keys_matching_architecture_doc():
    config = {
        'freshness.default.stale_sec': 1.0,
        'freshness.default.lost_sec': 3.0,
        'freshness.gps.status.stale_sec': 3.0,
        'freshness.gps.status.lost_sec': 10.0,
    }
    monitor = FreshnessMonitor.from_config(config)
    base = datetime(2026, 1, 1, tzinfo=timezone.utc)
    monitor.mark_received('gps.status', base)
    monitor.mark_received('other', base)

    now = base + timedelta(seconds=2.0)
    assert monitor.evaluate('gps.status', now=now) == FreshnessLevel.OK
    assert monitor.evaluate('other', now=now) == FreshnessLevel.STALE
