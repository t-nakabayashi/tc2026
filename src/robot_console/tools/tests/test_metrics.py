"""metrics モジュールの単体テスト。"""

from datetime import datetime, timedelta, timezone

from robot_console.core.metrics import (
    TripMetrics,
    compute_progress_ratio,
    euclidean_distance,
    is_within_arrival_threshold,
)


def test_euclidean_distance():
    assert euclidean_distance((0.0, 0.0, 0.0), (3.0, 4.0, 0.0)) == 5.0


def test_compute_progress_ratio_boundaries():
    assert compute_progress_ratio(0, 0) == 0.0
    assert compute_progress_ratio(5, 10) == 0.5
    assert compute_progress_ratio(10, 10) == 1.0
    assert compute_progress_ratio(-1, 10) == 0.0
    assert compute_progress_ratio(11, 10) == 1.0


def test_is_within_arrival_threshold():
    assert is_within_arrival_threshold(0.5, 1.0) is True
    assert is_within_arrival_threshold(1.0, 1.0) is True
    assert is_within_arrival_threshold(1.1, 1.0) is False


def test_trip_metrics_accumulates_distance_and_time():
    metrics = TripMetrics()
    base = datetime(2026, 1, 1, tzinfo=timezone.utc)

    metrics.update_position((0.0, 0.0, 0.0), base)
    metrics.update_position((3.0, 4.0, 0.0), base + timedelta(seconds=1.0))
    metrics.update_position((3.0, 4.0, 3.0), base + timedelta(seconds=2.0))

    snapshot = metrics.snapshot(now=base + timedelta(seconds=2.0))
    assert snapshot.traveled_distance_m == 8.0
    assert snapshot.elapsed_sec == 2.0


def test_trip_metrics_first_update_does_not_add_distance():
    metrics = TripMetrics()
    metrics.update_position((10.0, 10.0, 0.0))
    snapshot = metrics.snapshot()
    assert snapshot.traveled_distance_m == 0.0


def test_trip_metrics_reset_clears_state():
    metrics = TripMetrics()
    metrics.update_position((0.0, 0.0, 0.0))
    metrics.update_position((1.0, 0.0, 0.0))
    metrics.reset()

    snapshot = metrics.snapshot()
    assert snapshot.traveled_distance_m == 0.0
    assert snapshot.elapsed_sec == 0.0
