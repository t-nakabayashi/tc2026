"""経路CSVの分割ツールの単体テスト."""

from __future__ import annotations

import csv
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from split_route_csv import split_rows  # noqa: E402

FIELDS = [
    "label", "latitude", "longitude", "altitude", "heading_deg",
    "right_is_open", "left_is_open", "line_is_stop", "signal_is_stop", "isnot_skipnum",
]


def _row(label: str, line_stop: str = "0") -> dict:
    values = dict.fromkeys(FIELDS, "0")
    values["label"] = label
    values["line_is_stop"] = line_stop
    values["signal_is_stop"] = "0"
    return values


def test_split_rows_keeps_split_point_in_head() -> None:
    """分割点の waypoint は前半の末尾に入り、行の総数は変わらない."""

    rows = [_row(f"WP{i:04d}") for i in range(5)]

    head, tail = split_rows(rows, "WP0002")

    assert [r["label"] for r in head] == ["WP0000", "WP0001", "WP0002"]
    assert [r["label"] for r in tail] == ["WP0003", "WP0004"]
    assert len(head) + len(tail) == len(rows)


def test_split_rows_rejects_stop_waypoint() -> None:
    """停止点は分割点にできない."""

    rows = [_row("WP0000"), _row("E01", line_stop="1"), _row("WP0002")]

    with pytest.raises(ValueError, match="停止点"):
        split_rows(rows, "E01")


@pytest.mark.parametrize("label", ["WP0000", "WP0002", "WP9999"])
def test_split_rows_rejects_endpoints_and_missing_label(label: str) -> None:
    """経路の端や存在しないラベルは分割点にできない."""

    rows = [_row("WP0000"), _row("WP0001"), _row("WP0002")]

    with pytest.raises(ValueError):
        split_rows(rows, label)
