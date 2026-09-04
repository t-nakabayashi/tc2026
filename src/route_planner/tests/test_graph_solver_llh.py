"""graph_solver の LLH waypoint CSV 距離計算に関する単体テスト."""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from route_planner.graph_solver import _load_waypoint_csv_length  # noqa: E402


def test_load_waypoint_csv_length_accepts_latitude_longitude_headers(tmp_path: Path) -> None:
    """latitude/longitudeヘッダのWaypoint CSVを地理座標として扱う."""

    csv_path = tmp_path / "segment.csv"
    csv_path.write_text(
        "label,latitude,longitude,heading_deg\n"
        "A,36.000000,140.000000,0.0\n"
        "B,36.000010,140.000000,0.0\n",
        encoding="utf-8",
    )

    length_m, points = _load_waypoint_csv_length(str(csv_path))

    assert length_m > 1.0
    assert points == [(140.0, 36.0), (140.0, 36.00001)]
