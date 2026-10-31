"""選択課題Aの通路網から、route_plannerの可変ブロック一式を生成する.

入力は通路網の定義YAML（ノードの緯度経度、エッジ、回避幅、通過点）である。出力は
`nodes.csv`、`edges.csv`、エッジごとのwaypoint CSV、および課題A対応の `route_config`
である。エッジは直線とし、指定間隔以下へ等分して補間する。進行方位は始点から終点への
方位を用いる。

経路CSVそのものは複製しない。課題A対応のルートは、課題A非対応のルートと同じ
`fixed/part1.csv`・`fixed/part2.csv` を参照し、その間に可変ブロックを挟む。
"""
import argparse
import csv
import math
from pathlib import Path
from typing import Dict, List, Tuple

import yaml

WAYPOINT_FIELDS = [
    "label", "latitude", "longitude", "altitude", "heading_deg",
    "right_is_open", "left_is_open", "line_is_stop", "signal_is_stop", "isnot_skipnum",
]
MEAN_EARTH_RADIUS_M = 6371000.0


def local_offsets(origin: Tuple[float, float], point: Tuple[float, float]) -> Tuple[float, float]:
    """原点から見た東・北方向の距離を求める.

    Args:
        origin (Tuple[float, float]): 原点の (緯度, 経度) [deg].
        point (Tuple[float, float]): 対象点の (緯度, 経度) [deg].

    Returns:
        Tuple[float, float]: (東[m], 北[m]).
    """

    east = math.radians(point[1] - origin[1]) * MEAN_EARTH_RADIUS_M * math.cos(
        math.radians(origin[0])
    )
    north = math.radians(point[0] - origin[0]) * MEAN_EARTH_RADIUS_M
    return east, north


def edge_waypoints(
    source: str,
    target: str,
    a: Tuple[float, float],
    b: Tuple[float, float],
    spacing_m: float,
    right_open: float,
    left_open: float,
) -> List[dict]:
    """2ノードを結ぶ直線のwaypoint列を作る.

    Args:
        source (str): 始点ノードID.
        target (str): 終点ノードID.
        a (Tuple[float, float]): 始点の (緯度, 経度) [deg].
        b (Tuple[float, float]): 終点の (緯度, 経度) [deg].
        spacing_m (float): waypoint間隔の上限 [m].
        right_open (float): 右の回避幅 [m].
        left_open (float): 左の回避幅 [m].

    Returns:
        List[dict]: waypoint CSVの行.
    """

    east, north = local_offsets(a, b)
    length = math.hypot(east, north)
    if length <= 0.0:
        raise ValueError(f"エッジ {source}-{target} の長さが0です。")
    heading = math.degrees(math.atan2(east, north)) % 360.0
    count = max(1, math.ceil(length / spacing_m))
    rows = []
    for i in range(count + 1):
        ratio = i / count
        label = source if i == 0 else target if i == count else f"{source}-{target}_{i}"
        rows.append({
            "label": label,
            "latitude": f"{a[0] * (1 - ratio) + b[0] * ratio:.11f}",
            "longitude": f"{a[1] * (1 - ratio) + b[1] * ratio:.11f}",
            "altitude": "",
            "heading_deg": f"{heading:.8f}",
            "right_is_open": right_open,
            "left_is_open": left_open,
            "line_is_stop": 0,
            "signal_is_stop": 0,
            "isnot_skipnum": 1,
        })
    return rows


def build(spec: dict, output_dir: Path, fixed_blocks: List[str], config_name: str) -> Dict[str, int]:
    """定義YAMLから可変ブロック一式を書き出す.

    Args:
        spec (dict): 通路網の定義.
        output_dir (Path): ルートディレクトリ（`variable/` を作る親）.
        fixed_blocks (List[str]): 可変ブロックの前後に置く固定ブロックのCSVパス2つ.
        config_name (str): 書き出す route_config のファイル名.

    Returns:
        Dict[str, int]: 生成結果の要約.
    """

    nodes = {nid: (float(v["latitude"]), float(v["longitude"])) for nid, v in spec["nodes"].items()}
    spacing = float(spec["spacing_m"])
    right_open = float(spec["right_is_open"])
    left_open = float(spec["left_is_open"])

    variable_dir = output_dir / "variable"
    segments_dir = variable_dir / "segments"
    segments_dir.mkdir(parents=True, exist_ok=True)

    edge_rows = []
    total_points = 0
    total_length = 0.0
    for edge in spec["edges"]:
        source, target = edge["source"], edge["target"]
        for nid in (source, target):
            if nid not in nodes:
                raise ValueError(f"ノード '{nid}' が定義されていません。")
        rows = edge_waypoints(
            source, target, nodes[source], nodes[target], spacing, right_open, left_open
        )
        name = f"edge_{source}_{target}.csv"
        with (segments_dir / name).open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, WAYPOINT_FIELDS)
            writer.writeheader()
            writer.writerows(rows)
        east, north = local_offsets(nodes[source], nodes[target])
        total_length += math.hypot(east, north)
        total_points += len(rows)
        edge_rows.append({
            "source": source,
            "target": target,
            "waypoint_list": f"variable/segments/{name}",
            "reversible": 1,
            "weight_factor": 1,
        })

    with (variable_dir / "nodes.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, ["id", "lat", "lon"])
        writer.writeheader()
        for nid, (lat, lon) in nodes.items():
            writer.writerow({"id": nid, "lat": f"{lat:.7f}", "lon": f"{lon:.7f}"})

    with (variable_dir / "edges.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream, ["source", "target", "waypoint_list", "reversible", "weight_factor"]
        )
        writer.writeheader()
        writer.writerows(edge_rows)

    blocks = [
        {"name": "tsukuba2026_part1", "type": "fixed", "segment_id": fixed_blocks[0]},
        {"name": "task_a", "type": "variable",
         "nodes_file": "variable/nodes.csv", "edges_file": "variable/edges.csv",
         "start": spec["start"], "goal": spec["goal"], "checkpoints": list(spec["checkpoints"])},
        {"name": "tsukuba2026_part2", "type": "fixed", "segment_id": fixed_blocks[1]},
    ]
    header = (
        "# つくば2026 本走行コース（選択課題A対応）。\n"
        "# 固定ブロックは課題A非対応のルートと同じ2ファイルを参照する。CSVは複製しない。\n"
        "# 可変ブロックは src/route_survey/tools/build_task_a_route.py が生成する。\n"
    )
    (output_dir / config_name).write_text(
        header + yaml.safe_dump({"blocks": blocks}, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    return {"nodes": len(nodes), "edges": len(edge_rows),
            "waypoints": total_points, "length_m": round(total_length, 1)}


def main() -> None:
    """コマンドラインから可変ブロック一式を生成する."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--spec", type=Path, required=True, help="通路網の定義YAML")
    parser.add_argument("--output", type=Path, required=True, help="ルートディレクトリ")
    parser.add_argument("--fixed", nargs=2, default=["fixed/part1.csv", "fixed/part2.csv"],
                        help="前後に置く固定ブロックのCSVパス")
    parser.add_argument("--config-name", default="route_config_task_a.yaml",
                        help="書き出す route_config のファイル名")
    args = parser.parse_args()

    spec = yaml.safe_load(args.spec.read_text(encoding="utf-8"))
    summary = build(spec, args.output, args.fixed, args.config_name)
    print(f"ノード{summary['nodes']} エッジ{summary['edges']} "
          f"waypoint{summary['waypoints']} 通路総延長{summary['length_m']} m")


if __name__ == "__main__":
    main()
