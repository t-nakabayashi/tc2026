#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""記録済み rosbag で、水平スライス層と従来の回避ヒントを突き合わせて集計する.

`replay_bag_layers.py` が 1 場面ずつ画像と文章を書き出すのに対し、本ツールは多数の場面を
まとめて数える。確認したいのは次の 3 点である。

1. 層ごとに点が得られる割合（前下がり取付で近傍の層を観測できているか）
2. 層から求めた横移動余裕と、obstacle_monitor が記録時に出していたオフセットの差
3. 従来のオフセットが実測の空き幅を超えていた割合（回避幅の上限を是正した効果）

bag の配置はリポジトリ外であり、実行時に引数で与える。既定値は持たない。

使い方:
    python3 tools/analyze_bag_layers.py <bag_dir> [--near-threshold 2.0] [--csv out.csv]
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, List, Optional, Tuple
import argparse
import math
import statistics
import sys

import numpy as np

import rosbag2_py
from rclpy.duration import Duration
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from rosidl_runtime_py.utilities import get_message
from tf2_ros import Buffer

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lidar_layer_slicer.layer_core import (SOURCE_POINT_CLOUD, SOURCE_SCAN,
                                           ClearanceConfig, LayerObservation,
                                           LayerSpec, combined_available_offsets,
                                           nearest_forward_distance, observe_layer,
                                           scan_to_xy, slice_layer_points)
from replay_bag_layers import (BASE_FRAME, CLOUD_FRAME, FLOOR_TO_BASE_LINK_M,
                               SCAN_LAYER_HEIGHT_M, TOPIC_HINT, TOPIC_LIVOX,
                               TOPIC_SCAN, collect_static_transforms,
                               lookup_rigid_transform, open_reader)


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('bag', type=Path, help='rosbag のディレクトリ')
    parser.add_argument('--near-threshold', type=float, default=2.0,
                        help='場面とみなす直進帯の最近傍距離 [m]')
    parser.add_argument('--min-interval-sec', type=float, default=2.0,
                        help='場面を採る最小間隔 [s]')
    parser.add_argument('--limit', type=int, default=200, help='集計する場面の最大数')
    parser.add_argument('--accumulate-sec', type=float, default=0.3,
                        help='点群を累積する時間幅 [s]')
    parser.add_argument('--layer-heights', type=float, nargs='*',
                        default=[0.35, 0.50], help='点群層の床上高さ [m]')
    parser.add_argument('--layer-thickness', type=float, default=0.10, help='層の厚さ [m]')
    parser.add_argument('--robot-width', type=float, default=0.6, help='車体全幅 [m]')
    parser.add_argument('--side-margin', type=float, default=0.1, help='左右余裕 [m]')
    parser.add_argument('--forward-range', type=float, default=2.0,
                        help='横移動余裕を評価する前方距離 [m]')
    parser.add_argument('--min-offset', type=float, default=0.35,
                        help='回避の最小オフセット [m]。これ未満の余裕は回避不可とみなす')
    parser.add_argument('--csv', type=Path, default=None, help='場面ごとの値を書き出す CSV')
    return parser.parse_args(argv)


def collect_scene_times(bag: Path, types: Dict[str, str],
                        config: ClearanceConfig, args: argparse.Namespace
                        ) -> List[Tuple[int, object, object]]:
    """場面の時刻と、その時点の scan・hint を集める（点群は読まない）.

    点群の読み出しは重いので、先に場面を決めてから必要なフレームだけを復号する。
    """
    reader, _ = open_reader(bag)
    topics = [t for t in (TOPIC_SCAN, TOPIC_HINT) if t in types]
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    message_types = {t: get_message(types[t]) for t in topics}
    interval_ns = int(args.min_interval_sec * 1e9)
    last_ns = -(10 ** 18)
    scenes: List[Tuple[int, object, object]] = []
    hint = None
    while reader.has_next() and len(scenes) < args.limit:
        topic, data, stamp_ns = reader.read_next()
        if topic == TOPIC_HINT:
            hint = deserialize_message(data, message_types[topic])
            continue
        scan = deserialize_message(data, message_types[topic])
        if stamp_ns - last_ns < interval_ns:
            continue
        points = scan_to_xy(scan.ranges, float(scan.angle_min), float(scan.angle_increment),
                            float(scan.range_min), float(scan.range_max))
        if not (nearest_forward_distance(points, config) <= args.near_threshold):
            continue
        scenes.append((stamp_ns, scan, hint))
        last_ns = stamp_ns
    return scenes


def collect_clouds(bag: Path, types: Dict[str, str], scene_times: List[int],
                   transform, accumulate_ns: int) -> Dict[int, np.ndarray]:
    """場面ごとに、その直前 `accumulate_ns` の点群を base_link 系で集める.

    bag の記録時刻だけで必要なフレームを選び、復号は選んだフレームにのみ行う。
    """
    if transform is None or TOPIC_LIVOX not in types:
        return {}
    rotation, translation = transform
    reader, _ = open_reader(bag)
    reader.set_filter(rosbag2_py.StorageFilter(topics=[TOPIC_LIVOX]))
    message_type = get_message(types[TOPIC_LIVOX])
    wanted = sorted(scene_times)
    clouds: Dict[int, List[np.ndarray]] = {t: [] for t in wanted}
    index = 0
    while reader.has_next() and index < len(wanted):
        _, data, stamp_ns = reader.read_next()
        while index < len(wanted) and stamp_ns > wanted[index]:
            index += 1
        if index >= len(wanted):
            break
        if not (wanted[index] - accumulate_ns <= stamp_ns <= wanted[index]):
            continue
        message = deserialize_message(data, message_type)
        if not message.points:
            continue
        points = np.asarray([(p.x, p.y, p.z) for p in message.points], dtype=np.float32)
        clouds[wanted[index]].append(points @ rotation.T + translation)
    return {t: (np.concatenate(v, axis=0) if v else np.empty((0, 3), dtype=np.float32))
            for t, v in clouds.items()}


def observe(scan, cloud: np.ndarray, specs: List[LayerSpec],
            config: ClearanceConfig) -> List[LayerObservation]:
    """1 場面の各層を観測する."""
    observations = []
    for spec in specs:
        if spec.source == SOURCE_SCAN:
            points = scan_to_xy(scan.ranges, float(scan.angle_min),
                                float(scan.angle_increment), float(scan.range_min),
                                float(scan.range_max))
        else:
            points = slice_layer_points(cloud, spec, FLOOR_TO_BASE_LINK_M)
        observations.append(observe_layer(points, spec, config))
    return observations


def _ratio(count: int, total: int) -> str:
    return f'{count}/{total} ({100.0 * count / total:.0f}%)' if total else '0/0'


def _median(values: List[float]) -> str:
    finite = [v for v in values if math.isfinite(v)]
    return f'{statistics.median(finite):.2f}' if finite else '-'


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    if not args.bag.exists():
        print(f'bag が見つからない: {args.bag}')
        return 1
    config = ClearanceConfig(forward_range_m=args.forward_range,
                             robot_width_m=args.robot_width,
                             side_margin_m=args.side_margin)
    specs = [LayerSpec('urg', SOURCE_SCAN, SCAN_LAYER_HEIGHT_M, 0.0)]
    specs += [LayerSpec(f'h{int(round(h * 100)):03d}', SOURCE_POINT_CLOUD, float(h),
                        float(args.layer_thickness)) for h in args.layer_heights]
    specs.sort(key=lambda spec: spec.height_m)

    _, types = open_reader(args.bag)
    scenes = collect_scene_times(args.bag, types, config, args)
    if not scenes:
        print('条件に合う場面が無い。')
        return 0
    buffer = collect_static_transforms(args.bag, types)
    transform = lookup_rigid_transform(buffer, BASE_FRAME, CLOUD_FRAME)
    clouds = collect_clouds(args.bag, types, [s[0] for s in scenes], transform,
                            int(args.accumulate_sec * 1e9))

    rows = []
    for stamp_ns, scan, hint in scenes:
        observations = observe(scan, clouds.get(stamp_ns, np.empty((0, 3), dtype=np.float32)),
                               specs, config)
        left, right = combined_available_offsets(observations)
        row = {'stamp_ns': stamp_ns,
               'combined_left_m': left, 'combined_right_m': right,
               'hint_front_blocked': bool(hint.front_blocked) if hint else False,
               'hint_left_m': float(hint.left_offset_m) if hint else float('nan'),
               'hint_right_m': float(hint.right_offset_m) if hint else float('nan')}
        for observation in observations:
            name = observation.spec.name
            row[f'{name}_points'] = observation.point_count
            row[f'{name}_nearest_m'] = observation.nearest_x_m
            row[f'{name}_req_left_m'] = observation.required_left_offset_m
            row[f'{name}_avail_left_m'] = observation.available_left_offset_m
            row[f'{name}_req_right_m'] = observation.required_right_offset_m
            row[f'{name}_avail_right_m'] = observation.available_right_offset_m
        rows.append(row)

    print(f'=== {args.bag.name}')
    print(f'場面数 {len(rows)}（直進帯に {args.near_threshold:.1f} m 以内の点がある時刻を '
          f'{args.min_interval_sec:.0f} 秒間隔で抽出）')
    for spec in specs:
        observed = sum(1 for r in rows if r[f'{spec.name}_points'] > 0)
        counts = [r[f'{spec.name}_points'] for r in rows]
        print(f'  層 {spec.name} (床上 {spec.height_m:.2f} m): 点あり {_ratio(observed, len(rows))}'
              f' 点数中央値 {int(statistics.median(counts))}'
              f' 最近傍中央値 {_median([r[f"{spec.name}_nearest_m"] for r in rows])} m')

    blocked = sum(1 for r in rows if r['hint_front_blocked'])
    print(f'  記録時の front_blocked: {_ratio(blocked, len(rows))}')
    print(f'  実測の使用可能幅 中央値: 左 {_median([r["combined_left_m"] for r in rows])} m / '
          f'右 {_median([r["combined_right_m"] for r in rows])} m')
    print(f'  記録時の hint オフセット 中央値: 左 {_median([r["hint_left_m"] for r in rows])} m / '
          f'右 {_median([r["hint_right_m"] for r in rows])} m')

    exceed = sum(1 for r in rows
                 if (r['hint_left_m'] > r['combined_left_m'] + 1e-6
                     or r['hint_right_m'] > r['combined_right_m'] + 1e-6))
    print(f'  hint が実測の使用可能幅を超えた場面: {_ratio(exceed, len(rows))}')

    narrow = sum(1 for r in rows
                 if max(r['combined_left_m'], r['combined_right_m']) < args.min_offset)
    print(f'  実測の使用可能幅が最小オフセット {args.min_offset:.2f} m 未満: '
          f'{_ratio(narrow, len(rows))}')

    if args.csv:
        import csv
        with args.csv.open('w', newline='', encoding='utf-8') as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        print(f'  場面ごとの値を {args.csv} へ書き出した。')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
