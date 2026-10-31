#!/usr/bin/env python3
"""区間ごとの横移動量を、既存の経路CSVの回避幅2列へ展開する.

経路の座標・方位・停止属性・ラベルは変更しない。書き換えるのは `right_is_open` と
`left_is_open` の2列だけである。入力のCSVは上書きせず、別の場所へ書き出す。

区間の境界では値が不連続に変わる。`--blend` を指定すると、境界の前後でその距離だけ
かけて線形に遷移させ、追従中に上限が急変しないようにする。遷移は小さい方の値へ
先に寄せるため、境界をまたぐ区間で上限が過大になることはない。
"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np


def load_regions(path: Path) -> list[dict]:
    """区間ごとの左右の値を読み込む.

    Args:
        path (Path): estimate_region_widths.py の出力CSV.

    Returns:
        list[dict]: 開始・終了・左右の値を持つ区間の一覧（開始位置の昇順）.
    """
    merged: dict[tuple, dict] = {}
    for row in csv.DictReader(path.open()):
        key = (float(row['start_m']), float(row['end_m']))
        entry = merged.setdefault(key, dict(start=key[0], end=key[1], name=row['name']))
        entry[row['side']] = float(row['offset_m'])
    regions = sorted(merged.values(), key=lambda r: r['start'])
    for region in regions:
        if 'left' not in region or 'right' not in region:
            raise ValueError(f'区間 {region["start"]}-{region["end"]} に左右の値が揃っていない')
    return regions


def profile(regions: list[dict], distance: np.ndarray, blend: float) -> tuple:
    """各 waypoint の弧長に対する左右の値を求める.

    Args:
        regions (list[dict]): 区間の一覧.
        distance (np.ndarray): waypoint の弧長 [m].
        blend (float): 区間の境界でかけて遷移させる距離 [m]. 0 で遷移しない.

    Returns:
        tuple: 左の値と右の値の配列.
    """
    left = np.zeros(len(distance))
    right = np.zeros(len(distance))
    for region in regions:
        inside = (distance >= region['start'])&(distance < region['end'])
        left[inside] = region['left']
        right[inside] = region['right']
    last = regions[-1]
    tail = distance >= last['end']
    left[tail], right[tail] = last['left'], last['right']
    if blend <= 0:
        return left, right
    for values in (left, right):
        for region in regions[1:]:
            edge = region['start']
            near = np.abs(distance-edge) <= blend
            if not near.any():
                continue
            before = values[(distance < edge)&near]
            after = values[(distance >= edge)&near]
            if not len(before) or not len(after):
                continue
            low, high = before[-1], after[0]
            ratio = np.clip((distance[near]-(edge-blend))/(2*blend), 0., 1.)
            blended = low+(high-low)*ratio
            # 遷移中に上限が過大にならないよう、両端の小さい方を超えさせない
            values[near] = np.minimum(blended, max(low, high))
            values[near] = np.minimum(values[near], np.maximum(low, high))
    return left, right


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--waypoints-csv', type=Path, required=True, help='既存の経路CSV')
    parser.add_argument('--waypoints-enu', type=Path, required=True,
                        help='弧長を持つ waypoints_enu.json')
    parser.add_argument('--region-widths', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True, help='書き出し先のCSV')
    parser.add_argument('--blend', type=float, default=3., help='区間境界の遷移距離 [m]')
    args = parser.parse_args()

    if args.output.resolve() == args.waypoints_csv.resolve():
        raise ValueError('入力のCSVを上書きできない')
    regions = load_regions(args.region_widths)
    enu = json.loads(args.waypoints_enu.read_text())
    by_label = {str(row['label']): float(row['distance_from_start_m']) for row in enu}
    rows = list(csv.DictReader(args.waypoints_csv.open()))
    fields = list(rows[0].keys())
    for name in ('right_is_open', 'left_is_open'):
        if name not in fields:
            raise ValueError(f'{name} 列が無い')
    missing = [r['label'] for r in rows if r['label'] not in by_label]
    if missing:
        raise ValueError(f'弧長が引けない waypoint がある: {missing[:5]}')
    distance = np.array([by_label[r['label']] for r in rows])
    left, right = profile(regions, distance, args.blend)
    changed = 0
    for row, left_value, right_value in zip(rows, left, right):
        if (row['left_is_open'], row['right_is_open']) != (f'{left_value:.1f}',
                                                           f'{right_value:.1f}'):
            changed += 1
        row['left_is_open'] = f'{left_value:.1f}'
        row['right_is_open'] = f'{right_value:.1f}'
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    print(f'{len(rows)} 点中 {changed} 点の幅を更新した', flush=True)
    print(f'左: 0 の点 {int((left == 0).sum())}, 中央 {np.median(left):.1f} m, '
          f'最大 {left.max():.1f} m', flush=True)
    print(f'右: 0 の点 {int((right == 0).sum())}, 中央 {np.median(right):.1f} m, '
          f'最大 {right.max():.1f} m', flush=True)
    print(f'DONE -> {args.output}', flush=True)


if __name__ == '__main__':
    main()
