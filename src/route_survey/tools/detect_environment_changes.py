#!/usr/bin/env python3
"""断面の観測値から、走行環境が切り替わる位置の候補を挙げる.

走行環境は数十mから百m超にわたり同じ性質が続く。境界までの距離と検出率が前後で
大きく変わる位置を候補とし、利用者が映像で確認して区間を確定する。
候補を挙げるだけで、区間を確定する機能はない。曲率が大きい点は左右の意味が崩れる
ため除く。右左折そのものは環境の切り替わりと一致することが多いので、別の候補として
曲率から挙げる。
"""
import argparse
import csv
from pathlib import Path

import numpy as np


def window_median(values: np.ndarray, distance: np.ndarray, center: float,
                  span: float, side: int) -> tuple[float, float]:
    """指定位置の前または後の区間における中央値と検出率を求める.

    Args:
        values (np.ndarray): 対象の値。未検出は NaN.
        distance (np.ndarray): 各点の弧長 [m].
        center (float): 評価位置 [m].
        span (float): 窓の長さ [m].
        side (int): -1 で手前、1 で先.

    Returns:
        tuple[float, float]: 中央値と検出率. 点が無ければ NaN と 0.
    """
    low, high = (center-span, center) if side < 0 else (center, center+span)
    inside = (distance >= low)&(distance < high)
    if not inside.any():
        return float('nan'), 0.
    picked = values[inside]
    found = picked[~np.isnan(picked)]
    rate = len(found)/len(picked)
    return (float(np.median(found)) if len(found) else float('nan')), rate


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cross-sections', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--span', type=float, default=10., help='前後を比べる窓の長さ [m]')
    parser.add_argument('--step', type=float, default=1., help='評価位置の間隔 [m]')
    parser.add_argument('--min-separation', type=float, default=15.,
                        help='候補どうしを離す最小距離 [m]')
    parser.add_argument('--distance-change', type=float, default=.6,
                        help='境界距離がこれ以上変われば切り替わりとみなす [m]')
    parser.add_argument('--rate-change', type=float, default=.35,
                        help='検出率がこれ以上変われば切り替わりとみなす')
    parser.add_argument('--max-curvature', type=float, default=2.,
                        help='これを超える曲率の断面を除く [deg/m]')
    parser.add_argument('--turn-curvature', type=float, default=4.,
                        help='これを超える曲率は右左折として候補に挙げる [deg/m]')
    args = parser.parse_args()

    rows = list(csv.DictReader(args.cross_sections.open()))
    distance = np.array([float(r['distance_m']) for r in rows])
    curvature = np.array([float(r['curvature_deg_per_m']) for r in rows])
    columns = {name: np.array([float(r[name]) if r[name] else np.nan for r in rows])
               for name in ('urg_left_m', 'urg_right_m')}
    usable = curvature <= args.max_curvature
    scores, details = [], []
    for center in np.arange(distance[0]+args.span, distance[-1]-args.span, args.step):
        score, reason = 0., []
        for name, values in columns.items():
            before, rate_before = window_median(values[usable], distance[usable], center,
                                                args.span, -1)
            after, rate_after = window_median(values[usable], distance[usable], center,
                                              args.span, 1)
            side = '左' if 'left' in name else '右'
            if not (np.isnan(before) or np.isnan(after)):
                change = abs(after-before)
                if change >= args.distance_change:
                    score += change/args.distance_change
                    reason.append(f'{side}の境界距離 {before:.1f}→{after:.1f} m')
            rate_change = abs(rate_after-rate_before)
            if rate_change >= args.rate_change:
                score += rate_change/args.rate_change
                reason.append(f'{side}の検出率 {rate_before:.0%}→{rate_after:.0%}')
        scores.append(score)
        details.append((center, reason))
    scores = np.array(scores)
    # 近接する候補は最も強いものだけ残す
    picked, blocked = [], np.zeros(len(scores), bool)
    for index in np.argsort(-scores):
        if scores[index] <= 0 or blocked[index]:
            continue
        center = details[index][0]
        picked.append((center, scores[index], details[index][1], '観測の変化'))
        blocked |= np.abs(np.array([d[0] for d in details])-center) < args.min_separation
    # 右左折も環境の切り替わりの候補とする
    turning = curvature > args.turn_curvature
    if turning.any():
        edges = np.flatnonzero(np.diff(turning.astype(int)) != 0)
        for edge in edges:
            center = float(distance[edge])
            if all(abs(center-c) >= args.min_separation for c, *_ in picked):
                picked.append((center, 0., [f'曲率 {curvature[edge]:.1f} deg/m'], '右左折'))
    picked.sort(key=lambda item: item[0])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('w', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(['distance_m', 'score', 'kind', 'reason'])
        for center, score, reason, kind in picked:
            writer.writerow([f'{center:.0f}', f'{score:.1f}', kind, ' / '.join(reason)])
            print(f'{center:7.0f} m  {score:5.1f}  {kind}  {" / ".join(reason)}', flush=True)
    print(f'\nDONE 候補 {len(picked)} 件 -> {args.output}', flush=True)


if __name__ == '__main__':
    main()
