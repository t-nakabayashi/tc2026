#!/usr/bin/env python3
"""区間ごとに、左右それぞれの境界までの距離のばらつきを集計する.

同じ環境が続く区間で境界までの距離のばらつきが小さければ、その側には連続した
境界物があり、区間の代表値を取れる。ばらつきが大きい側は、境界が探索範囲の外に
あるか、出入口などで不連続であることを意味する。検出率を併記して両者を区別する。

左右の和（道幅）は、片側に境界が無い区間では意味を持たない。側ごとに判定する。
"""
import argparse
import csv
from pathlib import Path

import numpy as np

SOURCES = (('路面場', 'left_m', 'right_m'), ('URG', 'urg_left_m', 'urg_right_m'))
SIDES = (('左', 0), ('右', 1))


def load_sections(path: Path, max_curvature: float) -> list[dict]:
    """断面測定のCSVを読み、曲率が大きい点を除く.

    Args:
        path (Path): measure_cross_sections.py の出力.
        max_curvature (float): これを超える曲率の点を除く [deg/m].

    Returns:
        list[dict]: 断面の一覧.
    """
    rows = []
    for row in csv.DictReader(path.open()):
        if float(row['curvature_deg_per_m']) > max_curvature:
            continue
        rows.append(row)
    return rows


def spread(values: list[float]) -> tuple[float, float, int]:
    """中央値と四分位範囲を求める.

    Args:
        values (list[float]): 対象の値.

    Returns:
        tuple[float, float, int]: 中央値、四分位範囲、点数.
    """
    array = np.asarray(values, dtype=float)
    if len(array) < 4:
        return float('nan'), float('nan'), len(array)
    return (float(np.median(array)),
            float(np.percentile(array, 75)-np.percentile(array, 25)), len(array))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cross-sections', type=Path, required=True)
    parser.add_argument('--regions', type=Path, required=True,
                        help='start_m,end_m,name を持つCSV')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--max-curvature', type=float, default=2.,
                        help='これを超える曲率の断面を除く [deg/m]')
    parser.add_argument('--target-iqr', type=float, default=.4,
                        help='幅一定とみなせるばらつきの上限 [m]')
    args = parser.parse_args()

    sections = load_sections(args.cross_sections, args.max_curvature)
    distance = np.array([float(r['distance_m']) for r in sections])
    regions = list(csv.DictReader(args.regions.open()))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    passed = 0
    with args.output.open('w', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(['start_m', 'end_m', 'name', 'source', 'side', 'samples',
                         'detected', 'detection_rate', 'median_m', 'iqr_m', 'judge'])
        print(f'{"区間":>13} {"種別":<12} {"源":<5} 側 {"検出":>9} {"中央":>6} {"IQR":>6}  判定')
        for region in regions:
            start, end = float(region['start_m']), float(region['end_m'])
            inside = (distance >= start)&(distance <= end)
            for label, left_key, right_key in SOURCES:
                for side_name, index in SIDES:
                    key = (left_key, right_key)[index]
                    values, total = [], 0
                    for row in (r for r, keep in zip(sections, inside) if keep):
                        total += 1
                        if row[key] and float(row[key]) > 0:
                            values.append(float(row[key]))
                    median, iqr, count = spread(values)
                    rate = count/total if total else 0.
                    judge = ('点数不足' if count < 10 else
                             '安定' if iqr <= args.target_iqr else '不安定')
                    passed += judge == '安定'
                    writer.writerow([f'{start:.0f}', f'{end:.0f}', region['name'], label,
                                     side_name, total, count, f'{rate:.2f}',
                                     f'{median:.2f}', f'{iqr:.2f}', judge])
                    print(f'{start:5.0f}-{end:5.0f} {region["name"][:12]:<12} {label:<5} '
                          f'{side_name} {count:3d}/{total:3d}({rate:.0%}) '
                          f'{median:6.2f} {iqr:6.2f}  {judge}')
    print(f'\n安定 {passed} / {len(regions)*len(SOURCES)*len(SIDES)} 件 '
          f'(IQR {args.target_iqr} m 以下) -> {args.output}', flush=True)


if __name__ == '__main__':
    main()
