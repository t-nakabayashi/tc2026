#!/usr/bin/env python3
"""経路全体の回避幅を、ENU座標の俯瞰図と距離方向の帯で一枚に描く.

区間ごとの値を個別に確かめても全域の傾向は掴めないため、全waypointを実寸で描き、
左右の回避幅を経路の両側へ実寸のまま広げた領域として重ねる。区間の切れ目と番号を
俯瞰図と距離方向の図の双方へ入れ、同じ番号で対応を追えるようにする。
判定や幅の更新は行わず、入力をそのまま描くだけである。
"""
import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

# 幅の色は 0 m を赤、上限 2.5 m を緑とし、途中を黄で繋ぐ
WIDTH_COLORMAP = 'RdYlGn'


def load_geometry(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """ENU座標のwaypointから、位置・方位・弧長を取り出す.

    Args:
        path (Path): waypoints_enu.json のパス.

    Returns:
        tuple[np.ndarray, np.ndarray, np.ndarray]: 位置[m]、方位[rad]、弧長[m].
    """
    rows = json.loads(path.read_text())
    rows.sort(key=lambda r: float(r['distance_from_start_m']))
    point = np.array([[float(r['x']), float(r['y'])] for r in rows])
    yaw = np.array([float(r['yaw']) for r in rows])
    distance = np.array([float(r['distance_from_start_m']) for r in rows])
    return point, yaw, distance


def load_widths(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """走行用のwaypointリストから、左右の回避幅を取り出す.

    Args:
        path (Path): waypoints.csv のパス.

    Returns:
        tuple[np.ndarray, np.ndarray]: 左の幅[m]、右の幅[m].
    """
    left, right = [], []
    for row in csv.DictReader(path.open()):
        left.append(float(row['left_is_open']))
        right.append(float(row['right_is_open']))
    return np.array(left), np.array(right)


def load_regions(path: Path) -> list[dict]:
    """区間表を読み、開始・終了・名称を順に並べる.

    Args:
        path (Path): regions.csv のパス.

    Returns:
        list[dict]: 区間の定義を開始距離の順に並べたもの.
    """
    rows = [dict(start=float(r['start_m']), end=float(r['end_m']), name=r['name'],
                 left_type=r['left_type'], right_type=r['right_type'])
            for r in csv.DictReader(path.open())]
    rows.sort(key=lambda r: r['start'])
    return rows


def offsets(point: np.ndarray, yaw: np.ndarray, left: np.ndarray,
            right: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """左右の回避幅だけ経路を横へずらした境界線を作る.

    Args:
        point (np.ndarray): 経路上の位置 [m].
        yaw (np.ndarray): 進行方位 [rad].
        left (np.ndarray): 左の回避幅 [m].
        right (np.ndarray): 右の回避幅 [m].

    Returns:
        tuple[np.ndarray, np.ndarray]: 左端の列、右端の列.
    """
    normal = np.column_stack([-np.sin(yaw), np.cos(yaw)])
    return point+normal*left[:, None], point-normal*right[:, None]


def colored_route(axis, point: np.ndarray, value: np.ndarray, cap: float,
                  width: float) -> LineCollection:
    """経路を区間ごとに幅の値で塗り分けた線として描く.

    Args:
        axis: 描画先の座標軸.
        point (np.ndarray): 経路上の位置 [m].
        value (np.ndarray): 各点の幅 [m].
        cap (float): 色の上限とする幅 [m].
        width (float): 線の太さ.

    Returns:
        LineCollection: 色付けに使った線の集合.
    """
    segments = np.stack([point[:-1], point[1:]], axis=1)
    collection = LineCollection(segments, cmap=WIDTH_COLORMAP, linewidths=width)
    collection.set_array(np.minimum(value[:-1], cap))
    collection.set_clim(0., cap)
    axis.add_collection(collection)
    return collection


def draw_map(axis, point: np.ndarray, yaw: np.ndarray, left: np.ndarray,
             right: np.ndarray, distance: np.ndarray, regions: list[dict],
             cap: float) -> LineCollection:
    """ENU座標の俯瞰図へ、経路・回避領域・区間の切れ目を描く.

    Args:
        axis: 描画先の座標軸.
        point (np.ndarray): 経路上の位置 [m].
        yaw (np.ndarray): 進行方位 [rad].
        left (np.ndarray): 左の回避幅 [m].
        right (np.ndarray): 右の回避幅 [m].
        distance (np.ndarray): 弧長 [m].
        regions (list[dict]): 区間の定義.
        cap (float): 色の上限とする幅 [m].

    Returns:
        LineCollection: 幅で塗り分けた経路の線.
    """
    edge_left, edge_right = offsets(point, yaw, left, right)
    # 回避できる範囲を実寸のまま塗る。拡大すれば区間ごとの広がりをそのまま読める
    band = np.concatenate([edge_left, edge_right[::-1]])
    axis.fill(band[:, 0], band[:, 1], color='#8fb8e0', alpha=.45, linewidth=0,
              label='回避できる範囲（実寸）')
    collection = colored_route(axis, point, np.minimum(left, right), cap, 1.6)
    for number, region in enumerate(regions, start=1):
        index = int(np.argmin(np.abs(distance-region['start'])))
        axis.plot(point[index, 0], point[index, 1], marker='|', color='#333333',
                  markersize=7, markeredgewidth=1.2)
        axis.annotate(str(number), point[index], textcoords='offset points',
                      xytext=(4, 4), fontsize=7, color='#333333')
    axis.plot(point[0, 0], point[0, 1], marker='o', color='#1a1a1a', markersize=6)
    axis.annotate('出発', point[0], textcoords='offset points', xytext=(6, -12),
                  fontsize=9)
    axis.set_aspect('equal')
    axis.set_xlabel('ENU X [m]')
    axis.set_ylabel('ENU Y [m]')
    axis.set_title(f'全{len(point)}点の回避幅（色は左右の小さい方、帯は実寸の回避範囲）')
    axis.grid(color='#eeeeee')
    axis.legend(loc='upper right', fontsize=9)
    return collection


def load_run(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """走行記録の集計から、距離と横ずれを取り出す.

    Args:
        path (Path): summarize_avoidance_run.py の出力CSV.

    Returns:
        tuple[np.ndarray, np.ndarray]: 始点からの距離[m]、横ずれ[m]（左が正）.
    """
    rows = list(csv.DictReader(path.open()))
    return (np.array([float(r['distance_m']) for r in rows]),
            np.array([float(r['lateral_m']) for r in rows]))


def draw_profile(axis, distance: np.ndarray, left: np.ndarray, right: np.ndarray,
                 regions: list[dict], cap: float, run: list = None) -> None:
    """距離方向に、左右の回避幅と区間の切れ目を描く.

    Args:
        axis: 描画先の座標軸.
        distance (np.ndarray): 弧長 [m].
        left (np.ndarray): 左の回避幅 [m].
        right (np.ndarray): 右の回避幅 [m].
        regions (list[dict]): 区間の定義.
        cap (float): 縦軸の上限 [m].

    Returns:
        None
    """
    axis.fill_between(distance, 0, left, step='mid', color='#4c78a8', alpha=.75,
                      label='左の回避幅')
    axis.fill_between(distance, 0, -right, step='mid', color='#e45756', alpha=.75,
                      label='右の回避幅')
    for number, region in enumerate(regions, start=1):
        axis.axvline(region['start'], color='#999999', linewidth=.6)
        axis.annotate(str(number), (region['start'], cap*1.02),
                      fontsize=7, color='#333333')
    for number, (travelled, lateral) in enumerate(run or []):
        axis.plot(travelled, lateral, color='#111111', linewidth=.8, linestyle='none',
                  marker='.', markersize=1,
                  label='走行時の横ずれ' if number == 0 else None)
    axis.axhline(0, color='#333333', linewidth=.8)
    axis.set_xlim(distance[0], distance[-1])
    axis.set_ylim(-cap*1.1, cap*1.1)
    axis.set_xlabel('出発からの距離 [m]')
    axis.set_ylabel('回避幅 [m]（上が左、下が右）')
    axis.grid(axis='x', color='#eeeeee')
    axis.legend(loc='lower right', fontsize=9, ncol=2)


def region_table(regions: list[dict], distance: np.ndarray, left: np.ndarray,
                 right: np.ndarray) -> list[tuple[str, str, str, str, str]]:
    """区間ごとの番号・区間・左右の代表値・名称を列に分けて並べる.

    Args:
        regions (list[dict]): 区間の定義.
        distance (np.ndarray): 弧長 [m].
        left (np.ndarray): 左の回避幅 [m].
        right (np.ndarray): 右の回避幅 [m].

    Returns:
        list[tuple[str, str, str, str, str]]: 番号、区間[m]、左、右、名称.
    """
    rows = []
    for number, region in enumerate(regions, start=1):
        inside = (distance >= region['start']) & (distance < region['end'])
        span = f'{region["start"]:.0f}-{region["end"]:.0f}'
        if not inside.any():
            rows.append((str(number), span, '-', '-', region['name']))
            continue
        rows.append((str(number), span, f'{np.median(left[inside]):.1f}',
                     f'{np.median(right[inside]):.1f}', region['name']))
    return rows


def draw_table(axis, rows: list[tuple[str, str, str, str, str]]) -> None:
    """区間の一覧を、2段の表として図の下へ置く.

    Args:
        axis: 描画先の座標軸.
        rows (list[tuple[str, str, str, str, str]]): 区間ごとの表示内容.

    Returns:
        None
    """
    axis.axis('off')
    half = (len(rows)+1)//2
    # 列ごとに x を固定して書く。日本語等幅フォントが無く、空白詰めでは揃わない
    columns = [(.00, 'right', '番号'), (.05, 'right', '区間 [m]'),
               (.14, 'right', '左'), (.18, 'right', '右'), (.20, 'left', '区間の名称')]
    for block, chunk in enumerate([rows[:half], rows[half:]]):
        base = block*.5
        for (offset, align, title), index in zip(columns, range(5)):
            axis.text(base+offset, 1., title, va='top', ha=align, fontsize=9,
                      color='#555555')
            for line, row in enumerate(chunk, start=1):
                axis.text(base+offset, 1.-line*.055, row[index], va='top', ha=align,
                          fontsize=9)


def main() -> None:
    """引数を受け取り、俯瞰図と距離方向の図を一枚に出力する.

    Returns:
        None
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--waypoints-enu', type=Path, required=True,
                        help='ENU座標と方位を持つ waypoints_enu.json')
    parser.add_argument('--route', type=Path, required=True,
                        help='回避幅を書き込んだ waypoints.csv')
    parser.add_argument('--regions', type=Path, required=True, help='区間表 regions.csv')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--run', type=Path, action='append',
                        help='走行記録の集計CSV。指定すると横ずれを重ねる。複数指定できる')
    parser.add_argument('--cap', type=float, default=None,
                        help='色と縦軸の上限 [m]。既定は入力の最大値')
    parser.add_argument('--dpi', type=int, default=160)
    args = parser.parse_args()

    plt.rcParams.update({'font.family': 'Noto Sans CJK JP', 'font.size': 11})
    point, yaw, distance = load_geometry(args.waypoints_enu)
    left, right = load_widths(args.route)
    if len(left) != len(point):
        raise SystemExit(f'点数が一致しません: ENU {len(point)} 件, 経路 {len(left)} 件')
    regions = load_regions(args.regions)
    cap = args.cap if args.cap else float(max(left.max(), right.max()))

    figure = plt.figure(figsize=(16, 17))
    grid = figure.add_gridspec(3, 1, height_ratios=[6, 1.6, 2.4], hspace=.22)
    collection = draw_map(figure.add_subplot(grid[0]), point, yaw, left, right,
                          distance, regions, cap)
    figure.colorbar(collection, ax=figure.axes[0], shrink=.6, label='回避幅 [m]')
    draw_profile(figure.add_subplot(grid[1]), distance, left, right, regions, cap,
                 [load_run(path) for path in args.run or []])
    draw_table(figure.add_subplot(grid[2]), region_table(regions, distance, left, right))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=args.dpi, bbox_inches='tight')
    print(f'出力: {args.output}')
    print(f'左の幅 中央値 {np.median(left):.2f} m / 0 の点 {int((left == 0).sum())} 件')
    print(f'右の幅 中央値 {np.median(right):.2f} m / 0 の点 {int((right == 0).sum())} 件')


if __name__ == '__main__':
    main()
