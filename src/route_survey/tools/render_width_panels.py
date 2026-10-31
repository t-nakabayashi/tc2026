#!/usr/bin/env python3
"""路面場・経路・断面を、前方映像と同じ向きの俯瞰図にする.

カメラのコマと同じ地点・同じ向きで描くため、左右の判断を映像と直接照合できる。
断面は経路上の点を原点とし、進行方向の接線に垂直に引く。撮影位置からの直線外挿は
曲線部で経路を外れるため使わない。判定や幅の更新は行わず、確認用の画像を出すだけである。
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
from matplotlib.colors import ListedColormap

# 0:未観測 1:観測のみ 2:路面 3:危険(段差・障害) の4値で塗り分ける
PALETTE = ListedColormap(['#ffffff', '#d8d8d8', '#bfe6c8', '#f3b26a'])


def load_sections(path: Path) -> dict[int, dict]:
    """measure_cross_sections.py の出力を距離[m]で引けるようにする.

    Args:
        path (Path): 断面測定のCSV.

    Returns:
        dict[int, dict]: 距離[m]から断面の観測値への対応.
    """
    out = {}
    for row in csv.DictReader(path.open()):
        out[int(round(float(row['distance_m'])))] = dict(
            x=float(row['x']), y=float(row['y']), yaw=float(row['yaw_rad']),
            curvature=float(row['curvature_deg_per_m']),
            left=float(row['left_m']), right=float(row['right_m']),
            left_reason=row['left_reason'], right_reason=row['right_reason'])
    return out


def sample_masks(masks: dict, x: float, y: float, yaw: float, lateral: float,
                 back: float, ahead: float, pixel: float) -> np.ndarray:
    """進行方向を上にした局所画像へ、全コースの格子判定を取り出す.

    Args:
        masks (dict): surface_mask.npz の内容.
        x (float): 視点のENU X [m].
        y (float): 視点のENU Y [m].
        yaw (float): 視点の方位 [rad].
        lateral (float): 左右に描く範囲 [m].
        back (float): 後方に描く範囲 [m].
        ahead (float): 前方に描く範囲 [m].
        pixel (float): 1画素あたりの長さ [m].

    Returns:
        np.ndarray: 0-3の判定値を持つ画像配列（行が前方、列が左右）.
    """
    res = float(masks['resolution'])
    x0, _, y0, _ = masks['extent']
    surf, haz, obs = masks['surface'], masks['hazard'], masks['observed']
    v = np.arange(-back, ahead+1e-9, pixel)
    u = np.arange(-lateral, lateral+1e-9, pixel)
    vv, uu = np.meshgrid(v, u, indexing='ij')
    px = x+vv*np.cos(yaw)-uu*np.sin(yaw)
    py = y+vv*np.sin(yaw)+uu*np.cos(yaw)
    ix = np.clip(np.floor((px-x0)/res).astype(int), 0, surf.shape[1]-1)
    iy = np.clip(np.floor((py-y0)/res).astype(int), 0, surf.shape[0]-1)
    image = np.zeros(ix.shape, np.uint8)
    image[obs[iy, ix]] = 1
    image[surf[iy, ix]] = 2
    image[haz[iy, ix]] = 3
    return image


def to_view(points: np.ndarray, x: float, y: float, yaw: float) -> np.ndarray:
    """ENU座標を、視点を原点・進行方向を前方とする座標へ移す.

    Args:
        points (np.ndarray): ENU座標の点列 (N, 2).
        x (float): 視点のENU X [m].
        y (float): 視点のENU Y [m].
        yaw (float): 視点の方位 [rad].

    Returns:
        np.ndarray: (N, 2) の [左右, 前方]。左が正.
    """
    delta = np.asarray(points, dtype=float)-np.array([x, y])
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    forward = delta[:, 0]*cos_yaw+delta[:, 1]*sin_yaw
    lateral = -delta[:, 0]*sin_yaw+delta[:, 1]*cos_yaw
    return np.column_stack([lateral, forward])


def render(record: dict, masks: dict, section: dict, path: np.ndarray, output: Path,
           lateral: float, back: float, ahead: float, pixel: float) -> None:
    """1地点ぶんの俯瞰図を書き出す.

    Args:
        record (dict): frames.json の1件（カメラ位置と向き）.
        masks (dict): surface_mask.npz の内容.
        section (dict): 当該距離の断面観測値.
        path (np.ndarray): 経路のENU点列 (N, 2).
        output (Path): 出力先のPNGパス.
        lateral (float): 左右に描く範囲 [m].
        back (float): 後方に描く範囲 [m].
        ahead (float): 前方に描く範囲 [m].
        pixel (float): 1画素あたりの長さ [m].
    """
    target = record['target_distance_m']
    vx, vy, vyaw = record['x'], record['y'], record['yaw']
    image = sample_masks(masks, vx, vy, vyaw, lateral, back, ahead, pixel)
    fig, ax = plt.subplots(figsize=(4.2, 7.4))
    ax.imshow(image, origin='lower', cmap=PALETTE, vmin=0, vmax=3,
              extent=[-lateral, lateral, -back, ahead], interpolation='nearest')
    view = to_view(path, vx, vy, vyaw)
    inside = (np.abs(view[:, 0]) < lateral+2)&(view[:, 1] > -back-2)&(view[:, 1] < ahead+2)
    ax.plot(view[inside, 0], view[inside, 1], color='#2f6fb0', lw=1.4, alpha=.9)
    ax.plot(0., 0., marker='^', color='#2f6fb0', ms=11)
    if section is None:
        ax.set_title(f'{target:.0f} m 地点（断面の観測値なし）', fontsize=11)
    else:
        # 断面は経路上の点を原点とし、接線に垂直に引く
        origin = np.array([section['x'], section['y']])
        normal = np.array([-math.sin(section['yaw']), math.cos(section['yaw'])])
        ends = np.vstack([origin+normal*lateral, origin-normal*lateral])
        ev = to_view(ends, vx, vy, vyaw)
        ax.plot(ev[:, 0], ev[:, 1], color='#c0392b', lw=1.6)
        for value, sign, color in ((section['left'], 1, '#1b7f4b'),
                                   (section['right'], -1, '#1b4f9f')):
            if value > 0:
                seg = np.vstack([origin, origin+normal*sign*value])
                sv = to_view(seg, vx, vy, vyaw)
                ax.plot(sv[:, 0], sv[:, 1], color=color, lw=4, solid_capstyle='butt')
        ax.set_title(
            f'{target:.0f} m 断面（赤）  左 {section["left"]:.1f}({section["left_reason"]})'
            f' / 右 {section["right"]:.1f}({section["right_reason"]})\n'
            f'曲率 {section["curvature"]:.1f} deg/m', fontsize=10)
    for tick in (1., 2., 3.):
        if tick <= lateral:
            for sign in (1, -1):
                ax.axvline(sign*tick, color='#9aa6b2', lw=.5, ls=':')
    # 映像と見比べるため、車体から見た左を図でも左に置く
    ax.set_xlim(lateral, -lateral)
    ax.set_ylim(-back, ahead)
    ax.set_xlabel('← 左　　　右 → [m]')
    ax.set_ylabel('進行方向 [m]')
    ticks = np.arange(-int(lateral), int(lateral)+1, 1)
    ax.set_xticks(ticks)
    ax.set_xticklabels([str(abs(int(v))) for v in ticks])
    fig.tight_layout()
    fig.savefig(output, dpi=110)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--frames', type=Path, required=True, help='frames.json を含むディレクトリ')
    parser.add_argument('--surface-mask', type=Path, required=True)
    parser.add_argument('--cross-sections', type=Path, required=True,
                        help='measure_cross_sections.py の出力CSV')
    parser.add_argument('--waypoints', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--lateral', type=float, default=6.)
    parser.add_argument('--back', type=float, default=4.)
    parser.add_argument('--ahead', type=float, default=24.)
    parser.add_argument('--pixel', type=float, default=.05)
    args = parser.parse_args()

    plt.rcParams.update({'font.family': 'Noto Sans CJK JP', 'font.size': 11})
    records = json.loads((args.frames/'frames.json').read_text())
    masks = dict(np.load(args.surface_mask))
    sections = load_sections(args.cross_sections)
    rows = json.loads(args.waypoints.read_text())
    path = np.array([[float(r['x']), float(r['y'])] for r in rows])
    args.output.mkdir(parents=True, exist_ok=True)
    for record in records:
        name = Path(record['name']).with_suffix('.png').name
        section = sections.get(int(round(record['target_distance_m'])))
        render(record, masks, section, path, args.output/name,
               args.lateral, args.back, args.ahead, args.pixel)
    print(f'DONE {len(records)} panels -> {args.output}', flush=True)


if __name__ == '__main__':
    main()
