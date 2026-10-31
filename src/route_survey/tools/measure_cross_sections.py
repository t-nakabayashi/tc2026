#!/usr/bin/env python3
"""経路上の各点で、進行方向に垂直な断面の左右端までの距離を測る.

断面の原点は経路上の waypoint とし、向きは前後の waypoint から求めた接線の法線とする。
撮影位置から直線で外挿すると、曲線部で断面が経路から外れるため行わない。
曲率が大きい点では左右の意味が崩れるため、曲率を併記して利用者が除外できるようにする。
判定や幅の更新は行わず、観測値を出すだけである。
"""
import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np


class Route:
    """弧長で引ける経路。接線と曲率を前後の点から求める."""

    def __init__(self, waypoints: Path) -> None:
        rows = json.loads(waypoints.read_text())
        self.distance = np.array([float(r['distance_from_start_m']) for r in rows])
        self.point = np.array([[float(r['x']), float(r['y'])] for r in rows])
        self.label = [str(r['label']) for r in rows]
        order = np.argsort(self.distance)
        self.distance = self.distance[order]
        self.point = self.point[order]
        self.label = [self.label[i] for i in order]

    def index(self, s: float) -> int:
        """弧長 s に最も近い waypoint の番号を返す.

        Args:
            s (float): 経路始点からの距離 [m].

        Returns:
            int: waypoint の番号.
        """
        return int(np.argmin(np.abs(self.distance-s)))

    def tangent(self, s: float, span: float) -> float | None:
        """弧長 s での進行方位を、前後 span の 2 点から求める.

        Args:
            s (float): 経路始点からの距離 [m].
            span (float): 接線を取る前後の距離 [m].

        Returns:
            float | None: 方位 [rad]。2 点が重なる場合は None.
        """
        a = self.point[self.index(s-span)]
        b = self.point[self.index(s+span)]
        delta = b-a
        if float(np.hypot(*delta)) < 1e-6:
            return None
        return float(math.atan2(delta[1], delta[0]))

    def curvature(self, s: float, span: float) -> float:
        """弧長 s での方位変化率を求める.

        Args:
            s (float): 経路始点からの距離 [m].
            span (float): 方位差を取る前後の距離 [m].

        Returns:
            float: 方位変化率 [deg/m]. 求められない場合は 0.
        """
        before = self.tangent(s-span, span)
        after = self.tangent(s+span, span)
        if before is None or after is None:
            return 0.
        diff = math.atan2(math.sin(after-before), math.cos(after-before))
        return abs(math.degrees(diff))/(2*span)


class SurfaceField:
    """全コースの路面判定格子。断面に沿って端まで走査する."""

    def __init__(self, path: Path) -> None:
        data = np.load(path)
        self.surface = data['surface']
        self.hazard = data['hazard']
        self.observed = data['observed']
        self.resolution = float(data['resolution'])
        self.x0, _, self.y0, _ = data['extent']

    def edge(self, x: float, y: float, yaw: float, sign: int, limit: float,
             step: float) -> tuple[float, str]:
        """断面に沿って路面が連続している距離と、途切れた理由を返す.

        Args:
            x (float): 断面原点のENU X [m].
            y (float): 断面原点のENU Y [m].
            yaw (float): 進行方位 [rad].
            sign (int): 1 で左、-1 で右.
            limit (float): 走査の上限距離 [m].
            step (float): 走査の刻み [m].

        Returns:
            tuple[float, str]: 連続距離 [m] と理由.
        """
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        reach, reason = 0., '上限'
        for offset in np.arange(step, limit+1e-9, step):
            px = x-sign*sin_yaw*offset
            py = y+sign*cos_yaw*offset
            ix = int((px-self.x0)/self.resolution)
            iy = int((py-self.y0)/self.resolution)
            if not (0 <= ix < self.surface.shape[1] and 0 <= iy < self.surface.shape[0]):
                reason = '範囲外'
                break
            if self.hazard[iy, ix]:
                reason = '物体'
                break
            if not self.surface[iy, ix]:
                reason = '非路面' if self.observed[iy, ix] else '未観測'
                break
            reach = float(offset)
        return reach, reason


class BoundaryField:
    """URGの水平走査から得た鉛直物の分布。断面に沿って最初の境界までを測る."""

    def __init__(self, path: Path, threshold: float) -> None:
        data = np.load(path)
        self.ratio = data['ratio']
        self.threshold = threshold
        self.resolution = float(data['resolution'])
        self.x0, _, self.y0, _ = data['extent']

    def edge(self, x: float, y: float, yaw: float, sign: int, limit: float,
             step: float) -> float | None:
        """断面に沿って、最初に鉛直物を検出した距離を返す.

        Args:
            x (float): 断面原点のENU X [m].
            y (float): 断面原点のENU Y [m].
            yaw (float): 進行方位 [rad].
            sign (int): 1 で左、-1 で右.
            limit (float): 走査の上限距離 [m].
            step (float): 走査の刻み [m].

        Returns:
            float | None: 境界までの距離 [m]。検出しなければ None.
        """
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        for offset in np.arange(step, limit+1e-9, step):
            px = x-sign*sin_yaw*offset
            py = y+sign*cos_yaw*offset
            ix = int((px-self.x0)/self.resolution)
            iy = int((py-self.y0)/self.resolution)
            if not (0 <= ix < self.ratio.shape[1] and 0 <= iy < self.ratio.shape[0]):
                return None
            window = self.ratio[max(iy-1, 0):iy+2, max(ix-1, 0):ix+2]
            if window.size and float(window.max()) >= self.threshold:
                return float(offset)
        return None


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--waypoints', type=Path, required=True)
    parser.add_argument('--surface-mask', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--spacing', type=float, default=1., help='断面を取る間隔 [m]')
    parser.add_argument('--limit', type=float, default=5., help='左右の走査上限 [m]')
    parser.add_argument('--step', type=float, default=.1, help='走査の刻み [m]')
    parser.add_argument('--tangent-span', type=float, default=3., help='接線を取る前後の距離 [m]')
    parser.add_argument('--curvature-span', type=float, default=5., help='曲率を取る前後の距離 [m]')
    parser.add_argument('--urg-boundary', type=Path, default=None,
                        help='extract_urg_boundary.py の出力。指定すると鉛直境界の列を加える')
    parser.add_argument('--urg-threshold', type=float, default=.15,
                        help='鉛直物とみなす正規化当たり数の下限')
    args = parser.parse_args()

    route = Route(args.waypoints)
    field = SurfaceField(args.surface_mask)
    boundary = BoundaryField(args.urg_boundary, args.urg_threshold) if args.urg_boundary else None
    total = float(route.distance[-1])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    count = 0
    with args.output.open('w', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(['distance_m', 'label', 'x', 'y', 'yaw_rad', 'curvature_deg_per_m',
                         'left_m', 'left_reason', 'right_m', 'right_reason', 'width_m',
                         'urg_left_m', 'urg_right_m', 'urg_width_m'])
        for s in np.arange(0., total+1e-9, args.spacing):
            yaw = route.tangent(float(s), args.tangent_span)
            if yaw is None:
                continue
            i = route.index(float(s))
            x, y = route.point[i]
            left, left_reason = field.edge(x, y, yaw, 1, args.limit, args.step)
            right, right_reason = field.edge(x, y, yaw, -1, args.limit, args.step)
            urg_left = urg_right = None
            if boundary is not None:
                urg_left = boundary.edge(x, y, yaw, 1, args.limit, args.step)
                urg_right = boundary.edge(x, y, yaw, -1, args.limit, args.step)
            urg_width = ('' if urg_left is None or urg_right is None
                         else f'{urg_left+urg_right:.2f}')
            writer.writerow([f'{s:.1f}', route.label[i], f'{x:.3f}', f'{y:.3f}', f'{yaw:.5f}',
                             f'{route.curvature(float(s), args.curvature_span):.2f}',
                             f'{left:.2f}', left_reason, f'{right:.2f}', right_reason,
                             f'{left+right:.2f}',
                             '' if urg_left is None else f'{urg_left:.2f}',
                             '' if urg_right is None else f'{urg_right:.2f}', urg_width])
            count += 1
    print(f'DONE {count} 断面 -> {args.output}', flush=True)


if __name__ == '__main__':
    main()
