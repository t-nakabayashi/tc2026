# -*- coding: utf-8 -*-
"""水平スライス層の生成と横移動余裕の算出を行う ROS 非依存のコア.

座標系は base_link とし、x を前方、y を左を正とする。層の高さは床面基準で扱い、
base_link の床上高さ (`floor_to_base_link_m`) を介して z へ変換する。

停止判定には使わない観測値を作る処理であり、前方閉塞の判定は obstacle_monitor が
topURG の `/scan` のみから行う。本モジュールは AI 助言の入力と、回避幅の上限を
実測から与えることを目的とする。
"""

from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import math

import numpy as np

# 層の生成元
SOURCE_SCAN = 'scan'
SOURCE_POINT_CLOUD = 'point_cloud'


@dataclass(frozen=True)
class LayerSpec:
    """1 層の定義.

    Attributes:
        name (str): 層の識別名.
        source (str): 生成元 (`SOURCE_SCAN` または `SOURCE_POINT_CLOUD`).
        height_m (float): 床面からの層の中心高さ [m].
        thickness_m (float): 層の厚さ [m]. 中心から上下へ半分ずつ取る.
    """

    name: str
    source: str
    height_m: float
    thickness_m: float


@dataclass
class LayerObservation:
    """1 層の観測結果.

    `required_*` は前方帯が空く最小の横移動量で、通行可能な横移動量が無い場合は inf と
    する。`available_*` はその横移動量から連続して空いている最大値であり、required が
    inf の場合は 0 とする。
    """

    spec: LayerSpec
    point_count: int = 0
    nearest_x_m: float = math.inf
    required_left_offset_m: float = math.inf
    required_right_offset_m: float = math.inf
    available_left_offset_m: float = 0.0
    available_right_offset_m: float = 0.0
    points_xy: np.ndarray = field(
        default_factory=lambda: np.empty((0, 2), dtype=np.float32))
    free_intervals: List[Tuple[float, float]] = field(default_factory=list)


@dataclass(frozen=True)
class ClearanceConfig:
    """横移動余裕の判定条件.

    Attributes:
        forward_range_m (float): 評価する前方距離の上限 [m].
        robot_width_m (float): 車体全幅 [m].
        side_margin_m (float): 車体幅へ加える左右の余裕 [m].
        max_offset_m (float): 探索する横移動量の上限 [m].
        near_x_m (float): 評価から除外する近距離の下限 [m]. 車体直前の自己反射を除く.
    """

    forward_range_m: float = 2.0
    robot_width_m: float = 0.6
    side_margin_m: float = 0.1
    max_offset_m: float = 3.0
    near_x_m: float = 0.15

    @property
    def swath_half_m(self) -> float:
        """車体が占有する横幅の半分（余裕込み）[m] を返す."""
        return self.robot_width_m / 2.0 + self.side_margin_m


def slice_layer_points(points_xyz: np.ndarray, spec: LayerSpec,
                       floor_to_base_link_m: float) -> np.ndarray:
    """base_link 系の点群から 1 層分の xy 点を切り出す.

    Args:
        points_xyz (np.ndarray): base_link 系の点群. shape = (N, 3).
        spec (LayerSpec): 層の定義.
        floor_to_base_link_m (float): 床面から base_link 原点までの高さ [m].

    Returns:
        np.ndarray: 層に含まれる点の xy 座標. shape = (M, 2).
    """
    if points_xyz is None or len(points_xyz) == 0:
        return np.empty((0, 2), dtype=np.float32)
    array = np.asarray(points_xyz, dtype=np.float32)
    if array.ndim != 2 or array.shape[1] < 3:
        return np.empty((0, 2), dtype=np.float32)
    # 床面基準の高さへ変換してから層の厚さで抽出する。
    height = array[:, 2] + float(floor_to_base_link_m)
    half = max(float(spec.thickness_m), 0.0) / 2.0
    inside = np.abs(height - float(spec.height_m)) <= half
    finite = np.isfinite(array[:, 0]) & np.isfinite(array[:, 1]) & np.isfinite(array[:, 2])
    return array[inside & finite][:, :2].astype(np.float32, copy=False)


def scan_to_xy(ranges: Sequence[float], angle_min: float, angle_increment: float,
               range_min: float = 0.0, range_max: float = math.inf,
               half_fov_deg: float = 90.0) -> np.ndarray:
    """LaserScan の距離列を xy 点へ変換する.

    前方 ±`half_fov_deg` の範囲のみ残す。無効値 (NaN / inf / 範囲外) は除外する。

    Args:
        ranges (Sequence[float]): 距離列 [m].
        angle_min (float): 最初の計測角 [rad].
        angle_increment (float): 計測角の増分 [rad].
        range_min (float): 有効な最小距離 [m].
        range_max (float): 有効な最大距離 [m].
        half_fov_deg (float): 残す前方範囲の半角 [deg].

    Returns:
        np.ndarray: xy 座標. shape = (M, 2).
    """
    if ranges is None or len(ranges) == 0:
        return np.empty((0, 2), dtype=np.float32)
    values = np.asarray(ranges, dtype=np.float64)
    index = np.arange(values.size, dtype=np.float64)
    angles = float(angle_min) + index * float(angle_increment)
    half_fov = math.radians(max(half_fov_deg, 0.0))
    valid = (np.isfinite(values) & (values >= float(range_min))
             & (values <= float(range_max)) & (np.abs(angles) <= half_fov))
    if not np.any(valid):
        return np.empty((0, 2), dtype=np.float32)
    radius = values[valid]
    angle = angles[valid]
    return np.stack([radius * np.cos(angle), radius * np.sin(angle)],
                    axis=1).astype(np.float32)


def free_intervals(points_xy: np.ndarray, config: ClearanceConfig) -> List[Tuple[float, float]]:
    """車体中心を横へ寄せられる区間を求める.

    前方帯 (0 < x <= forward_range_m) にある点それぞれが、車体占有幅の分だけ横移動量を
    塞ぐと考え、塞がれていない区間を返す。返す区間は車体中心の横移動量 [m] の範囲で、
    左が正である。

    Args:
        points_xy (np.ndarray): 層の xy 点. shape = (N, 2).
        config (ClearanceConfig): 判定条件.

    Returns:
        List[Tuple[float, float]]: 空いている横移動量の区間 (lo, hi) の一覧.
    """
    limit = float(config.max_offset_m)
    if limit <= 0.0:
        return []
    blocked: List[Tuple[float, float]] = []
    if points_xy is not None and len(points_xy) > 0:
        array = np.asarray(points_xy, dtype=np.float32)
        forward = (array[:, 0] > float(config.near_x_m)) & (array[:, 0] <= float(config.forward_range_m))
        half = config.swath_half_m
        for y in array[forward][:, 1]:
            blocked.append((float(y) - half, float(y) + half))
    if not blocked:
        return [(-limit, limit)]
    blocked.sort()
    free: List[Tuple[float, float]] = []
    cursor = -limit
    for lo, hi in blocked:
        if hi <= cursor:
            continue
        if lo > cursor:
            upper = min(lo, limit)
            if upper > cursor:
                free.append((cursor, upper))
        cursor = max(cursor, hi)
        if cursor >= limit:
            return free
    if cursor < limit:
        free.append((cursor, limit))
    return free


def offsets_from_intervals(intervals: Sequence[Tuple[float, float]],
                           ) -> Tuple[float, float, float, float]:
    """空き区間から左右の required / available 横移動量を求める.

    Args:
        intervals (Sequence[Tuple[float, float]]): `free_intervals` の戻り値.

    Returns:
        Tuple[float, float, float, float]:
            (required_left, available_left, required_right, available_right) [m].
            通行可能な横移動量が無い側は required=inf、available=0.0 とする.
    """
    required_left, available_left = math.inf, 0.0
    required_right, available_right = math.inf, 0.0
    # 左側: 0 以上の横移動量を含む最初の区間を採る。
    for lo, hi in sorted(intervals):
        if hi < 0.0:
            continue
        required_left = max(lo, 0.0)
        available_left = hi
        break
    # 右側: 0 以下の横移動量を含む区間のうち、0 に最も近いものを採る。
    for lo, hi in sorted(intervals, reverse=True):
        if lo > 0.0:
            continue
        required_right = max(-hi, 0.0)
        available_right = -lo
        break
    return required_left, available_left, required_right, available_right


def nearest_forward_distance(points_xy: np.ndarray, config: ClearanceConfig) -> float:
    """車体幅の帯内で最も近い前方距離 [m] を返す. 該当点が無ければ inf."""
    if points_xy is None or len(points_xy) == 0:
        return math.inf
    array = np.asarray(points_xy, dtype=np.float32)
    half = config.swath_half_m
    mask = (array[:, 0] > float(config.near_x_m)) & (np.abs(array[:, 1]) <= half)
    if not np.any(mask):
        return math.inf
    return float(np.min(array[mask][:, 0]))


def observe_layer(points_xy: np.ndarray, spec: LayerSpec,
                  config: ClearanceConfig) -> LayerObservation:
    """1 層分の xy 点から観測結果を作る."""
    intervals = free_intervals(points_xy, config)
    required_left, available_left, required_right, available_right = \
        offsets_from_intervals(intervals)
    points = (np.asarray(points_xy, dtype=np.float32) if points_xy is not None
              else np.empty((0, 2), dtype=np.float32))
    return LayerObservation(
        spec=spec,
        point_count=int(len(points)),
        nearest_x_m=nearest_forward_distance(points, config),
        required_left_offset_m=required_left,
        required_right_offset_m=required_right,
        available_left_offset_m=available_left,
        available_right_offset_m=available_right,
        points_xy=points,
        free_intervals=intervals,
    )


def combined_available_offsets(observations: Sequence[LayerObservation],
                               ) -> Tuple[float, float]:
    """全層で共通に使える左右の最大横移動量 [m] を返す.

    層ごとの available の最小値を採る。点が無い層は未観測であり、障害物が無いことを
    意味しないため、判定へ含めない。全層が未観測の場合は (0.0, 0.0) を返す。
    """
    left_candidates = [o.available_left_offset_m for o in observations if o.point_count > 0]
    right_candidates = [o.available_right_offset_m for o in observations if o.point_count > 0]
    left = min(left_candidates) if left_candidates else 0.0
    right = min(right_candidates) if right_candidates else 0.0
    return float(left), float(right)


def layer_context_lines(observations: Sequence[LayerObservation],
                        config: ClearanceConfig) -> List[str]:
    """AI へ渡す構造化テキストの行を作る.

    画像から寸法を読み取らせず、数値はこの行で渡す。単位を各行に明記する。
    """
    def fmt(value: float) -> str:
        return 'inf' if not math.isfinite(value) else f'{value:.2f}'

    lines = [
        f'robot_width_m={config.robot_width_m:.2f} '
        f'side_margin_m={config.side_margin_m:.2f} '
        f'forward_range_m={config.forward_range_m:.2f}',
    ]
    for observation in observations:
        spec = observation.spec
        lines.append(
            f'layer={spec.name} height_m={spec.height_m:.2f} '
            f'source={spec.source} points={observation.point_count} '
            f'nearest_x_m={fmt(observation.nearest_x_m)} '
            f'required_left_m={fmt(observation.required_left_offset_m)} '
            f'available_left_m={fmt(observation.available_left_offset_m)} '
            f'required_right_m={fmt(observation.required_right_offset_m)} '
            f'available_right_m={fmt(observation.available_right_offset_m)}')
    left, right = combined_available_offsets(observations)
    lines.append(f'combined_available_left_m={fmt(left)} '
                 f'combined_available_right_m={fmt(right)}')
    return lines


def observable_height_range(distance_m: float, sensor_height_m: float,
                            pitch_down_deg: float, elevation_min_deg: float,
                            elevation_max_deg: float) -> Tuple[float, float]:
    """前下がり取付の LiDAR が、ある距離で観測できる床上高さの範囲 [m] を返す.

    層の高さを選ぶ根拠を実装側でも確認できるようにするための補助である。
    仰角は水平を 0、上向きを正とし、前下がり角は正で前方を下げる向きとする。

    Args:
        distance_m (float): 前方距離 [m].
        sensor_height_m (float): センサの床上高さ [m].
        pitch_down_deg (float): 前下がり角 [deg].
        elevation_min_deg (float): センサ固有の最小仰角 [deg].
        elevation_max_deg (float): センサ固有の最大仰角 [deg].

    Returns:
        Tuple[float, float]: (最低高さ, 最高高さ) [m]. 床面より下は 0.0 で丸める.
    """
    lower_deg = float(elevation_min_deg) - float(pitch_down_deg)
    upper_deg = float(elevation_max_deg) - float(pitch_down_deg)
    lower = sensor_height_m + math.tan(math.radians(lower_deg)) * float(distance_m)
    upper = sensor_height_m + math.tan(math.radians(upper_deg)) * float(distance_m)
    return max(lower, 0.0), max(upper, 0.0)


def minimum_observable_distance(height_m: float, sensor_height_m: float,
                                pitch_down_deg: float,
                                elevation_min_deg: float) -> Optional[float]:
    """ある床上高さの層を観測できる最短距離 [m] を返す.

    センサ高より高い層、または最下段ビームが水平以上を向く場合は None を返す。
    """
    drop = float(sensor_height_m) - float(height_m)
    if drop <= 0.0:
        return 0.0
    angle_deg = float(pitch_down_deg) - float(elevation_min_deg)
    if angle_deg <= 0.0:
        return None
    return drop / math.tan(math.radians(angle_deg))
