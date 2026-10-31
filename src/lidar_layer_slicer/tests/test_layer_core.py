"""水平スライス層の切り出しと横移動余裕の算出を固定する。ROS グラフを使わない。

層の高さは床面基準であり、点群は base_link 系で与える。判定条件は車体全幅 0.6 m、
左右余裕 0.1 m（車体占有半幅 0.4 m）、前方 2.0 m を既定とする。
"""

from pathlib import Path
import math
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
from lidar_layer_slicer.layer_core import (SOURCE_POINT_CLOUD, ClearanceConfig,
                                           LayerSpec, combined_available_offsets,
                                           free_intervals, layer_context_lines,
                                           minimum_observable_distance,
                                           observable_height_range, observe_layer,
                                           offsets_from_intervals, scan_to_xy,
                                           slice_layer_points)

FLOOR_TO_BASE_LINK_M = 0.148248
CONFIG = ClearanceConfig(forward_range_m=2.0, robot_width_m=0.6, side_margin_m=0.1,
                         max_offset_m=3.0, near_x_m=0.15)
SPEC = LayerSpec(name='mid', source=SOURCE_POINT_CLOUD, height_m=0.5, thickness_m=0.1)


def test_slice_takes_points_within_thickness_of_floor_height() -> None:
    """床上高さが層の厚さの範囲内にある点だけが切り出される.

    層は床面基準で指定し、点群は base_link 系で与えるため、base_link の床上高さを
    足して判定する。厚さ 0.1 m は中心から上下 0.05 m を意味する。
    """
    base_z = 0.5 - FLOOR_TO_BASE_LINK_M  # 床上 0.50 m に相当する base_link の z
    points = np.asarray([
        [1.0, 0.0, base_z],          # 層の中心
        [1.1, 0.0, base_z + 0.04],   # 層の内側
        [1.2, 0.0, base_z + 0.06],   # 層の外側
        [1.3, 0.0, base_z - 0.20],   # 層より下
    ], dtype=np.float32)

    sliced = slice_layer_points(points, SPEC, FLOOR_TO_BASE_LINK_M)

    assert sliced.shape == (2, 2)
    assert np.allclose(sliced[:, 0], [1.0, 1.1])


def test_slice_of_empty_cloud_returns_empty_layer() -> None:
    """点群が空のとき、空の層を返す（例外にしない）."""
    assert slice_layer_points(np.empty((0, 3), dtype=np.float32), SPEC,
                              FLOOR_TO_BASE_LINK_M).shape == (0, 2)


def test_no_points_leaves_whole_offset_range_free() -> None:
    """点が無い層では横移動量の全範囲が空き、required は 0 になる.

    点が無いことは未観測であることを意味し、障害物が無いことの保証ではない。
    `point_count` が 0 である事実を購読側が確認する必要がある。
    """
    observation = observe_layer(np.empty((0, 2), dtype=np.float32), SPEC, CONFIG)

    assert observation.point_count == 0
    assert observation.required_left_offset_m == 0.0
    assert observation.required_right_offset_m == 0.0
    assert observation.available_left_offset_m == CONFIG.max_offset_m
    assert math.isinf(observation.nearest_x_m)


def test_center_obstacle_requires_offset_of_occupied_half_width() -> None:
    """正面の点は、車体占有半幅だけ横へ寄れば避けられると算出される.

    車体占有半幅は 0.6 / 2 + 0.1 = 0.4 m であり、y=0 の点は横移動量 -0.4〜+0.4 を塞ぐ。
    """
    points = np.asarray([[1.0, 0.0]], dtype=np.float32)

    observation = observe_layer(points, SPEC, CONFIG)

    assert observation.required_left_offset_m == pytest.approx(0.4)
    assert observation.required_right_offset_m == pytest.approx(0.4)
    assert observation.available_left_offset_m == CONFIG.max_offset_m
    assert observation.nearest_x_m == 1.0


def test_wall_on_left_limits_available_offset_on_that_side() -> None:
    """左の壁は左側の使用可能な横移動量を制限し、右側へは影響しない."""
    points = np.asarray([[1.0, 0.0], [1.0, 1.2], [1.5, 1.2]], dtype=np.float32)

    observation = observe_layer(points, SPEC, CONFIG)

    assert observation.required_left_offset_m == pytest.approx(0.4)
    # 壁 1.2 m から車体占有半幅 0.4 m を引いた 0.8 m までしか寄れない。
    assert observation.available_left_offset_m == pytest.approx(0.8, abs=1e-6)
    assert observation.required_right_offset_m == pytest.approx(0.4)
    assert observation.available_right_offset_m == CONFIG.max_offset_m


def test_narrow_corridor_reports_no_feasible_offset() -> None:
    """車体が通れない幅しか無い場合、required は inf、available は 0 になる."""
    points = np.asarray([[1.0, y] for y in np.arange(-3.0, 3.01, 0.1)], dtype=np.float32)

    observation = observe_layer(points, SPEC, CONFIG)

    assert math.isinf(observation.required_left_offset_m)
    assert math.isinf(observation.required_right_offset_m)
    assert observation.available_left_offset_m == 0.0
    assert observation.available_right_offset_m == 0.0


def test_points_beyond_forward_range_do_not_block_but_are_still_reported() -> None:
    """評価する前方距離より遠い点は横移動量を塞がないが、最近傍距離には現れる.

    横移動量の判定は `forward_range_m` までの帯で行う。一方 `nearest_x_m` は距離の
    上限を持たず、直進帯で最も近い点を報告する。遠方の障害物を回避幅の制限には
    使わないが、AI へ渡す状況説明からは落とさない。
    """
    points = np.asarray([[2.5, 0.0]], dtype=np.float32)

    observation = observe_layer(points, SPEC, CONFIG)

    assert observation.required_left_offset_m == 0.0
    assert observation.nearest_x_m == 2.5


def test_points_closer_than_near_x_are_ignored() -> None:
    """車体直前の近距離点は評価から除外する."""
    points = np.asarray([[0.10, 0.0]], dtype=np.float32)

    observation = observe_layer(points, SPEC, CONFIG)

    assert observation.required_left_offset_m == 0.0
    assert math.isinf(observation.nearest_x_m)


def test_free_intervals_merge_overlapping_obstacles() -> None:
    """隣接する点が作る塞がれた区間は統合される."""
    points = np.asarray([[1.0, 0.0], [1.0, 0.3], [1.0, 0.6]], dtype=np.float32)

    intervals = free_intervals(points, CONFIG)

    assert len(intervals) == 2
    assert intervals[0][1] == pytest.approx(-0.4)
    assert intervals[1][0] == pytest.approx(1.0, abs=1e-6)


def test_offsets_from_intervals_picks_interval_nearest_to_center() -> None:
    """左右それぞれ、横移動量 0 に最も近い空き区間を採る."""
    intervals = [(-2.0, -1.0), (0.5, 1.5), (2.0, 3.0)]

    left_required, left_available, right_required, right_available = \
        offsets_from_intervals(intervals)

    assert (left_required, left_available) == (0.5, 1.5)
    assert (right_required, right_available) == (1.0, 2.0)


def test_combined_offsets_ignore_unobserved_layers() -> None:
    """点が無い層は共通の使用可能幅の算出へ含めない.

    未観測の層を「空いている」として採ると、観測できていない高さの障害物へ
    向かって幅を広げることになる。逆に 0 として採ると、観測できる距離に達して
    いない層のせいで回避できなくなる。点数 0 の層は判定から外す。
    """
    observed = observe_layer(np.asarray([[1.0, 1.0]], dtype=np.float32), SPEC, CONFIG)
    unobserved = observe_layer(np.empty((0, 2), dtype=np.float32), SPEC, CONFIG)

    left, right = combined_available_offsets([observed, unobserved])

    assert left == pytest.approx(0.6, abs=1e-6)
    assert right == CONFIG.max_offset_m


def test_combined_offsets_are_zero_when_all_layers_unobserved() -> None:
    """全層が未観測なら、使用可能幅を 0 として扱う."""
    unobserved = observe_layer(np.empty((0, 2), dtype=np.float32), SPEC, CONFIG)

    assert combined_available_offsets([unobserved, unobserved]) == (0.0, 0.0)


def test_scan_to_xy_drops_invalid_and_rear_measurements() -> None:
    """無効値と後方の計測は除外される."""
    ranges = [1.0, float('inf'), float('nan'), 2.0, 0.0]
    # -180° から 90° 刻み。前方 ±90° に入るのは index 2, 3 のみ。
    points = scan_to_xy(ranges, angle_min=-math.pi, angle_increment=math.pi / 2,
                        range_min=0.1, range_max=30.0)

    assert points.shape == (1, 2)
    assert points[0][0] == pytest.approx(0.0, abs=1e-6)
    assert points[0][1] == pytest.approx(2.0)


def test_context_lines_report_values_with_units() -> None:
    """AI へ渡す構造化テキストに、層ごとの数値と単位付きの名前が並ぶ."""
    observation = observe_layer(np.asarray([[1.0, 0.0]], dtype=np.float32), SPEC, CONFIG)

    lines = layer_context_lines([observation], CONFIG)

    assert lines[0].startswith('robot_width_m=0.60')
    assert 'layer=mid height_m=0.50' in lines[1]
    assert 'nearest_x_m=1.00' in lines[1]
    assert lines[-1].startswith('combined_available_left_m=')


def test_observable_height_range_matches_mount_geometry() -> None:
    """前下がり 26.9°、床上 0.562 m の MID-360 で観測できる高さ帯を確認する.

    仰角 -7°〜+52° が 26.9° 前下がりで回転し、車体基準で -33.9°〜+25.1° になる。
    1.0 m 前方では床面から約 1.03 m までが観測範囲に入る。
    """
    lower, upper = observable_height_range(1.0, 0.562, 26.9, -7.0, 52.0)

    assert lower == 0.0  # 最下段ビームは 0.84 m で地面へ達する
    assert 1.02 < upper < 1.04


def test_minimum_observable_distance_for_each_layer_height() -> None:
    """層の高さごとに観測できる最短距離を確認する.

    床上 0.35 m と 0.50 m の層は、停止判定が起きる 0.75 m より手前から観測できる。
    """
    low = minimum_observable_distance(0.35, 0.562, 26.9, -7.0)
    mid = minimum_observable_distance(0.50, 0.562, 26.9, -7.0)

    assert 0.31 < low < 0.33
    assert 0.08 < mid < 0.10


def test_minimum_observable_distance_is_unavailable_for_horizontal_mount() -> None:
    """水平取付では、センサより低い層を近傍で観測できない.

    前下がり 0° では最下段ビームが -7° であり、床上 0.35 m の層は 1.7 m 以遠まで
    観測できない。層の高さを選ぶ前提が取付角に依存することを固定する。
    """
    distance = minimum_observable_distance(0.35, 0.562, 0.0, -7.0)

    assert distance is not None
    assert distance > 1.7
