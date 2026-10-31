"""AI 入力用画像の座標写像と構成を固定する。ROS グラフも画面表示も使わない。

画像は実距離に比例した画素配置で、原点は描画領域の下端中心である。縮小は行わない。
"""

from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
from lidar_layer_slicer.layer_core import (SOURCE_POINT_CLOUD, ClearanceConfig,
                                           LayerSpec, observe_layer)
from lidar_layer_slicer.layer_render_core import (ViewConfig, render_layer,
                                                  render_layer_set, to_pixel)

CONFIG = ClearanceConfig(forward_range_m=2.0, robot_width_m=0.6, side_margin_m=0.1,
                         max_offset_m=3.0, near_x_m=0.15)
VIEW = ViewConfig(range_x_m=4.0, range_y_m=2.0, pixel_pitch=120)
SPEC = LayerSpec(name='mid', source=SOURCE_POINT_CLOUD, height_m=0.5, thickness_m=0.1)


def test_origin_maps_to_bottom_center_of_drawing_area() -> None:
    """base_link 原点は描画領域の下端中心へ写る."""
    px, py = to_pixel(0.0, 0.0, VIEW)

    assert px == VIEW.width_px // 2
    assert py == VIEW.height_px


def test_forward_and_left_directions_map_to_up_and_left() -> None:
    """x 前方は画像の上、y 左は画像の左へ写り、画素数は距離に比例する."""
    px_front, py_front = to_pixel(1.0, 0.0, VIEW)
    px_left, _ = to_pixel(0.0, 1.0, VIEW)

    assert py_front == VIEW.height_px - VIEW.pixel_pitch
    assert px_front == VIEW.width_px // 2
    assert px_left == VIEW.width_px // 2 - VIEW.pixel_pitch


def test_rendered_layer_has_expected_size_and_is_not_resized() -> None:
    """1 層の画像は表示範囲と画素密度から決まる大きさを持つ."""
    observation = observe_layer(np.asarray([[1.0, 0.0]], dtype=np.float32), SPEC, CONFIG)

    image = render_layer(observation, CONFIG, VIEW)

    assert image.shape == (VIEW.height_px, VIEW.width_px, 3)
    assert image.dtype == np.uint8


def test_point_is_drawn_in_red_at_its_distance() -> None:
    """点は実距離に対応する画素位置へ赤で描かれる."""
    observation = observe_layer(np.asarray([[1.0, 0.5]], dtype=np.float32), SPEC, CONFIG)

    image = render_layer(observation, CONFIG, VIEW)
    px, py = to_pixel(1.0, 0.5, VIEW)

    assert tuple(int(v) for v in image[py, px]) == (0, 0, 255)


def test_points_outside_the_view_are_clipped() -> None:
    """表示範囲の外にある点は描画されず、例外にもならない."""
    observation = observe_layer(np.asarray([[9.0, 9.0]], dtype=np.float32), SPEC, CONFIG)

    image = render_layer(observation, CONFIG, VIEW)

    assert not np.any(np.all(image == np.asarray([0, 0, 255], dtype=np.uint8), axis=2))


def test_layer_set_is_combined_side_by_side_with_separators() -> None:
    """複数層は同一スケールで横へ並び、間に区切り線が入る."""
    observation = observe_layer(np.empty((0, 2), dtype=np.float32), SPEC, CONFIG)

    image = render_layer_set([observation, observation, observation], CONFIG, VIEW)

    assert image.shape[0] == VIEW.height_px
    assert image.shape[1] == VIEW.width_px * 3 + 2 * 2


def test_empty_layer_set_returns_single_annotated_tile() -> None:
    """層が無い場合も画像を返す（配信を止めない）."""
    image = render_layer_set([], CONFIG, VIEW)

    assert image.shape == (VIEW.height_px, VIEW.width_px, 3)
