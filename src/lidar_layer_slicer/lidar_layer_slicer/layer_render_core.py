# -*- coding: utf-8 -*-
"""水平スライス層を AI 入力用の画像へ描画する ROS 非依存のコア.

既存 obstacle_monitor の `sensor_viewer` と同じ方針（実距離に比例した画素配置、下端中心を
センサ位置とする原点、白背景・赤点）を踏襲しつつ、距離グリッドと数値ラベル、車体矩形、
進行方向を加える。表示範囲は近傍へ寄せ、縮小は行わない。

画像内の文字は OpenCV の既定フォントで描くため ASCII に限る。寸法は画像とは別に
構造化テキストでも渡すため、制御に使う数値が画像の読み取り精度に依存しない。
"""

from dataclasses import dataclass
from typing import List, Sequence, Tuple

import math

import cv2
import numpy as np

from lidar_layer_slicer.layer_core import ClearanceConfig, LayerObservation

# 描画色 (BGR)
COLOR_BACKGROUND = (255, 255, 255)
COLOR_GRID = (210, 210, 210)
COLOR_AXIS = (120, 120, 120)
COLOR_POINT = (0, 0, 255)
COLOR_ROBOT = (0, 0, 0)
COLOR_CORRIDOR = (0, 165, 255)
COLOR_TEXT = (0, 0, 0)
COLOR_SEPARATOR = (160, 160, 160)


@dataclass(frozen=True)
class ViewConfig:
    """描画範囲と解像度.

    Attributes:
        range_x_m (float): 前方に描く距離 [m].
        range_y_m (float): 左右に描く距離 [m]（片側）.
        pixel_pitch (int): 1 m あたりの画素数 [pix/m].
        grid_step_m (float): グリッド線の間隔 [m].
        point_radius_px (int): 点の描画半径 [pix].
        header_px (int): 層名を書く上端の余白 [pix].
    """

    range_x_m: float = 4.0
    range_y_m: float = 2.0
    pixel_pitch: int = 120
    grid_step_m: float = 1.0
    point_radius_px: int = 3
    header_px: int = 24

    @property
    def width_px(self) -> int:
        """画像の幅 [pix]."""
        return int(round(2.0 * self.range_y_m * self.pixel_pitch))

    @property
    def body_height_px(self) -> int:
        """描画領域の高さ [pix]（上端の余白を含まない）."""
        return int(round(self.range_x_m * self.pixel_pitch))

    @property
    def height_px(self) -> int:
        """画像の高さ [pix]."""
        return self.body_height_px + self.header_px


def to_pixel(x_m: float, y_m: float, view: ViewConfig) -> Tuple[int, int]:
    """base_link 座標 [m] を画素座標へ変換する.

    原点は描画領域の下端中心であり、x 前方が上、y 左が画像左になる。
    """
    px = view.width_px / 2.0 - y_m * view.pixel_pitch
    py = view.header_px + view.body_height_px - x_m * view.pixel_pitch
    return int(round(px)), int(round(py))


def render_layer(observation: LayerObservation, config: ClearanceConfig,
                 view: ViewConfig) -> np.ndarray:
    """1 層を 1 枚の画像へ描画する.

    Args:
        observation (LayerObservation): 層の観測結果.
        config (ClearanceConfig): 車体幅・前方距離などの判定条件.
        view (ViewConfig): 描画範囲と解像度.

    Returns:
        np.ndarray: bgr8 の画像.
    """
    image = np.full((view.height_px, view.width_px, 3), COLOR_BACKGROUND, dtype=np.uint8)

    # 距離グリッドと数値ラベル。AI が画素から距離を推定しなくて済むようにする。
    step = max(float(view.grid_step_m), 0.1)
    distance = step
    while distance <= view.range_x_m + 1e-6:
        _, py = to_pixel(distance, 0.0, view)
        cv2.line(image, (0, py), (view.width_px, py), COLOR_GRID, 1)
        cv2.putText(image, f'{distance:.0f}m', (4, py - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, COLOR_AXIS, 1, cv2.LINE_AA)
        distance += step
    lateral = -view.range_y_m
    while lateral <= view.range_y_m + 1e-6:
        px, _ = to_pixel(0.0, lateral, view)
        cv2.line(image, (px, view.header_px), (px, view.height_px), COLOR_GRID, 1)
        if abs(lateral) > 1e-6:
            label = f'{lateral:+.0f}m'
            cv2.putText(image, label, (px + 2, view.height_px - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, COLOR_AXIS, 1, cv2.LINE_AA)
        lateral += step

    # 直進時に車体が通る帯（車体幅＋余裕）。
    half = config.swath_half_m
    for sign in (-1.0, 1.0):
        px, _ = to_pixel(0.0, sign * half, view)
        cv2.line(image, (px, view.header_px), (px, view.height_px), COLOR_CORRIDOR, 1, cv2.LINE_AA)

    # 評価した前方距離の上限。
    if config.forward_range_m <= view.range_x_m:
        _, py = to_pixel(config.forward_range_m, 0.0, view)
        cv2.line(image, (0, py), (view.width_px, py), COLOR_CORRIDOR, 1, cv2.LINE_AA)

    # 層の点。
    points = observation.points_xy
    if points is not None and len(points) > 0:
        for x, y in np.asarray(points, dtype=np.float32):
            if not (0.0 <= x <= view.range_x_m) or abs(y) > view.range_y_m:
                continue
            px, py = to_pixel(float(x), float(y), view)
            if 0 <= px < view.width_px and view.header_px <= py < view.height_px:
                cv2.circle(image, (px, py), view.point_radius_px, COLOR_POINT, -1)

    # 車体矩形（車軸中心を原点とした半幅）と進行方向。
    body_half = config.robot_width_m / 2.0
    left_px, _ = to_pixel(0.0, body_half, view)
    right_px, _ = to_pixel(0.0, -body_half, view)
    _, bottom_py = to_pixel(0.0, 0.0, view)
    _, top_py = to_pixel(0.3, 0.0, view)
    cv2.rectangle(image, (left_px, top_py), (right_px, bottom_py), COLOR_ROBOT, 2)
    center_px = int(round(view.width_px / 2.0))
    _, arrow_py = to_pixel(0.6, 0.0, view)
    cv2.arrowedLine(image, (center_px, top_py), (center_px, arrow_py),
                    COLOR_ROBOT, 2, cv2.LINE_AA, tipLength=0.3)

    # 層名と高さ。画像単体でもどの層かが分かるようにする。
    spec = observation.spec
    title = (f'{spec.name} h={spec.height_m:.2f}m t={spec.thickness_m:.2f}m '
             f'src={spec.source} n={observation.point_count}')
    cv2.putText(image, title, (4, view.header_px - 7),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, COLOR_TEXT, 1, cv2.LINE_AA)
    return image


def render_layer_set(observations: Sequence[LayerObservation], config: ClearanceConfig,
                     view: ViewConfig) -> np.ndarray:
    """複数層を同一スケールで横へ並べた 1 枚の画像を作る.

    層の対応を取りやすくするため、原点・縮尺・表示範囲を全層で揃える。
    観測が無い場合は注記だけを描いた画像を返す。
    """
    if not observations:
        image = np.full((view.height_px, view.width_px, 3), COLOR_BACKGROUND, dtype=np.uint8)
        cv2.putText(image, 'no layer', (4, view.header_px - 7),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, COLOR_TEXT, 1, cv2.LINE_AA)
        return image
    tiles: List[np.ndarray] = []
    separator = np.full((view.height_px, 2, 3), COLOR_SEPARATOR, dtype=np.uint8)
    for index, observation in enumerate(observations):
        if index > 0:
            tiles.append(separator)
        tiles.append(render_layer(observation, config, view))
    return np.hstack(tiles)


def format_offset(value: float) -> str:
    """オフセット値を画像・ログ向けの文字列へ整える."""
    return 'inf' if not math.isfinite(value) else f'{value:.2f}'
