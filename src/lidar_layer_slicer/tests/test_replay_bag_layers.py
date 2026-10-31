"""bag 再生ツールの層定義と画像変換を固定する。bag も ROS グラフも使わない。

bag の読み出し自体は記録済みデータが要るため自動確認の対象にしない。ここで固定するのは、
bag が無くても判定できる引数の解釈と変換だけである。
"""

from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).parents[1]))
sys.path.insert(0, str(Path(__file__).parents[1] / 'tools'))
from replay_bag_layers import build_layer_specs, image_to_bgr, parse_args

from lidar_layer_slicer.layer_core import SOURCE_POINT_CLOUD, SOURCE_SCAN


class _Image:
    """sensor_msgs/Image の最小の代用."""

    def __init__(self, height: int, width: int, encoding: str, data: bytes) -> None:
        self.height = height
        self.width = width
        self.encoding = encoding
        self.data = data


def _args(*extra: str):
    return parse_args(['bag', '--output', 'out', *extra])


def test_layers_are_ordered_from_low_to_high() -> None:
    """層は床上高さの低い順に並び、scan 層が先頭へ来る."""
    specs = build_layer_specs(_args('--layer-heights', '0.50', '0.35'))

    assert [spec.height_m for spec in specs] == [0.25, 0.35, 0.50]
    assert specs[0].source == SOURCE_SCAN
    assert specs[1].source == SOURCE_POINT_CLOUD


def test_skip_cloud_leaves_only_the_scan_layer() -> None:
    """点群を読まない指定では scan 層だけになる."""
    specs = build_layer_specs(_args('--skip-cloud'))

    assert len(specs) == 1
    assert specs[0].source == SOURCE_SCAN


def test_layer_names_encode_height_in_centimetres() -> None:
    """点群層の名前は床上高さを cm で表し、出力ファイルから層を特定できる."""
    specs = build_layer_specs(_args('--layer-heights', '0.42'))

    assert specs[-1].name == 'h042'


def test_rgb8_image_is_converted_to_bgr() -> None:
    """rgb8 は channel 順を入れ替えて BGR として扱う."""
    data = bytes([10, 20, 30])

    image = image_to_bgr(_Image(1, 1, 'rgb8', data))

    assert list(image[0, 0]) == [30, 20, 10]


def test_bgr8_image_is_passed_through() -> None:
    """bgr8 はそのまま扱う."""
    image = image_to_bgr(_Image(1, 1, 'bgr8', bytes([10, 20, 30])))

    assert list(image[0, 0]) == [10, 20, 30]


def test_unsupported_encoding_returns_none() -> None:
    """未対応の encoding は None を返し、カメラ画像を書き出さない."""
    assert image_to_bgr(_Image(1, 1, 'bayer_rggb8', bytes([0]))) is None


def test_yuyv_image_is_converted() -> None:
    """実機カメラの yuyv を BGR へ変換できる."""
    data = np.zeros((2, 2, 2), dtype=np.uint8).tobytes()

    image = image_to_bgr(_Image(2, 2, 'yuyv', data))

    assert image.shape == (2, 2, 3)
