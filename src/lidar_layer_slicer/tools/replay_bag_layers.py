#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""記録済み rosbag から水平スライス層を再現し、AI 入力の素材を書き出すツール.

手動走行で記録した bag を入力に、前方へ障害物が近づいた場面を抜き出し、その時刻の
3 層画像、カメラ画像、構造化テキストを出力ディレクトリへ保存する。層の高さと厚さの
妥当性確認と、AI へ渡す入力の事前評価に使う。

場面の選定は `/scan` から直接計算した直進帯の最近傍距離で行う。記録時の
obstacle_monitor の設定（前方抽出距離や閉塞しきい値）に依存させないためである。
手動走行では運転者が障害物へ近づく前に避けるため、`front_blocked` を条件にすると
場面がほとんど得られない。

bag の配置はリポジトリ外であり、実行時に引数で与える。既定値は持たない。

使い方:
    python3 tools/replay_bag_layers.py <bag_dir> --output <出力先> [--limit 20]
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple
import argparse
import math
import sys

import cv2
import numpy as np

import rosbag2_py
from rclpy.duration import Duration
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from rosidl_runtime_py.utilities import get_message
from tf2_ros import Buffer

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lidar_layer_slicer.layer_core import (SOURCE_POINT_CLOUD, SOURCE_SCAN,
                                           ClearanceConfig, LayerSpec,
                                           layer_context_lines,
                                           nearest_forward_distance, observe_layer,
                                           scan_to_xy, slice_layer_points)
from lidar_layer_slicer.layer_render_core import ViewConfig, render_layer_set

DEFAULT_LAYER_HEIGHTS_M = (0.35, 0.50)
SCAN_LAYER_HEIGHT_M = 0.25
FLOOR_TO_BASE_LINK_M = 0.148248
BASE_FRAME = 'base_link'
CLOUD_FRAME = 'mid360_frame'

TOPIC_SCAN = '/scan'
TOPIC_LIVOX = '/mid360/livox/lidar'
TOPIC_IMAGE = '/usb_cam/image_raw'
TOPIC_HINT = '/obstacle_avoidance_hint'
TOPIC_CMD_VEL = '/cmd_vel'
TOPIC_TF_STATIC = '/tf_static'


@dataclass
class ReplayState:
    """再生中に保持する最新入力."""

    scan: object = None
    image: Optional[np.ndarray] = None
    hint: object = None
    stopped: bool = False
    cloud_frames: List[Tuple[int, np.ndarray]] = field(default_factory=list)


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('bag', type=Path, help='rosbag のディレクトリ')
    parser.add_argument('--output', type=Path, required=True, help='出力先ディレクトリ')
    parser.add_argument('--limit', type=int, default=20, help='書き出す場面の最大数')
    parser.add_argument('--near-threshold', type=float, default=2.0,
                        help='場面とみなす直進帯の最近傍距離 [m]')
    parser.add_argument('--min-interval-sec', type=float, default=10.0,
                        help='同一の接近を重複して採らないための最小間隔 [s]')
    parser.add_argument('--require-stop', action='store_true',
                        help='停止している場面のみを対象にする')
    parser.add_argument('--speed-threshold', type=float, default=0.05,
                        help='停止とみなす速度 [m/s, rad/s]')
    parser.add_argument('--accumulate-sec', type=float, default=0.3,
                        help='点群を累積する時間幅 [s]')
    parser.add_argument('--layer-heights', type=float, nargs='*', default=None,
                        help='点群層の床上高さ [m]（既定 0.35 0.50）')
    parser.add_argument('--layer-thickness', type=float, default=0.10,
                        help='点群層の厚さ [m]')
    parser.add_argument('--robot-width', type=float, default=0.6, help='車体全幅 [m]')
    parser.add_argument('--side-margin', type=float, default=0.1, help='左右余裕 [m]')
    parser.add_argument('--forward-range', type=float, default=2.0,
                        help='横移動余裕を評価する前方距離 [m]')
    parser.add_argument('--skip-cloud', action='store_true',
                        help='点群を読まず scan 層だけを出力する（高速確認用）')
    return parser.parse_args(argv)


def open_reader(bag_dir: Path) -> Tuple[rosbag2_py.SequentialReader, Dict[str, str]]:
    """bag を開き、topic 名から型名への対応を返す."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=''),
        rosbag2_py.ConverterOptions(input_serialization_format='cdr',
                                    output_serialization_format='cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    return reader, types


def collect_static_transforms(bag_dir: Path, types: Dict[str, str]) -> Buffer:
    """`/tf_static` を読み、記録時の取付姿勢を持つ TF バッファを作る.

    取付角をツールに直書きせず、記録時の値をそのまま使う。
    """
    buffer = Buffer(cache_time=Duration(seconds=3600))
    if TOPIC_TF_STATIC not in types:
        return buffer
    reader, _ = open_reader(bag_dir)
    reader.set_filter(rosbag2_py.StorageFilter(topics=[TOPIC_TF_STATIC]))
    message_type = get_message(types[TOPIC_TF_STATIC])
    while reader.has_next():
        _, data, _ = reader.read_next()
        for transform in deserialize_message(data, message_type).transforms:
            buffer.set_transform_static(transform, 'bag')
    return buffer


def lookup_rigid_transform(buffer: Buffer, target: str, source: str
                           ) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    """静的 TF から回転行列と並進を取り出す."""
    try:
        transform = buffer.lookup_transform(target, source, Time())
    except Exception:
        return None
    q = transform.transform.rotation
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0:
        return None
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    rotation = np.asarray([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ], dtype=np.float32)
    translation = np.asarray([transform.transform.translation.x,
                              transform.transform.translation.y,
                              transform.transform.translation.z], dtype=np.float32)
    return rotation, translation


def build_layer_specs(args: argparse.Namespace) -> List[LayerSpec]:
    """scan 層と点群層の定義を低い順に作る."""
    heights = args.layer_heights if args.layer_heights else list(DEFAULT_LAYER_HEIGHTS_M)
    specs = [LayerSpec('urg', SOURCE_SCAN, SCAN_LAYER_HEIGHT_M, 0.0)]
    if not args.skip_cloud:
        specs += [LayerSpec(f'h{int(round(height * 100)):03d}', SOURCE_POINT_CLOUD,
                            float(height), float(args.layer_thickness))
                  for height in heights]
    return sorted(specs, key=lambda spec: spec.height_m)


def image_to_bgr(message) -> Optional[np.ndarray]:
    """sensor_msgs/Image を BGR の ndarray へ変換する（cv_bridge を使わない）."""
    height, width = int(message.height), int(message.width)
    buffer = np.frombuffer(bytes(message.data), dtype=np.uint8)
    encoding = str(message.encoding)
    if encoding in ('rgb8', 'bgr8'):
        image = buffer.reshape(height, width, 3)
        return image[:, :, ::-1].copy() if encoding == 'rgb8' else image.copy()
    if encoding in ('yuyv', 'yuv422_yuy2'):
        return cv2.cvtColor(buffer.reshape(height, width, 2), cv2.COLOR_YUV2BGR_YUYV)
    if encoding == 'mono8':
        return cv2.cvtColor(buffer.reshape(height, width), cv2.COLOR_GRAY2BGR)
    return None


def scan_points(scan) -> np.ndarray:
    """LaserScan を前方のみの xy 点へ変換する."""
    if scan is None:
        return np.empty((0, 2), dtype=np.float32)
    return scan_to_xy(scan.ranges, float(scan.angle_min), float(scan.angle_increment),
                      float(scan.range_min), float(scan.range_max))


def observe_scene(state: ReplayState, specs: List[LayerSpec],
                  config: ClearanceConfig) -> List:
    """保持している最新入力から各層の観測結果を作る."""
    cloud = (np.concatenate([points for _, points in state.cloud_frames], axis=0)
             if state.cloud_frames else np.empty((0, 3), dtype=np.float32))
    observations = []
    for spec in specs:
        if spec.source == SOURCE_SCAN:
            points = scan_points(state.scan)
        else:
            points = slice_layer_points(cloud, spec, FLOOR_TO_BASE_LINK_M)
        observations.append(observe_layer(points, spec, config))
    return observations


def write_scene(stamp_ns: int, state: ReplayState, observations: List,
                config: ClearanceConfig, view: ViewConfig,
                output: Path) -> List[Path]:
    """1 場面分の層画像・カメラ画像・構造化テキストを書き出す."""
    name = str(stamp_ns)
    written: List[Path] = []

    layer_path = output / f'{name}_layers.png'
    cv2.imwrite(str(layer_path), render_layer_set(observations, config, view))
    written.append(layer_path)

    if state.image is not None:
        camera_path = output / f'{name}_camera.png'
        cv2.imwrite(str(camera_path), state.image)
        written.append(camera_path)

    lines = [f'stamp_ns={stamp_ns}',
             f'stopped={state.stopped}',
             f'accumulated_frames={len(state.cloud_frames)}']
    if state.hint is not None:
        lines += [
            f'hint_front_blocked={bool(state.hint.front_blocked)}',
            f'hint_front_clearance_m={float(state.hint.front_clearance_m):.2f}',
            f'hint_left_offset_m={float(state.hint.left_offset_m):.2f}',
            f'hint_right_offset_m={float(state.hint.right_offset_m):.2f}']
    lines += layer_context_lines(observations, config)
    text_path = output / f'{name}_context.txt'
    text_path.write_text('\n'.join(lines) + '\n', encoding='utf-8')
    written.append(text_path)
    return written


def replay(bag_dir: Path, types: Dict[str, str], buffer: Buffer,
           args: argparse.Namespace) -> Tuple[int, int]:
    """bag を 1 回走査し、条件に合う場面を書き出す.

    Returns:
        Tuple[int, int]: (書き出した場面数, 書き出したファイル数).
    """
    specs = build_layer_specs(args)
    config = ClearanceConfig(forward_range_m=args.forward_range,
                             robot_width_m=args.robot_width,
                             side_margin_m=args.side_margin)
    view = ViewConfig()

    transform = None
    if not args.skip_cloud:
        transform = lookup_rigid_transform(buffer, BASE_FRAME, CLOUD_FRAME)
        if transform is None:
            print(f'警告: {BASE_FRAME} <- {CLOUD_FRAME} の静的 TF が bag に無い。'
                  '点群層を作らない。')

    wanted = [topic for topic in (TOPIC_SCAN, TOPIC_IMAGE, TOPIC_HINT, TOPIC_CMD_VEL)
              if topic in types]
    if transform is not None and TOPIC_LIVOX in types:
        wanted.append(TOPIC_LIVOX)
    reader, _ = open_reader(bag_dir)
    reader.set_filter(rosbag2_py.StorageFilter(topics=wanted))
    message_types = {topic: get_message(types[topic]) for topic in wanted}

    state = ReplayState()
    accumulate_ns = int(args.accumulate_sec * 1e9)
    interval_ns = int(args.min_interval_sec * 1e9)
    last_written_ns = -(10 ** 18)
    scenes = 0
    files = 0
    args.output.mkdir(parents=True, exist_ok=True)

    while reader.has_next() and scenes < args.limit:
        topic, data, stamp_ns = reader.read_next()
        if topic == TOPIC_IMAGE:
            state.image = image_to_bgr(deserialize_message(data, message_types[topic]))
            continue
        if topic == TOPIC_HINT:
            state.hint = deserialize_message(data, message_types[topic])
            continue
        if topic == TOPIC_CMD_VEL:
            twist = deserialize_message(data, message_types[topic])
            state.stopped = (abs(twist.linear.x) < args.speed_threshold
                             and abs(twist.angular.z) < args.speed_threshold)
            continue
        if topic == TOPIC_LIVOX:
            message = deserialize_message(data, message_types[topic])
            points = np.asarray([(p.x, p.y, p.z) for p in message.points],
                                dtype=np.float32)
            if points.size:
                rotation, translation = transform
                state.cloud_frames.append((stamp_ns, points @ rotation.T + translation))
            state.cloud_frames = [(t, p) for t, p in state.cloud_frames
                                  if stamp_ns - t <= accumulate_ns]
            continue

        # ここから下は /scan 受信時のみ。場面の判定は scan の最近傍距離で行う。
        state.scan = deserialize_message(data, message_types[topic])
        if stamp_ns - last_written_ns < interval_ns:
            continue
        if args.require_stop and not state.stopped:
            continue
        nearest = nearest_forward_distance(scan_points(state.scan), config)
        if not (nearest <= args.near_threshold):
            continue
        observations = observe_scene(state, specs, config)
        files += len(write_scene(stamp_ns, state, observations, config, view, args.output))
        scenes += 1
        last_written_ns = stamp_ns
    return scenes, files


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    if not args.bag.exists():
        print(f'bag が見つからない: {args.bag}')
        return 1
    _, types = open_reader(args.bag)
    if TOPIC_SCAN not in types:
        print(f'{TOPIC_SCAN} が bag に無いため場面を選べない。')
        return 1
    buffer = collect_static_transforms(args.bag, types)
    scenes, files = replay(args.bag, types, buffer, args)
    if scenes == 0:
        print('条件に合う場面が見つからなかった。'
              '--near-threshold を広げるか --require-stop を外す。')
        return 0
    print(f'{scenes} 場面、{files} ファイルを {args.output} へ書き出した。')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
