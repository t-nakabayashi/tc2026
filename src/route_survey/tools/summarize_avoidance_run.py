#!/usr/bin/env python3
"""走行記録から、横ずれが waypoint の回避幅に収まっているかを集計する.

回避幅は追従側の横移動量を制限する値であり、記録された経路からの横ずれと突き合わせれば
値が実際の走行をどう縛ったかが分かる。経路からの横ずれは、最寄りの waypoint ではなく
弧長で内挿した経路上の点と、その点での接線の法線で求める。判定や幅の更新は行わない。
"""
import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

import rclpy  # noqa: F401  rosbag2_py がROSの型支援を使うため先に読み込む
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
from geo_pose_converter.geo_core import LlhPoint, llh_to_enu, load_projection_config_from_yaml


def read_topics(bag: Path, topics: set[str]) -> dict[str, list]:
    """bagから指定トピックのメッセージを時刻付きで取り出す.

    Args:
        bag (Path): rosbag2のディレクトリ.
        topics (set[str]): 取り出すトピック名.

    Returns:
        dict[str, list]: トピック名から (時刻[s], メッセージ) の列への対応.
    """
    # 記録が中断され metadata.yaml が残らない場合に備え、mcap を直接開けるようにする
    storage = ''
    if bag.is_dir() and not (bag/'metadata.yaml').is_file():
        found = sorted(bag.glob('*.mcap'))
        if not found:
            raise SystemExit(f'記録が読めません: {bag}')
        bag, storage = found[0], 'mcap'
    elif bag.suffix == '.mcap':
        storage = 'mcap'
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=storage),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    out = {name: [] for name in topics if name in types}
    while reader.has_next():
        name, data, stamp = reader.read_next()
        if name in out:
            out[name].append((stamp*1e-9, deserialize_message(data, get_message(types[name]))))
    return out


class RouteFrame:
    """経路を弧長で引き、任意の位置を縦方向の進みと横ずれに分解する.

    経路は走行時と同じ投影設定で緯度経度から ENU へ変換する。別の原点で作った ENU を
    混ぜると、記録した姿勢との間に原点差がそのまま横ずれとして出る。
    """

    def __init__(self, route: Path, projection: Path) -> None:
        config = load_projection_config_from_yaml(str(projection))
        rows = list(csv.DictReader(route.open()))
        points = []
        for row in rows:
            enu = llh_to_enu(LlhPoint(float(row['latitude']), float(row['longitude']), 0.),
                             config)
            points.append([enu.x, enu.y])
        self.point = np.array(points)
        step = np.linalg.norm(np.diff(self.point, axis=0), axis=1)
        self.distance = np.concatenate([[0.], np.cumsum(step)])
        delta = np.diff(self.point, axis=0)
        self.segment = np.vstack([delta, delta[-1:]])
        self.length2 = np.maximum((self.segment**2).sum(axis=1), 1e-9)

    def project(self, x: float, y: float, near: int = None,
                window: int = 200) -> tuple[float, float, int]:
        """位置を経路へ投影し、始点からの距離と左向きを正とする横ずれを返す.

        往路と復路が近接する区間では、経路全体から最寄りを選ぶと別の足へ飛ぶ。
        直前の点が分かる場合は、その周辺だけを探す。

        Args:
            x (float): ENU X [m].
            y (float): ENU Y [m].
            near (int | None): 直前に対応した waypoint の番号.
            window (int): near の前後に探す waypoint 数.

        Returns:
            tuple[float, float, int]: 始点からの距離 [m]、横ずれ [m]（左が正）、
                対応した waypoint の番号.
        """
        here = np.array([x, y])
        low, high = 0, len(self.point)
        if near is not None:
            low, high = max(0, near-window), min(len(self.point), near+window)
        point = self.point[low:high]
        segment = self.segment[low:high]
        length2 = self.length2[low:high]
        ratio = np.clip(((here-point)*segment).sum(axis=1)/length2, 0., 1.)
        foot = point+segment*ratio[:, None]
        index = int(np.argmin(((foot-here)**2).sum(axis=1)))
        tangent = segment[index]/math.sqrt(length2[index])
        offset = here-foot[index]
        lateral = float(-tangent[1]*offset[0]+tangent[0]*offset[1])
        step = float(np.linalg.norm(segment[index])*ratio[index])
        return float(self.distance[low+index]+step), lateral, low+index


def load_widths(route: Path) -> tuple[np.ndarray, np.ndarray]:
    """走行用のwaypointリストから左右の回避幅を取り出す.

    Args:
        route (Path): waypoints.csv のパス.

    Returns:
        tuple[np.ndarray, np.ndarray]: 左の幅[m]、右の幅[m].
    """
    rows = list(csv.DictReader(route.open()))
    return (np.array([float(r['left_is_open']) for r in rows]),
            np.array([float(r['right_is_open']) for r in rows]))


def main() -> None:
    """走行記録を読み、進捗・回避の判断・幅超過をまとめて出す.

    Returns:
        None
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--route', type=Path, required=True,
                        help='回避幅を書き込んだ waypoints.csv')
    parser.add_argument('--projection', type=Path, required=True,
                        help='走行時に使った projection.yaml')
    parser.add_argument('--output', type=Path, required=True, help='距離ごとの集計CSV')
    parser.add_argument('--pose-topic', default='/localization/pose_enu')
    parser.add_argument('--max-gap', type=float, default=30.,
                        help='この秒数を超える記録の途切れで打ち切る。1つの記録に複数回の'
                             '走行が入っている場合に、最初の走行だけを見る')
    args = parser.parse_args()

    frame = RouteFrame(args.route, args.projection)
    left, right = load_widths(args.route)
    data = read_topics(args.bag, {args.pose_topic, '/manager_status', '/follower_state',
                                  '/body_contacts'})
    poses = data.get(args.pose_topic, [])
    if not poses:
        raise SystemExit(f'姿勢が記録されていません: {args.pose_topic}')
    for position, ((previous, _), (stamp, _)) in enumerate(zip(poses, poses[1:]), start=1):
        if stamp-previous > args.max_gap:
            print(f'記録が{stamp-previous:.0f}s途切れたため、ここまでを1回の走行として扱う')
            poses = poses[:position]
            limit_time = previous
            break
    else:
        limit_time = poses[-1][0]
    data = {name: [(stamp, message) for stamp, message in rows if stamp <= limit_time]
            for name, rows in data.items()}

    track = []
    near = None
    for stamp, message in poses:
        position = message.pose.pose.position
        distance, lateral, index = frame.project(position.x, position.y, near)
        near = index
        limit = left[index] if lateral >= 0 else right[index]
        track.append((stamp, distance, lateral, limit, index))

    status = data.get('/manager_status', [])
    decisions = {}
    for stamp, message in status:
        key = (message.state, message.decision, message.last_cause)
        decisions.setdefault(key, []).append(stamp)
    # 接触トピックは構成によっては配信されない。記録が無いことを0件と書かない
    contact_messages = data.get('/body_contacts')
    contacts = (None if not contact_messages else
                sum(1 for _, message in contact_messages if getattr(message, 'contacts', [])))

    start = track[0][0]
    with args.output.open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['time_s', 'distance_m', 'lateral_m', 'limit_m', 'waypoint_index',
                         'over_limit_m'])
        for stamp, distance, lateral, limit, index in track:
            writer.writerow([f'{stamp-start:.2f}', f'{distance:.2f}', f'{lateral:.3f}',
                             f'{limit:.2f}', index, f'{max(0., abs(lateral)-limit):.3f}'])

    over = np.array([max(0., abs(t[2])-t[3]) for t in track])
    lateral = np.array([abs(t[2]) for t in track])
    reach = max(t[1] for t in track)
    print(f'記録 {len(track)} 点 / 経過 {track[-1][0]-start:.0f} s / 到達 {reach:.0f} m')
    print(f'横ずれ 中央 {np.median(lateral):.2f} m / p95 {np.percentile(lateral, 95):.2f} m '
          f'/ 最大 {lateral.max():.2f} m')
    print(f'幅の超過 {int((over > 0).sum())} 点 ({100*(over > 0).mean():.1f}%) / '
          f'最大 {over.max():.2f} m')
    print('接触 記録なし（この構成では配信されない）' if contacts is None
          else f'接触 {contacts} 件')
    print('route_manager の状態:')
    for (state, decision, cause), stamps in sorted(decisions.items(), key=lambda kv: -len(kv[1])):
        print(f'  state={state} decision={decision} cause={cause or "-"}: {len(stamps)} 回')
    stagnation = {}
    for _, message in data.get('/follower_state', []):
        if message.last_stagnation_reason:
            stagnation[message.last_stagnation_reason] = \
                stagnation.get(message.last_stagnation_reason, 0)+1
    if stagnation:
        print('追従側の滞留理由:')
        for reason, count in sorted(stagnation.items(), key=lambda kv: -kv[1]):
            print(f'  {reason}: {count} 回')
    print(f'出力: {args.output}')


if __name__ == '__main__':
    main()
