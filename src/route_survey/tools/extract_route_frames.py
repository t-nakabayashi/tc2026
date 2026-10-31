#!/usr/bin/env python3
"""経路距離を指定して手動走行bagから前方カメラのコマを取り出す（確認資料の素材作成）.

前方カメラは車体の真横を写さないため、距離 s の断面を見るには s より手前で
撮られたコマを使う。手前量は --lead で指定し、複数指定すると比較用に並べて出す。
走行・制御は行わず、bagを読取り専用で開くだけである。
"""
import argparse
import json
from pathlib import Path
import sqlite3

import numpy as np
import cv2
from PIL import Image
import yaml


def load_samples(waypoints: Path) -> list[dict]:
    """採取済みwaypointから距離・時刻・記録区間の対応表を作る.

    Args:
        waypoints (Path): route_survey が出力した waypoints_enu.json.

    Returns:
        list[dict]: 距離昇順の {distance, time, run, x, y, yaw} の一覧.
    """
    rows = json.loads(waypoints.read_text())
    out = [dict(distance=float(r['distance_from_start_m']), time=float(r['source_time']),
                run=int(r['source_run']), x=float(r['x']), y=float(r['y']),
                yaw=float(r['yaw']), label=str(r['label'])) for r in rows
           if r.get('source_time') is not None and r.get('source_run') is not None]
    out.sort(key=lambda r: r['distance'])
    return out


def resolve_bags(run_cache: Path, bag_roots: list[Path]) -> dict[int, Path]:
    """記録区間番号から、手元のbagディレクトリへの対応を作る.

    summary.json に記録された元のbagパスは採取時の媒体を指すため、名前だけを使い、
    指定された探索先から同名のディレクトリを探す。見つからない記録区間は、
    実際に必要になった時点で誤りとして扱う。

    Args:
        run_cache (Path): run<N>/summary.json を含むディレクトリ.
        bag_roots (list[Path]): bagディレクトリ群を置いた場所の一覧.

    Returns:
        dict[int, Path]: 記録区間番号から bag ディレクトリへの対応.
    """
    bags = {}
    for summary in sorted(run_cache.glob('run*/summary.json')):
        info = json.loads(summary.read_text())
        name = Path(info['bag']).name
        found = next((root/name for root in bag_roots if (root/name).is_dir()), None)
        if found is not None:
            bags[int(info['run'])] = found
        else:
            print(f'bag未検出のため記録区間{info["run"]}は使用できない: {name}', flush=True)
    if not bags:
        raise FileNotFoundError(f'使用できるbagが無い: {run_cache}')
    return bags


def nearest_frame(bag: Path, topic: str, stamp: float, window: float) -> tuple[float, bytes, dict]:
    """指定時刻に最も近い画像メッセージを1件取り出す.

    Args:
        bag (Path): bagディレクトリ.
        topic (str): 画像トピック名.
        stamp (float): 目標時刻 [s].
        window (float): 前後の探索幅 [s].

    Returns:
        tuple[float, bytes, dict]: ヘッダ時刻、生データ、画像の寸法情報.
    """
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image as RosImage
    meta = yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    best = None
    for entry in meta['files']:
        start = entry['starting_time']['nanoseconds_since_epoch']/1e9
        end = start+entry['duration']['nanoseconds']/1e9
        if not start-window < stamp < end+window:
            continue
        uri = 'file:'+str(bag/entry['path'])+'?mode=ro'
        with sqlite3.connect(uri, uri=True) as con:
            found = con.execute('select id from topics where name=?', (topic,)).fetchone()
            if not found:
                continue
            rows = con.execute(
                'select timestamp,data from messages where topic_id=? and timestamp between ? and ?'
                ' order by timestamp', (found[0], int((stamp-window)*1e9), int((stamp+window)*1e9)))
            for _, blob in rows:
                msg = deserialize_message(blob, RosImage)
                t = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
                if best is None or abs(t-stamp) < abs(best[0]-stamp):
                    best = (t, bytes(msg.data),
                            dict(width=msg.width, height=msg.height, step=msg.step,
                                 encoding=msg.encoding))
    if best is None:
        raise LookupError(f'{topic} のコマが見つからない: {bag.name} {stamp:.2f}')
    return best


def to_rgb(data: bytes, info: dict) -> np.ndarray:
    """記録された画像データをRGB配列へ変換する.

    Args:
        data (bytes): メッセージの画素データ.
        info (dict): width・height・step・encoding.

    Returns:
        np.ndarray: RGB配列.
    """
    buf = np.frombuffer(data, np.uint8).reshape(info['height'], info['step'])
    if info['encoding'] == 'yuv422_yuy2':
        yuy2 = buf[:, :info['width']*2].reshape(info['height'], info['width'], 2)
        return cv2.cvtColor(yuy2, cv2.COLOR_YUV2RGB_YUY2)
    if info['encoding'] in ('rgb8', 'bgr8'):
        rgb = buf[:, :info['width']*3].reshape(info['height'], info['width'], 3)
        return rgb if info['encoding'] == 'rgb8' else rgb[:, :, ::-1]
    raise ValueError(f'未対応のencoding: {info["encoding"]}')


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--waypoints', type=Path, required=True)
    parser.add_argument('--run-cache', type=Path, required=True)
    parser.add_argument('--bag-root', type=Path, action='append', required=True,
                        help='bagディレクトリ群の場所。複数指定できる。')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--topic', default='/usb_cam/image_raw')
    parser.add_argument('--spacing', type=float, default=20.)
    parser.add_argument('--lead', type=float, action='append', default=None,
                        help='断面より何m手前のコマを使うか。複数指定で比較用に全て出す。')
    parser.add_argument('--distances', type=str, default=None,
                        help='カンマ区切りの対象距離[m]。指定時は--spacingより優先する。')
    parser.add_argument('--window', type=float, default=3.)
    parser.add_argument('--quality', type=int, default=92)
    args = parser.parse_args()

    leads = args.lead if args.lead else [8.]
    samples = load_samples(args.waypoints)
    bags = resolve_bags(args.run_cache, args.bag_root)
    total = samples[-1]['distance']
    if args.distances:
        targets = [float(v) for v in args.distances.split(',')]
    else:
        targets = list(np.arange(0., total+1e-9, args.spacing))
    args.output.mkdir(parents=True, exist_ok=True)
    records = []
    for target in targets:
        for lead in leads:
            want = target-lead
            if want < 0:
                continue
            usable = [r for r in samples if r['run'] in bags]
            pick = min(usable, key=lambda r: abs(r['distance']-want))
            if abs(pick['distance']-want) > 2.:
                print(f'skip {target:.0f} m lead {lead:.0f} m: 近傍waypointなし', flush=True)
                continue
            stamp, data, info = nearest_frame(bags[pick['run']], args.topic, pick['time'],
                                              args.window)
            name = f'd{target:05.0f}_lead{lead:04.1f}.jpg'
            Image.fromarray(to_rgb(data, info)).save(args.output/name, quality=args.quality)
            records.append(dict(target_distance_m=target, lead_m=lead, name=name,
                                camera_distance_m=pick['distance'], label=pick['label'],
                                run=pick['run'], requested_time=pick['time'],
                                image_time=stamp, delta_s=stamp-pick['time'],
                                x=pick['x'], y=pick['y'], yaw=pick['yaw']))
            print(f'{target:7.0f} m lead {lead:4.1f} m -> {name} dt {stamp-pick["time"]:+.2f} s',
                  flush=True)
    (args.output/'frames.json').write_text(json.dumps(records, ensure_ascii=False, indent=2))
    print(f'DONE {len(records)} frames -> {args.output}', flush=True)


if __name__ == '__main__':
    main()
