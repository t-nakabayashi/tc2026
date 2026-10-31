#!/usr/bin/env python3
"""topURG の水平走査から、鉛直な境界物の分布を地図座標で作る.

URG は床から一定の高さを水平に走査するため、路面そのものを観測しない。
生垣・壁・柱・車止め・ロードコーンのような鉛直物だけが写るので、
点群の路面判定に混入する微小な凹凸の影響を受けない境界の手掛かりになる。

走査の配置には地図作成と同じ最適化済みキーフレーム姿勢を使う。経路の waypoint は
別の推定器で記録されており地図と中央 0.2 m 程度ずれるため、重畳の基準には使わない。
物体の種別は判定しない。高さ方向の情報も持たないため、人や車も同じように写る。
"""
import argparse
import json
import math
from pathlib import Path
import sqlite3

import numpy as np
from scipy.signal import fftconvolve
from scipy.spatial.transform import Rotation
from scipy.spatial import cKDTree
import yaml


def base_poses(poses: Path, keyframes: Path, hardware: Path) -> tuple:
    """キーフレームの最適化姿勢を、車体基準の位置・方位・時刻に直す.

    Args:
        poses (Path): optimized_poses_enu.npy.
        keyframes (Path): keyframes.json.
        hardware (Path): 取付設定の hardware.yaml.

    Returns:
        tuple: (時刻, 位置XY, 方位, 記録区間, URGの車体内オフセットXY).
    """
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'icart_mapping'))
    from icart_mapping.mount import geometry
    config = yaml.safe_load(hardware.read_text())
    geo = geometry(config)
    base_rotation = Rotation.from_euler('xyz', geo['rpy_deg'], degrees=True).as_matrix()
    imu_to_base = -base_rotation.T@np.array(geo['imu'])
    matrices = np.load(poses)
    frames = json.loads(keyframes.read_text())
    stamp = np.array([float(f['t']) for f in frames])
    segment = np.array([int(f['segment']) for f in frames])
    position = matrices[:, :3, 3]+np.einsum('nij,j->ni', matrices[:, :3, :3], imu_to_base)
    heading = np.einsum('nij,jk->nik', matrices[:, :3, :3], base_rotation.T)
    yaw = np.arctan2(heading[:, 1, 0], heading[:, 0, 0])
    urg = np.array(config['urg']['xyz'][:2], dtype=float)
    order = np.argsort(stamp)
    return stamp[order], position[order, :2], yaw[order], segment[order], urg


def pose_at(stamp: np.ndarray, position: np.ndarray, yaw: np.ndarray, segment: np.ndarray,
            query: float, max_gap: float) -> tuple | None:
    """指定時刻の車体位置・方位を、前後のキーフレームから内挿する.

    Args:
        stamp (np.ndarray): キーフレーム時刻.
        position (np.ndarray): キーフレーム位置XY.
        yaw (np.ndarray): キーフレーム方位.
        segment (np.ndarray): 記録区間番号.
        query (float): 求めたい時刻.
        max_gap (float): 内挿を許す前後キーフレームの時間差 [s].

    Returns:
        tuple | None: (位置XY, 方位)。区間をまたぐ場合や時間差が大きい場合は None.
    """
    j = int(np.searchsorted(stamp, query))
    if j <= 0 or j >= len(stamp):
        return None
    if segment[j-1] != segment[j]:
        return None
    span = stamp[j]-stamp[j-1]
    if span <= 0 or span > max_gap:
        return None
    ratio = (query-stamp[j-1])/span
    point = position[j-1]+(position[j]-position[j-1])*ratio
    delta = math.atan2(math.sin(yaw[j]-yaw[j-1]), math.cos(yaw[j]-yaw[j-1]))
    return point, yaw[j-1]+delta*ratio


def iter_scans(bag: Path, topic: str, stride: int, min_range: float):
    """bag から LaserScan を順に読み出す.

    Args:
        bag (Path): bagディレクトリ.
        topic (str): 走査トピック名.
        stride (int): 何件に1件を使うか.
        min_range (float): これより近い反射を捨てる距離 [m]。車体自身の構造物を除く。

    Yields:
        tuple: (時刻, 角度配列, 距離配列).
    """
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import LaserScan
    meta = yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    count = 0
    for entry in sorted(meta['files'], key=lambda f: f['starting_time']['nanoseconds_since_epoch']):
        uri = 'file:'+str(bag/entry['path'])+'?mode=ro'
        with sqlite3.connect(uri, uri=True) as con:
            found = con.execute('select id from topics where name=?', (topic,)).fetchone()
            if not found:
                continue
            for _, blob in con.execute(
                    'select timestamp,data from messages where topic_id=? order by timestamp',
                    (found[0],)):
                count += 1
                if count % stride:
                    continue
                msg = deserialize_message(blob, LaserScan)
                ranges = np.asarray(msg.ranges, dtype=float)
                angles = msg.angle_min+np.arange(len(ranges))*msg.angle_increment
                valid = (np.isfinite(ranges)&(ranges > max(msg.range_min, min_range))
                         &(ranges < msg.range_max))
                yield (msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                       angles[valid], ranges[valid])


def observation_ratio(hits: np.ndarray, origins: np.ndarray, resolution: float,
                      max_range: float, min_views: int) -> np.ndarray:
    """当たり数を、その場所を見られた走査数で正規化する.

    当たり数そのものは走査の密度に比例するため、場所ごとの比較に使えない。
    走査原点の分布を到達半径の円板で畳み込み、各格子を見られた走査数の概算とする。
    畳み込みは 1 m 格子で行う。可視性は滑らかに変化するため、この粗さで足りる。

    Args:
        hits (np.ndarray): 格子ごとの当たり数.
        origins (np.ndarray): 格子ごとの走査原点の数.
        resolution (float): 格子の分解能 [m].
        max_range (float): 走査の到達半径 [m].
        min_views (int): これ未満の可視走査数の格子は 0 とする.

    Returns:
        np.ndarray: 当たり数 / 可視走査数.
    """
    factor = int(round(1./resolution))
    ny, nx = hits.shape
    ny2, nx2 = ny//factor, nx//factor
    coarse = origins[:ny2*factor, :nx2*factor].reshape(ny2, factor, nx2, factor).sum((1, 3))
    radius = int(round(max_range))
    grid_y, grid_x = np.mgrid[-radius:radius+1, -radius:radius+1]
    disk = ((grid_y**2+grid_x**2) <= radius**2).astype(np.float32)
    views = fftconvolve(coarse.astype(np.float32), disk, mode='same')
    views = np.repeat(np.repeat(views, factor, axis=0), factor, axis=1)[:ny, :nx]
    return np.where(views >= min_views, hits/np.maximum(views, 1.), 0.).astype(np.float32)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--poses', type=Path, required=True)
    parser.add_argument('--keyframes', type=Path, required=True)
    parser.add_argument('--hardware', type=Path, required=True)
    parser.add_argument('--surface-mask', type=Path, required=True,
                        help='格子の範囲を揃えるために参照する')
    parser.add_argument('--run-cache', type=Path, required=True)
    parser.add_argument('--bag-root', type=Path, action='append', required=True)
    parser.add_argument('--waypoints', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--topic', default='/scan')
    parser.add_argument('--resolution', type=float, default=.1, help='出力格子の分解能 [m]')
    parser.add_argument('--scan-stride', type=int, default=5, help='何件に1件の走査を読むか')
    parser.add_argument('--pose-stride', type=float, default=.25,
                        help='前に使った走査から車体がこの距離だけ動くまで次を使わない [m]。'
                             '停止中に周囲へ留まる人が同じ格子へ集中することを避ける。')
    parser.add_argument('--max-range', type=float, default=12., help='使用する最大距離 [m]')
    parser.add_argument('--min-range', type=float, default=.5,
                        help='これより近い反射を捨てる [m]。車体前方の構造物が常時写るため。')
    parser.add_argument('--route-margin', type=float, default=10.,
                        help='経路からこの距離までの点だけを残す [m]')
    parser.add_argument('--max-gap', type=float, default=3., help='姿勢内挿を許す時間差 [s]')
    parser.add_argument('--min-views', type=int, default=10,
                        help='正規化に必要な可視走査数の下限')
    args = parser.parse_args()

    stamp, position, yaw, segment, urg = base_poses(args.poses, args.keyframes, args.hardware)
    mask = np.load(args.surface_mask)
    x0, x1, y0, y1 = [float(v) for v in mask['extent']]
    nx = int(round((x1-x0)/args.resolution))
    ny = int(round((y1-y0)/args.resolution))
    hits = np.zeros((ny, nx), np.uint32)
    views = np.zeros((ny, nx), np.uint32)
    route = np.array([[float(r['x']), float(r['y'])]
                      for r in json.loads(args.waypoints.read_text())])
    tree = cKDTree(route)

    bags = {}
    for summary in sorted(args.run_cache.glob('run*/summary.json')):
        info = json.loads(summary.read_text())
        name = Path(info['bag']).name
        found = next((root/name for root in args.bag_root if (root/name).is_dir()), None)
        if found is None:
            print(f'bag未検出のため記録区間{info["run"]}は使用しない: {name}', flush=True)
            continue
        bags[int(info['run'])] = found

    used = skipped = stationary = 0
    for run in sorted(bags):
        last = None
        for t, angles, ranges in iter_scans(bags[run], args.topic, args.scan_stride,
                                            args.min_range):
            pose = pose_at(stamp, position, yaw, segment, t, args.max_gap)
            if pose is None:
                skipped += 1
                continue
            origin, heading = pose
            # 走行距離で間引く。時間で間引くと停止地点の観測だけが積み上がる。
            if last is not None and float(np.hypot(*(origin-last))) < args.pose_stride:
                stationary += 1
                continue
            last = origin.copy()
            keep = ranges < args.max_range
            if not keep.any():
                continue
            local = np.column_stack([np.cos(angles[keep])*ranges[keep],
                                     np.sin(angles[keep])*ranges[keep]])+urg
            cos_h, sin_h = math.cos(heading), math.sin(heading)
            world = np.column_stack([origin[0]+local[:, 0]*cos_h-local[:, 1]*sin_h,
                                     origin[1]+local[:, 0]*sin_h+local[:, 1]*cos_h])
            near = tree.query(world, distance_upper_bound=args.route_margin)[0]
            world = world[np.isfinite(near)]
            if len(world):
                ix = ((world[:, 0]-x0)/args.resolution).astype(int)
                iy = ((world[:, 1]-y0)/args.resolution).astype(int)
                inside = (ix >= 0)&(ix < nx)&(iy >= 0)&(iy < ny)
                np.add.at(hits, (iy[inside], ix[inside]), 1)
            ox = int((origin[0]-x0)/args.resolution)
            oy = int((origin[1]-y0)/args.resolution)
            if 0 <= ox < nx and 0 <= oy < ny:
                views[oy, ox] += 1
            used += 1
            if used % 2000 == 0:
                print(f'run{run} 使用 {used} 走査', flush=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    ratio = observation_ratio(hits.astype(np.float32), views.astype(np.float32),
                              args.resolution, args.max_range, args.min_views)
    np.savez_compressed(args.output, hits=hits, origins=views, ratio=ratio,
                        extent=np.array([x0, x1, y0, y1]),
                        resolution=np.array(args.resolution),
                        scan_stride=np.array(args.scan_stride),
                        pose_stride=np.array(args.pose_stride),
                        max_range=np.array(args.max_range),
                        min_range=np.array(args.min_range),
                        urg_height_m=np.array(float(
                            yaml.safe_load(args.hardware.read_text())['urg']['xyz'][2])))
    print(f'DONE 使用 {used} 走査 / 姿勢不明 {skipped} / 停止中 {stationary} -> {args.output}',
          flush=True)


if __name__ == '__main__':
    main()
