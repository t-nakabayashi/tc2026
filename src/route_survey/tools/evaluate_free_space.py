#!/usr/bin/env python3
"""Read-only ROS bag replay and synthetic closed-loop local-space experiment.

No ROS node is created and no velocity is published. See docs/空き空間参照サンプル.md.
"""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import sqlite3
import time

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation, Slerp

from route_survey.free_space_core import (
    DROP, GROUND, MATERIAL, OBSTACLE, SpaceConfig, arc, build_space,
    choose_path, space_from_labels,
)
from gnss_lio_fusion.gravity_core import GravityAlignment
from gnss_lio_fusion.mount_core import rotation


def write_json(path, data):
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False, allow_nan=False)+'\n')


def read_bag(bag, output):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from sensor_msgs_py.point_cloud2 import read_points
    import yaml
    import cv2
    meta = yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    start = meta['starting_time']['nanoseconds_since_epoch']*1e-9
    cache = output/'cache'
    cache.mkdir()
    rates = {'/lio/odometry': .04, '/mid360/livox/imu': .015,
             '/ypspur_ros/odom': .04, '/localization/pose_enu': .1,
             '/cloud_registered_body': .19, '/scan': .09,
             '/follower_state': .1, '/fusion/gnss_dropout_active': .1,
             '/active_route': 1000., '/usb_cam/camera_info': 1000.,
             '/usb_cam/image_raw': .19}
    series = {key: [] for key in ('lio', 'imu', 'wheel', 'pose', 'follower', 'dropout')}
    frames, scans, cameras, camera_done = [], [], [], set()
    last, types, route, camera_info = {}, {}, [], {}
    for bag_file in meta['relative_file_paths']:
        connection = sqlite3.connect('file:'+str(bag/bag_file)+'?mode=ro', uri=True)
        topics = {i: (n, t) for i, n, t in connection.execute('select id,name,type from topics') if n in rates}
        query = 'select topic_id,timestamp,data from messages where topic_id in ('+','.join('?' for _ in topics)+') order by timestamp'
        for tid, received, blob in connection.execute(query, tuple(topics)):
            topic, kind = topics[tid]
            t = received*1e-9-start
            if t-last.get(topic, -10000) < rates[topic]:
                continue
            last[topic] = t
            target = None
            if topic == '/usb_cam/image_raw':
                target = next((v for v in (25, 50, 73, 120, 175, 205) if v not in camera_done and abs(t-v) < .15), None)
                if target is None:
                    continue
            if kind not in types:
                types[kind] = get_message(kind)
            message = deserialize_message(blob, types[kind])
            stamp = (message.header.stamp.sec+message.header.stamp.nanosec*1e-9-start
                     if hasattr(message, 'header') else t)
            if topic == '/lio/odometry':
                p, q = message.pose.pose.position, message.pose.pose.orientation
                series['lio'].append([stamp, p.x, p.y, p.z, q.x, q.y, q.z, q.w])
            elif topic == '/mid360/livox/imu':
                series['imu'].append([stamp, *[getattr(message.linear_acceleration, k) for k in 'xyz'],
                                     *[getattr(message.angular_velocity, k) for k in 'xyz']])
            elif topic == '/ypspur_ros/odom':
                series['wheel'].append([stamp, message.twist.twist.linear.x, message.twist.twist.angular.z])
            elif topic == '/localization/pose_enu':
                p, q = message.pose.pose.position, message.pose.pose.orientation
                yaw = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz')[2]
                series['pose'].append([stamp, p.x, p.y, yaw])
            elif topic == '/follower_state':
                series['follower'].append([t, message.active_waypoint_index, message.state == 'RUNNING'])
            elif topic == '/fusion/gnss_dropout_active':
                series['dropout'].append([t, message.data])
            elif topic == '/cloud_registered_body':
                values = read_points(message, field_names=['x', 'y', 'z', 'intensity'])
                points = np.column_stack([values[k].reshape(-1) for k in ('x', 'y', 'z', 'intensity')])
                filename = f'cloud_{len(frames):05d}.npz'
                np.savez_compressed(cache/filename, points=points)
                frames.append([stamp, t, filename])
            elif topic == '/scan':
                ranges = np.array(message.ranges)
                angles = message.angle_min+np.arange(len(ranges))*message.angle_increment
                good = np.isfinite(ranges) & (ranges >= message.range_min) & (ranges <= min(message.range_max, 8.))
                xy = np.column_stack([ranges[good]*np.cos(angles[good])+.075, ranges[good]*np.sin(angles[good])])
                filename = f'scan_{len(scans):05d}.npy'
                np.save(cache/filename, xy)
                scans.append([stamp, filename])
            elif topic == '/active_route':
                route = [[p.pose.position.x, p.pose.position.y] for p in message.waypoints]
            elif topic == '/usb_cam/camera_info':
                camera_info = dict(k=list(message.k), frame=message.header.frame_id,
                                   calibrated=bool(message.k[0] > 0 and message.k[4] > 0))
            elif topic == '/usb_cam/image_raw':
                pixels = np.frombuffer(message.data, np.uint8).reshape(message.height, message.step)
                if message.encoding not in ('yuv422_yuy2', 'yuyv'):
                    raise ValueError('This sample expects C920 YUY2 images')
                bgr = cv2.cvtColor(pixels[:, :message.width*2].reshape(message.height, message.width, 2), cv2.COLOR_YUV2BGR_YUY2)
                cv2.imwrite(str(output/f'camera_{target}.jpg'), bgr)
                cameras.append(dict(target=target, received_s=t, header_s=stamp))
                camera_done.add(target)
        connection.close()
        print('read', bag_file, flush=True)
    arrays = {k: np.asarray(v, dtype=float) for k, v in series.items()}
    np.savez_compressed(cache/'telemetry.npz', **arrays)
    info = dict(bag=str(bag), start_epoch=start, duration_s=meta['duration']['nanoseconds']*1e-9,
                frames=frames, scans=scans, route=route, camera_info=camera_info, cameras=cameras)
    write_json(cache/'manifest.json', info)
    return info, arrays


def nearest(data, t):
    return data[np.argmin(abs(data[:, 0]-t))]


def latest(data, t):
    index = np.searchsorted(data[:, 0], t, side='right')-1
    return data[index] if index >= 0 else None


def calibration(data):
    core = GravityAlignment()
    for row in data['lio']:
        imu, wheel = nearest(data['imu'], row[0]), nearest(data['wheel'], row[0])
        if abs(imu[0]-row[0]) > .05 or abs(wheel[0]-row[0]) > .1:
            core.samples.clear()
            continue
        matrix = core.observe(row[0], 'raw', row[1:4], row[4:8], imu[1:4], imu[4:7], wheel[1:3])
        if matrix is not None:
            return matrix, float(row[0])
    raise ValueError('No stationary gravity calibration in this bag')


def replay(output, info, data, scene_output=None):
    config = SpaceConfig()
    if scene_output is not None:
        scene_output.mkdir(parents=True, exist_ok=True)
    level, calibrated_at = calibration(data)
    lio = data['lio']
    order = np.argsort(lio[:, 0]); lio = lio[order]
    _, unique = np.unique(lio[:, 0], return_index=True); lio = lio[unique]
    orientations = Slerp(lio[:, 0], Rotation.from_quat(lio[:, 4:8]))
    mount = rotation(*np.radians([-.6, 26.9, 0.]))
    lever = np.array([-.0102608953813, .0228267078793, .3694617699794])
    route = np.asarray(info['route'])

    def pose(t):
        matrix = level @ orientations(t).as_matrix()
        position = level @ np.array([np.interp(t, lio[:, 0], lio[:, k]) for k in range(1, 4)])
        base = position-matrix @ mount.T @ lever
        yaw = Rotation.from_matrix(matrix @ mount.T).as_euler('xyz')[2]
        return position, matrix, base, yaw

    history = deque()
    results, snapshots = [], []
    scan_times = np.array([v[0] for v in info['scans']])
    last_eval, previous_curvature = -100., 0.
    for stamp, received, filename in info['frames']:
        if stamp < max(10., lio[0, 0]) or stamp > lio[-1, 0]:
            continue
        points = np.load(output/'cache'/filename)['points']
        points = points[np.isfinite(points).all(axis=1)]
        position, matrix, base, yaw = pose(stamp)
        world = points.copy(); world[:, :3] = points[:, :3] @ matrix.T+position
        world = world[np.linalg.norm(world[:, :2]-base[:2], axis=1) < 7.]
        history.append((stamp, world))
        while history and stamp-history[0][0] > 6.:
            history.popleft()
        if stamp-last_eval < .95:
            continue
        last_eval = stamp
        tick = time.perf_counter()
        c, s = math.cos(yaw), math.sin(yaw)
        local_rotation = np.array([[c, -s], [s, c]])

        def local(world_points):
            result = world_points.copy()
            result[:, :3] -= base
            result[:, :2] = result[:, :2] @ local_rotation
            return result

        ground = local(np.concatenate([v for _, v in history]))
        keep = ((ground[:, 0] >= config.xmin) & (ground[:, 0] < config.xmax)
                & (ground[:, 1] >= config.ymin) & (ground[:, 1] < config.ymax)
                & (ground[:, 2] > -.65) & (ground[:, 2] < 1.8))
        ground = ground[keep]
        if len(ground):
            _, index = np.unique(np.floor(ground[:, :3]/.025).astype(np.int32), axis=0, return_index=True)
            ground = ground[index]
        preliminary = build_space(ground, np.empty((0, 2)), config)
        current = local(world)
        if preliminary.plane is None:
            obstacle = np.empty((0, 2))
        else:
            p = np.asarray(preliminary.plane)
            residual = current[:, 2] - current[:, :2] @ p[:2]-p[2]
            obstacle = current[(residual > .12) & (residual < 1.8), :2]
        scan_index = np.searchsorted(scan_times, stamp, side='right')-1
        scan_valid = False
        observation_age = 1.
        if scan_index >= 0 and stamp-scan_times[scan_index] <= .2 and scan_times[scan_index] >= lio[0, 0]:
            scan_stamp, scan_file = info['scans'][scan_index]
            scan = np.load(output/'cache'/scan_file)
            scan_valid = len(scan) > 0
            # Match the existing URG driver's consumer: sub-0.2 m returns are
            # inside the scanner/body near field, not external free-space evidence.
            scan = scan[np.linalg.norm(scan-[.075, 0.], axis=1) >= .2]
            _, _, scan_base, scan_yaw = pose(scan_stamp)
            cr, sr = math.cos(scan_yaw), math.sin(scan_yaw)
            scan_world = scan @ np.array([[cr, sr], [-sr, cr]])+scan_base[:2]
            scan = (scan_world-base[:2]) @ local_rotation
            obstacle = np.vstack([obstacle, scan])
            observation_age = max(0., received-stamp)+stamp-scan_stamp if scan_valid else 1.
        space = build_space(ground, obstacle, config)
        fused, follower = latest(data['pose'], stamp), latest(data['follower'], received)
        if fused is None or follower is None or stamp-fused[0] > .5:
            continue
        segment = min(max(1, int(follower[1])), len(route)-1)
        direction = route[segment]-route[segment-1]
        route_yaw = math.atan2(direction[1], direction[0])
        heading = math.atan2(math.sin(route_yaw-fused[3]), math.cos(route_yaw-fused[3]))
        route_normal = np.array([-math.sin(route_yaw), math.cos(route_yaw)])
        lateral = float((route[segment]-fused[1:3]) @ route_normal)
        input_previous_curvature = previous_curvature
        decision = choose_path(space, heading, lateral, previous_curvature=previous_curvature,
                               observation_age=observation_age)
        previous_curvature = decision['curvature']
        observed_obstacle_clearance = None
        if decision['path'] and len(obstacle):
            observed_obstacle_clearance = float(cKDTree(obstacle).query(np.asarray(decision['path']))[0].min())
        # Diagnose why no initial swept footprint could be established.
        xx, yy = np.meshgrid(config.xmin+(np.arange(space.labels.shape[1])+.5)*config.resolution,
                             config.ymin+(np.arange(space.labels.shape[0])+.5)*config.resolution)
        near = np.hypot(xx, yy) <= config.radius+config.resolution*math.sqrt(2)+config.sample_step/2
        near_types = sorted(int(v) for v in np.unique(space.labels[near]) if v != GROUND)
        variants = []
        for yaw_bias in (-10., 0., 10.):
            for position_bias in (-1., 0., 1.):
                v = choose_path(space, heading-math.radians(yaw_bias), lateral-position_bias,
                                observation_age=observation_age)
                variants.append(dict(yaw_bias_deg=yaw_bias, lateral_bias_m=position_bias,
                                     status=v['status'], curvature=v['curvature']))
        dropout = latest(data['dropout'], received)
        wheel = nearest(data['wheel'], stamp)
        result = dict(t=float(stamp), route_heading_deg=math.degrees(heading), lateral_target_m=lateral,
                      moving=bool(abs(wheel[1]) > .05), running=bool(follower[2]),
                      gnss_off=bool(dropout is not None and dropout[1]), scan_valid=scan_valid,
                      near_start_non_ground_labels=near_types,
                      observed_obstacle_clearance_m=observed_obstacle_clearance,
                      ground_cells=int(np.sum(space.labels == GROUND)), decision=decision,
                      variants=variants, analysis_ms=(time.perf_counter()-tick)*1000)
        results.append(result)
        if scene_output is not None:
            # Keep every input needed for an identical independent threshold sweep.
            scene_file = f'scene_{len(results)-1:04d}.npz'
            np.savez_compressed(scene_output/scene_file, ground=ground, obstacles=obstacle)
            result.update(scene_file=scene_file, observation_age=observation_age,
                          previous_curvature=input_previous_curvature)
        if len(results) % 40 == 0:
            print('evaluated', len(results), 'snapshots', flush=True)
        for requested in (25, 50, 73, 120, 175, 205):
            if abs(stamp-requested) < .55:
                np.savez_compressed(output/f'space_{requested}.npz', labels=space.labels,
                                    clearance=space.clearance, ground=ground, obstacles=obstacle)
                snapshots.append(dict(requested=requested, result=result))
    moving = [v for v in results if v['moving'] and v['running']]
    off = [v for v in moving if v['gnss_off']]
    reasons = {}
    for v in moving:
        reason = v['decision']['reason']; reasons[reason] = reasons.get(reason, 0)+1
    stats = dict(total_snapshots=len(results), moving_running_snapshots=len(moving),
                 moving_candidates=sum(v['decision']['status'] == 'candidate' for v in moving),
                 off_snapshots=len(off), off_candidates=sum(v['decision']['status'] == 'candidate' for v in off),
                 moving_reasons=reasons,
                 min_observed_obstacle_clearance_m=min(
                     (v['observed_obstacle_clearance_m'] for v in moving
                      if v['observed_obstacle_clearance_m'] is not None), default=None),
                 analysis_p50_ms=float(np.median([v['analysis_ms'] for v in results])),
                 analysis_p95_ms=float(np.percentile([v['analysis_ms'] for v in results], 95)),
                 gravity_calibration_at_s=calibrated_at, gravity_matrix=level.tolist(),
                 retrospective_calibration=True, camera_calibrated=info['camera_info']['calibrated'],
                 note='Recorded-observation replay; alternative trajectories have not been driven or re-sensed.')
    write_json(output/'replay.json', results)
    write_json(output/'replay_summary.json', stats)
    write_json(output/'snapshots.json', snapshots)
    if scene_output is not None:
        write_json(scene_output/'scenes.json', results)
    return stats


def synthetic_space(x, y, yaw, blocked=False, unknown=False, config=SpaceConfig()):
    ny = math.ceil((config.ymax-config.ymin)/config.resolution)
    nx = math.ceil((config.xmax-config.xmin)/config.resolution)
    xx, yy = np.meshgrid(config.xmin+(np.arange(nx)+.5)*config.resolution,
                         config.ymin+(np.arange(ny)+.5)*config.resolution)
    wx = x+math.cos(yaw)*xx-math.sin(yaw)*yy
    wy = y+math.sin(yaw)*xx+math.cos(yaw)*yy
    labels = np.where(abs(wy) < 1.2, GROUND, MATERIAL).astype(np.uint8)
    if blocked:
        labels[(wx > 4.) & (wx < 4.4)] = OBSTACLE
    if unknown:
        labels[wx > 4.] = 0
    return space_from_labels(labels, config)


def simulate(mode, position_bias, yaw_bias_deg, blocked=False, unknown=False):
    config = SpaceConfig()
    x, y, yaw, previous = 0., 0., 0., 0.
    path = [[x, y]]; outcome = 'timeout'
    for _ in range(900):
        estimated_yaw = yaw+math.radians(yaw_bias_deg)
        if mode == 'route_only':
            angle = math.atan2(-(y+position_bias), 2.5)-estimated_yaw
            curvature = float(np.clip(2*math.sin(angle)/2.5, -config.max_curvature, config.max_curvature))
            speed = config.max_speed
        else:
            space = synthetic_space(x, y, yaw, blocked, unknown, config)
            decision = choose_path(space, -estimated_yaw, -(y+position_bias),
                                   previous_curvature=previous)
            if decision['status'] == 'stop':
                outcome = 'stopped'; break
            curvature, speed = decision['curvature'], decision['speed']
        previous = curvature
        # Closed-loop unicycle integration. This synthetic map is independent of sensors.
        distance = speed*.1
        local = arc(curvature, distance)[-1]
        x, y = (x+math.cos(yaw)*local[0]-math.sin(yaw)*local[1],
                y+math.sin(yaw)*local[0]+math.cos(yaw)*local[1])
        yaw += curvature*distance
        path.append([x, y])
        # Analytical truth checks, separate from the planner's raster grid.
        if abs(y)+config.radius > 1.2:
            outcome = 'left_sidewalk'; break
        if blocked and x+config.radius >= 4.:
            outcome = 'hit_obstacle'; break
        if unknown and x+config.radius >= 4.:
            outcome = 'entered_unknown'; break
        if x >= 12.:
            outcome = 'reached'; break
    return dict(mode=mode, position_bias_m=position_bias, yaw_bias_deg=yaw_bias_deg,
                blocked=blocked, unknown=unknown, outcome=outcome, path=path,
                max_abs_lateral_m=float(np.max(np.abs(np.asarray(path)[:, 1]))))


def synthetic(output):
    cases = [simulate(mode, position_bias, yaw_bias)
             for position_bias in (-1., -.5, 0., .5, 1.)
             for yaw_bias in (-10., 0., 10.)
             for mode in ('route_only', 'local_space')]
    cases += [simulate('local_space', 0., 0., blocked=True),
              simulate('local_space', 0., 0., unknown=True)]
    write_json(output/'synthetic.json', cases)
    summary = {mode: dict(reached=sum(v['outcome'] == 'reached' for v in cases if v['mode'] == mode and not v['blocked'] and not v['unknown']),
                         total=15) for mode in ('route_only', 'local_space')}
    summary['blocked'] = cases[-2]['outcome']; summary['unknown'] = cases[-1]['outcome']
    for mode in ('route_only', 'local_space'):
        summary[mode]['max_abs_lateral_m'] = max(v['max_abs_lateral_m'] for v in cases if v['mode'] == mode)
    write_json(output/'synthetic_summary.json', summary)
    return summary


def material_counterexample(output):
    """A deliberately indistinguishable low-grass case exposes the sensor limit."""
    config = SpaceConfig()
    xx, yy = np.meshgrid(np.arange(config.xmin, config.xmax, .03),
                         np.arange(config.ymin, config.ymax, .03))
    grass = yy < -.6
    points = np.column_stack([xx.ravel(), yy.ravel(), np.full(xx.size, -.15),
                              np.where(grass, 20., 15.).ravel()])
    space = build_space(points, np.empty((0, 2)), config)
    cell_y = config.ymin+(np.arange(space.labels.shape[0])+.5)*config.resolution
    grass_labels = space.labels[cell_y < -.75]
    result = dict(scenario='Flat low grass, intensity 20; pavement intensity 15',
                  false_ground_cells=int(np.sum(grass_labels == GROUND)),
                  grass_cells=int(grass_labels.size),
                  conclusion='Geometry/intensity cannot establish sidewalk semantics. Camera calibration and a validated semantic classifier are required.')
    write_json(output/'material_counterexample.json', result)
    return result


def plots(output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    cases = json.loads((output/'synthetic.json').read_text())
    fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True, sharey=True, constrained_layout=True)
    for ax, mode, title in zip(axes, ('route_only', 'local_space'),
                               ('Route following model', 'Route direction + local free space')):
        ax.axhspan(-1.2, 1.2, color='#e2efe6')
        ax.axhline(.8, color='#777777', ls='--', lw=1); ax.axhline(-.8, color='#777777', ls='--', lw=1)
        for item in cases:
            if item['mode'] != mode or item['blocked'] or item['unknown']:
                continue
            points = np.array(item['path'])
            ax.plot(points[:, 0], points[:, 1], alpha=.7, color='#247ba0' if item['outcome'] == 'reached' else '#c73e45')
        ax.set(title=title, ylabel='Lateral position [m]', xlim=(0, 12.5), ylim=(-1.45, 1.45))
        ax.grid(alpha=.2)
    axes[-1].set_xlabel('Forward progress [m]')
    fig.suptitle('Synthetic 2.4 m sidewalk: position bias +/-1 m, yaw bias +/-10 deg\nDashed lines: robot-centre limit for radius 0.40 m', fontsize=12)
    fig.savefig(output/'synthetic_comparison.png', dpi=180); plt.close(fig)
    config = SpaceConfig()
    snapshots = json.loads((output/'snapshots.json').read_text())
    cmap = ListedColormap(['#dbe0e5', '#a4d5ae', '#c4a060', '#cd4a50', '#794fa1'])
    for snapshot in snapshots:
        requested = snapshot['requested']; decision = snapshot['result']['decision']
        saved = np.load(output/f'space_{requested}.npz')
        fig, axes = plt.subplots(1, 2, figsize=(10, 5.3), constrained_layout=True)
        photo = output/f'camera_{requested}.jpg'
        if photo.exists():
            axes[0].imshow(plt.imread(photo))
        axes[0].set_title(f'Camera: approximately {requested} s\nVisual reference; no calibrated projection', fontsize=11)
        axes[0].axis('off')
        ax = axes[1]
        # Display forward as vertical and robot-left as plot-left.
        ax.imshow(saved['labels'].T, origin='lower', extent=[config.ymin, config.ymax, config.xmin, config.xmax],
                  cmap=cmap, vmin=0, vmax=4, interpolation='nearest')
        ax.invert_xaxis()
        ax.add_patch(plt.Circle((0, 0), config.radius, fill=False, color='#153d5c', lw=2))
        heading = math.radians(snapshot['result']['route_heading_deg'])
        ax.plot([0, 2*math.sin(heading)], [0, 2*math.cos(heading)], '--', color='#3575bd', label='Route direction')
        if decision['path']:
            path = np.asarray(decision['path'])
            ax.plot(path[:, 1], path[:, 0], color='#111111', lw=2.5, label='Selected candidate')
        ax.set(xlabel='Left [m]', ylabel='Forward [m]', title=f"{decision['status'].upper()}: {decision['reason']}")
        ax.legend(loc='upper right', fontsize=8)
        fig.suptitle('Green: observed ground | Red: current obstacles\nBrown: material/roughness veto | Purple: lower surface | Grey: unknown', fontsize=11)
        fig.savefig(output/f'example_{requested}.png', dpi=160); plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--reuse-cache', action='store_true')
    parser.add_argument('--synthetic-only', action='store_true')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    if not args.synthetic_only:
        if args.reuse_cache:
            info = json.loads((args.output/'cache/manifest.json').read_text())
            data = dict(np.load(args.output/'cache/telemetry.npz'))
        else:
            if args.bag is None:
                parser.error('--bag is required without --reuse-cache')
            info, data = read_bag(args.bag.resolve(), args.output)
        print(json.dumps(replay(args.output, info, data), indent=2), flush=True)
    print(json.dumps(synthetic(args.output), indent=2), flush=True)
    print(json.dumps(material_counterexample(args.output), indent=2), flush=True)
    if not args.synthetic_only:
        plots(args.output)


if __name__ == '__main__':
    main()
