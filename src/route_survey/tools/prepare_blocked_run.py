#!/usr/bin/env python3
"""回避幅を効かせるため、模擬環境の経路上へ障害物を置いた試験を用意する.

同梱地図には生垣やロードコーンが無く、経路沿いの離隔が広いため、そのまま走らせても
回避が起きない。幅の値が追従側の横移動をどう縛るかを見るには、経路上に障害物を置いて
回避を起こす必要がある。開始位置も障害物の手前へ移し、1回の走行を短くする。

地図・経路の原本は変更せず、展開済みのセッションだけを書き換える。
"""
import argparse
import csv
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from geo_pose_converter.geo_core import LlhPoint, llh_to_enu, load_projection_config_from_yaml

# 既存の評価環境と同じ寸法の箱を使う。高さはURGの走査面0.49 mを越える
BLOCKER_SIZE = (.6, .6, .8)


def route_in_world(route: Path, projection: Path) -> tuple[list[str], np.ndarray, np.ndarray]:
    """経路を、走行時と同じ投影でGazebo world座標へ直し、弧長を付ける.

    Args:
        route (Path): 回避幅を書き込んだ waypoints.csv.
        projection (Path): セッションの projection.yaml.

    Returns:
        tuple[list[str], np.ndarray, np.ndarray]: ラベル、位置[m]、始点からの距離[m].
    """
    config = load_projection_config_from_yaml(str(projection))
    rows = list(csv.DictReader(route.open()))
    point = np.array([[(enu := llh_to_enu(
        LlhPoint(float(r['latitude']), float(r['longitude']), 0.), config)).x, enu.y]
        for r in rows])
    step = np.linalg.norm(np.diff(point, axis=0), axis=1)
    return [r['label'] for r in rows], point, np.concatenate([[0.], np.cumsum(step)])


class Terrain:
    """地形の高さ格子。物体とロボットを地面へ置くために引く."""

    def __init__(self, world_json: Path) -> None:
        self.grid = json.loads(world_json.read_text())['grid']

    def height(self, x: float, y: float) -> float:
        """world座標の地表高さを返す.

        Args:
            x (float): world X [m].
            y (float): world Y [m].

        Returns:
            float: 地表の高さ [m].
        """
        grid = self.grid
        i = min(max(round((x-grid['ox'])/grid['res']), 0), grid['w']-1)
        j = min(max(round((y-grid['oy'])/grid['res']), 0), grid['h']-1)
        return float(grid['height'][j*grid['w']+i])


def place(world, name: str, x: float, y: float, z: float) -> None:
    """静的な箱を world へ追加する.

    Args:
        world: SDFのworld要素.
        name (str): モデル名.
        x (float): world X [m].
        y (float): world Y [m].
        z (float): 箱の中心高さ [m].

    Returns:
        None
    """
    model = ET.SubElement(world, 'model', name=name)
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'pose').text = f'{x} {y} {z} 0 0 0'
    link = ET.SubElement(model, 'link', name='link')
    for kind in ['visual', 'collision']:
        element = ET.SubElement(link, kind, name=kind)
        box = ET.SubElement(ET.SubElement(element, 'geometry'), 'box')
        ET.SubElement(box, 'size').text = ' '.join(str(v) for v in BLOCKER_SIZE)


def main() -> None:
    """セッションの経路・開始終了・world を、障害物つきの試験用へ書き換える.

    Returns:
        None
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path, required=True, help='展開済みセッションのディレクトリ')
    parser.add_argument('--route', type=Path, required=True, help='回避幅を書き込んだ waypoints.csv')
    parser.add_argument('--start-distance', type=float, required=True, help='走行開始の距離 [m]')
    parser.add_argument('--goal-distance', type=float, required=True, help='目標の距離 [m]')
    parser.add_argument('--blocker-distance', type=float, action='append', required=True,
                        help='障害物を置く距離 [m]。複数指定できる')
    parser.add_argument('--blocker-lateral', type=float, default=0.,
                        help='経路中心からの横オフセット [m]。左が正')
    args = parser.parse_args()

    session = args.session
    projection = session/'projection.yaml'
    label, point, distance = route_in_world(args.route, projection)
    if args.goal_distance <= args.start_distance:
        raise SystemExit('目標は開始より先にしてください')
    if not all(args.start_distance < b < args.goal_distance for b in args.blocker_distance):
        raise SystemExit('障害物は開始と目標の間に置いてください')

    (session/'routes/fixed').mkdir(parents=True, exist_ok=True)
    (session/'routes/fixed/waypoints.csv').write_text(args.route.read_text())
    start = int(np.argmin(np.abs(distance-args.start_distance)))
    goal = int(np.argmin(np.abs(distance-args.goal_distance)))
    settings = json.loads((session/'session.yaml').read_text())
    settings['start_label'] = label[start]
    settings['goal_label'] = label[goal]
    (session/'session.yaml').write_text(json.dumps(settings, ensure_ascii=False, indent=2))

    terrain = Terrain(session/'world/world.json')
    tree = ET.parse(session/'world/trial.sdf')
    world = tree.getroot().find('world')
    # 開始位置を手前へ移し、1回の走行を短くする。向きは次の点へ向ける
    x, y = point[start]
    ahead = point[min(start+1, len(point)-1)]
    yaw = math.atan2(ahead[1]-y, ahead[0]-x)
    world.find("model[@name='icart_mini']/pose").text = \
        f'{x} {y} {terrain.height(x, y)+.08} 0 0 {yaw}'
    for number, target in enumerate(sorted(args.blocker_distance), start=1):
        index = int(np.argmin(np.abs(distance-target)))
        base = point[index]
        follow = point[min(index+1, len(point)-1)]
        heading = math.atan2(follow[1]-base[1], follow[0]-base[0])
        normal = np.array([-math.sin(heading), math.cos(heading)])
        bx, by = base+normal*args.blocker_lateral
        place(world, f'width_check_blocker_{number:02d}', bx, by,
              terrain.height(bx, by)+BLOCKER_SIZE[2]/2)
        print(f'障害物{number}: {target:.0f} m 地点 ({bx:.1f}, {by:.1f}) '
              f'横オフセット {args.blocker_lateral:+.1f} m')
    tree.write(session/'world/trial.sdf', encoding='utf-8', xml_declaration=True)
    print(f'開始 {label[start]} ({distance[start]:.0f} m) → 目標 {label[goal]} '
          f'({distance[goal]:.0f} m)')


if __name__ == '__main__':
    main()
