"""規定のLLH CSVと、編集・再確認用の観測情報を保存する."""
import csv
import json
from pathlib import Path

import numpy as np
import yaml
from geo_pose_converter.geo_core import (
    ProjectionConfig, enu_to_llh_on_ground, yaw_enu_rad_to_heading_deg,
)

FIELDS = ['label', 'latitude', 'longitude', 'altitude', 'heading_deg', 'right_is_open',
          'left_is_open', 'line_is_stop', 'signal_is_stop', 'isnot_skipnum']


def save(directory: Path, rows: list[dict], projection: ProjectionConfig,
         cloud: np.ndarray, active: bool, traces: dict | None = None) -> None:
    """一時ファイルを置換し、書きかけCSVを公開しない。既存採取の再開はしない."""
    (directory/'fixed').mkdir(parents=True, exist_ok=True)
    export = []
    for i, row in enumerate(rows):
        llh = enu_to_llh_on_ground(row['x'], row['y'], projection,
                                  ground_altitude=projection.origin_altitude)
        export.append(dict(label=str(i), latitude=llh.latitude, longitude=llh.longitude,
                           altitude='', heading_deg=yaw_enu_rad_to_heading_deg(
                               row['yaw']+projection.map_yaw_offset_rad),
                           **{k: row[k] for k in FIELDS[5:]}))
    target = directory/'fixed/waypoints.csv'
    tmp = target.with_suffix('.tmp')
    with tmp.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS, lineterminator='\n')
        writer.writeheader()
        writer.writerows(export)
    tmp.replace(target)
    metadata = dict(version=2, traces=traces or {}, active=active, projection=vars(projection),
                    waypoints=[dict(a, observation=b.get('observation', {}),
                                    reason=b.get('reason', 'edited')) for a, b in zip(export, rows)],
                    cloud_frame='map', cloud_source='FAST-LIO body scans + synchronized localization pose',
                    cloud=np.asarray(cloud).tolist())
    tmp = directory/'survey.tmp'
    tmp.write_text(json.dumps(metadata, ensure_ascii=False), encoding='utf-8')
    tmp.replace(directory/'survey.json')
    (directory/'projection.yaml').write_text(yaml.safe_dump(
        {'/**': {'ros__parameters': vars(projection)}}))
    (directory/'route_config.yaml').write_text(yaml.safe_dump(
        {'blocks': [{'type': 'fixed', 'name': 'survey', 'segment_id': 'fixed/waypoints.csv'}]}))
