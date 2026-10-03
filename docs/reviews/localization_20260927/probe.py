"""Read-only synthetic probes. No ROS nodes, drivers, or vehicle commands."""
import json
from pathlib import Path
import sys
import threading

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src/gnss_lio_fusion'))
sys.path.insert(0, str(ROOT / 'src/rtk_gps_um982/third_party/UM982-RTK-GPS-Library'))
from gnss_lio_fusion.motion_guard_core import MotionGuard
from um982.client import UM982Client
from um982.types import GGAData, UniheadingData

guard = MotionGuard()
rows = []
samples = [(0., [0, 0, 0], [0, 0, 0]),
           (.1, [.05, 0, 0], [.05, 0, 0]),
           (.2, [.1, 0, 0], None),
           (.3, [10, 0, 0], [.15, 0, 0])]
samples += [(i / 10, [10 + (i - 3) * .05, 0, 0], [i * .05, 0, 0])
            for i in range(4, 51)]
for stamp, lio, wheel in samples:
    out = guard.select(stamp, np.array(lio), None if wheel is None else np.array(wheel))
    rows.append(dict(stamp=stamp, pose=None if out is None else out.tolist(),
                     accepted_stamp=guard.stamp, source=guard.source))

client = UM982Client.__new__(UM982Client)
client._data_lock = threading.Lock()
client._rmc = None
client._gga = GGAData(36., 140., 10., 4, 20, .7, 0., '', 1000., '')
client._uniheading = UniheadingData(.15, 90., 0., 1., 1., '', 20, 20, 20, 20, 900.)
pos = client.get_position()
result = dict(
    stale_heading=dict(position_stamp=pos.timestamp, heading_stamp=client._uniheading.timestamp,
                       age_s=pos.timestamp-client._uniheading.timestamp,
                       copied_heading_deg=pos.heading, copied_sigma_deg=pos.heading_stddev,
                       rtk_state=pos.rtk_state),
    motion_recovery=dict(final_source=guard.source, final_accepted_stamp=guard.stamp,
                         rejected_count=guard.rejected_count, samples=rows))
assert pos.heading == 90. and pos.timestamp == 1000.
assert guard.source == 'NO_MOTION' and guard.stamp == .2
print(json.dumps(result, indent=2))
