"""GNSSと自己位置推定の軌跡を別々に保存する。fusedキーは保存形式の互換名。"""
from collections import deque
import math


class Traces:
    def __init__(self, limit: int = 100000) -> None:
        self.gnss = deque(maxlen=limit)
        self.fused = deque(maxlen=limit)
        self.statuses = deque(maxlen=limit)
        self.dropped = 0

    def append(self, kind: str, value: dict) -> None:
        target = getattr(self, kind)
        if target and value['stamp'] <= target[-1]['stamp']:
            return
        if kind != 'statuses' and target and value['stamp']-target[-1]['stamp'] < .19:
            return
        self.dropped += int(len(target) == target.maxlen)
        target.append(value)

    def export(self) -> dict:
        statuses = list(self.statuses)
        gnss = []
        index = 0
        for sample in self.gnss:
            while index+1 < len(statuses) and statuses[index+1]['stamp'] <= sample['stamp']:
                index += 1
            choices = statuses[max(0,index-1):index+2]
            nearest = min(choices, key=lambda s: abs(s['stamp']-sample['stamp'])) if choices else None
            valid = nearest is not None and abs(nearest['stamp']-sample['stamp']) <= .25
            gnss.append(dict(sample, quality=dict(nearest) if valid else None))
        return dict(gnss=gnss, fused=list(self.fused), dropped=self.dropped,
                    max_join_delta_s=.25, max_line_gap_s=1.)


def finite(value: float) -> float | None:
    return float(value) if math.isfinite(value) else None
