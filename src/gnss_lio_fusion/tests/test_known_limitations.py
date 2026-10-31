"""詳細設計書 10.2 節の既知の欠陥を再現する。ROS・ドライバ・車体は起動しない。

いずれも**望ましい挙動**を assert し、`xfail(strict=True)` を付けている。現状は欠陥が
あるため失敗し、xfail として扱われる。欠陥が修正されると xpass になり、strict 指定に
よりテストが失敗する。そこで初めて、このマーカーと詳細設計書 10.2 節の記述を外す。

現状の（誤った）挙動を assert して通してしまうと、修正時にテストが壊れる形になり、
欠陥を固定してしまう。そのため逆向きに書く。
"""

from pathlib import Path
import sys
import threading

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src/gnss_lio_fusion'))
sys.path.insert(0, str(ROOT / 'src/rtk_gps_um982/third_party/UM982-RTK-GPS-Library'))

from gnss_lio_fusion.motion_guard_core import MotionGuard  # noqa: E402


@pytest.mark.xfail(
    strict=True,
    reason='詳細設計書 10.2.1: 方位の鮮度を入力境界で検査していない',
)
def test_stale_heading_is_not_merged_into_fresh_position() -> None:
    """100 秒古い方位が、新しい位置観測へそのまま結合されないことを確認する.

    位置 stamp=1000、方位 stamp=900 を与える。望ましい挙動は、方位を無効として扱うか、
    方位側の観測時刻を保持して購読側が鮮度を判定できる状態にすることである。
    """
    from um982.client import UM982Client
    from um982.types import GGAData, UniheadingData

    client = UM982Client.__new__(UM982Client)
    client._data_lock = threading.Lock()
    client._rmc = None
    client._gga = GGAData(36., 140., 10., 4, 20, .7, 0., '', 1000., '')
    client._uniheading = UniheadingData(.15, 90., 0., 1., 1., '', 20, 20, 20, 20, 900.)

    position = client.get_position()

    # 位置 stamp と方位 stamp の差が購読側から判定できる状態を期待する。
    heading_age_s = position.timestamp - client._uniheading.timestamp
    assert heading_age_s == pytest.approx(100.)
    assert position.heading is None or getattr(position, 'heading_timestamp', None) is not None


@pytest.mark.xfail(
    strict=True,
    reason='詳細設計書 10.2.2: 相対運動を採用できない入力から自動復帰しない',
)
def test_motion_guard_recovers_after_inputs_return_to_normal() -> None:
    """入力が正常化したら相対運動の採用が再開することを確認する.

    0.2 秒で車輪が欠測し、0.3 秒で LIO がジャンプして車輪が復帰する。0.4 秒以降は
    両入力が正常である。望ましい挙動は、採用時刻が 0.2 秒で止まらず進むことである。
    """
    guard = MotionGuard()
    samples = [
        (0., [0, 0, 0], [0, 0, 0]),
        (.1, [.05, 0, 0], [.05, 0, 0]),
        (.2, [.1, 0, 0], None),
        (.3, [10, 0, 0], [.15, 0, 0]),
    ]
    samples += [
        (i / 10, [10 + (i - 3) * .05, 0, 0], [i * .05, 0, 0])
        for i in range(4, 51)
    ]
    for stamp, lio, wheel in samples:
        guard.select(stamp, np.array(lio), None if wheel is None else np.array(wheel))

    assert guard.stamp > .2, '入力が正常化しても採用時刻が進んでいない'
    assert guard.source != 'NO_MOTION'
