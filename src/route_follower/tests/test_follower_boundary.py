"""FollowerCore の境界挙動を固定する。ROS グラフもモータ接続も使わない。

ここで固定するのは、詳細設計書と README が制約として記載している値である。
どちらか一方だけが変更されると検出できるよう、関連ノードの既定値も併せて検査する。
"""

from pathlib import Path
import sys

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).parents[1]))
from route_follower.follower_core import FollowerCore, Pose, Route, Waypoint


class _NullLogger:
    """FollowerCore が要求するロガー。出力は破棄する."""

    def info(self, *args: object) -> None:
        return None


def _core_at_waypoint(line_stop: bool) -> FollowerCore:
    """1 点だけの経路を与え、到達判定が成立する位置まで進めた core を返す.

    Args:
        line_stop (bool): 唯一の waypoint に停止属性を付けるか.

    Returns:
        FollowerCore: tick() 済みの core.
    """
    core = FollowerCore(_NullLogger())
    core.update_route(Route(1, [Waypoint('target', Pose(1, 0), line_stop=line_stop)]))
    core.update_pose(Pose(0, 0))
    core.tick()
    core.update_pose(Pose(0.45, 0))
    return core


def test_line_stop_waypoint_stops_outside_navigator_tolerance() -> None:
    """line_stop 到達時、follower は navigator の位置許容より外側で停止判定する.

    残距離 0.55 m で WAITING_STOP へ遷移する一方、navigator の既定 pos_tol は
    0.5 m である。follower が停止を判断した時点でも navigator は目標未到達と
    見なすため、両者の判定は一致しない。
    """
    core = _core_at_waypoint(line_stop=True)
    out = core.tick()

    assert core.status.name == 'WAITING_STOP'
    # 停止判定後も目標を返し続ける。navigator 側の指令は失効しない。
    assert out.target_pose is not None

    params = yaml.safe_load(
        (Path(__file__).parents[2] / 'robot_navigator/params/default.yaml').read_text(
            encoding='utf-8'
        )
    )
    pos_tol = params['robot_navigator']['ros__parameters']['pos_tol']
    assert 0.55 > pos_tol


def test_final_waypoint_without_line_stop_finishes_and_drops_target() -> None:
    """停止属性の無い最終 waypoint に到達すると FINISHED になり目標を返さない."""
    core = _core_at_waypoint(line_stop=False)
    out = core.tick()

    assert core.status.name == 'FINISHED'
    assert out.target_pose is None


def test_avoidance_is_not_started_when_allowed_width_is_too_narrow() -> None:
    """許可幅が最小オフセットに満たない場合、回避を開始しない.

    最小オフセットを下限として優先すると、waypoint が許可した幅 (left_open) を
    超える横移動を作ってしまう。幅が足りない側は回避できないものとして扱い、
    呼び出し側は再経路の要求へ進む。
    """
    core = FollowerCore(_NullLogger())
    allowed_m = 0.1

    started = core._start_avoidance_sequence(
        Waypoint('narrow', Pose(2, 0), left_open=allowed_m), Pose(0, 0), allowed_m, 0
    )

    assert started is False
    assert core.last_applied_offset_m == 0.0


def test_avoidance_offset_stays_within_the_allowed_width() -> None:
    """許可幅が最小オフセット以上なら、その幅を超えない横移動を生成する."""
    core = FollowerCore(_NullLogger())
    allowed_m = 0.5

    started = core._start_avoidance_sequence(
        Waypoint('narrow', Pose(2, 0), left_open=allowed_m), Pose(0, 0), 2.0, 0
    )

    assert started is True
    assert abs(core.last_applied_offset_m) == pytest.approx(allowed_m)
