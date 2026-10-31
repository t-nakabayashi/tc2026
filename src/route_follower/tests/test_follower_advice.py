"""AI 助言の要求条件・採用・待機の挙動を固定する。ROS グラフも CLI も使わない。

助言は提案値であり、採用しても実効上限を超える横移動は作らない。助言が無い・遅い・
余裕が足りない場合は従来の回避へ退避する。ここではその境界を固定する。
"""

from pathlib import Path
import sys
import time

import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
from route_follower.follower_core import (AdviceDecision, FollowerCore, FollowerStatus,
                                          HintSample, Pose, Route, Waypoint)


class _NullLogger:
    def info(self, *args: object) -> None:
        return None


def _blocked_core(left_open: float = 2.0, right_open: float = 2.0) -> FollowerCore:
    """前方遮蔽が多数決で成立し、停止している core を作る."""
    core = FollowerCore(_NullLogger())
    core.use_ai_advice = True
    core.update_route(Route(1, [Waypoint('target', Pose(10, 0),
                                         left_open=left_open, right_open=right_open)]))
    core.update_pose(Pose(0, 0))
    core.tick()
    now = time.time()
    # 停止判定に使う位置履歴を、窓いっぱいに同じ位置で埋める。
    core.pose_hist.clear()
    for offset in range(5):
        core.pose_hist.append((now - core.window_sec + offset * 0.4, Pose(0, 0)))
    for index in range(core.hint_min_samples + 1):
        core.update_hint(HintSample(t=now - index * 0.1, front_blocked=True,
                                    left_offset=1.0, right_offset=1.0,
                                    front_clearance=0.7))
    return core


def _trigger_ready(core: FollowerCore) -> None:
    """停止継続の条件を満たした状態にする."""
    core.tick()
    core._advice_stop_since = time.time() - core.advice_trigger_hold_sec - 0.1


def test_no_request_is_made_while_the_feature_is_disabled() -> None:
    """既定（無効）では助言を要求しない。従来どおりの動作になる."""
    core = _blocked_core()
    core.use_ai_advice = False
    _trigger_ready(core)
    core._advice_stop_since = None

    assert core.take_advice_request() is None


def test_no_request_before_the_stop_hold_time() -> None:
    """停止が続いた時間が足りないうちは要求しない.

    低速の先行車へ追い付いて短く止まる動作を繰り返す場合に、呼び出しが嵩むのを防ぐ。
    """
    core = _blocked_core()
    core.tick()
    core._advice_stop_since = time.time() - 0.5

    assert core.take_advice_request() is None


def test_request_carries_the_effective_side_limits() -> None:
    """要求には採取幅・実測空き幅・大域上限の最小値が入る.

    助言側が上限を知らないと、採用できない値が返ってくる。上限は要求側が渡す。
    """
    core = _blocked_core(left_open=2.0, right_open=2.0)
    core.update_layer_clearance(0.8, 0.4)
    _trigger_ready(core)

    request = core.take_advice_request()

    assert request is not None
    assert request.waypoint_left_open_m == pytest.approx(0.8)
    assert request.waypoint_right_open_m == pytest.approx(0.4)
    assert request.min_offset_m == pytest.approx(core.avoid_min_offset_m)


def test_stale_layer_clearance_is_not_used_as_a_limit() -> None:
    """実測空き幅が古い場合は上限に使わない.

    観測が無いことを「塞がっている」と扱うと、観測が途絶えただけで回避できなくなる。
    """
    core = _blocked_core(left_open=2.0, right_open=2.0)
    core.update_layer_clearance(0.8, 0.4)
    core._layer_stamp = time.time() - core.layer_clearance_timeout_sec - 1.0
    _trigger_ready(core)

    request = core.take_advice_request()

    assert request.waypoint_left_open_m == pytest.approx(2.0)


def test_second_request_is_suppressed_while_one_is_in_flight() -> None:
    """問い合わせ中は次の要求を出さない."""
    core = _blocked_core()
    _trigger_ready(core)

    assert core.take_advice_request() is not None
    assert core.take_advice_request() is None


def test_request_is_suppressed_by_cooldown_after_a_result() -> None:
    """結果を受け取った直後は、時間も距離も満たさない限り再要求しない."""
    core = _blocked_core()
    _trigger_ready(core)
    core.take_advice_request()
    core.submit_advice(None, 'timeout')
    core._advice_stop_since = time.time() - core.advice_trigger_hold_sec - 0.1

    assert core.take_advice_request() is None


def test_moving_far_enough_allows_a_new_request() -> None:
    """一定距離を進めば、クールダウン中でも再要求できる.

    別の障害物へ進んだ場合に、前の問い合わせのせいで助言を得られなくなることを防ぐ。
    """
    core = _blocked_core()
    _trigger_ready(core)
    core.take_advice_request()
    core.submit_advice(None, 'timeout')
    core.update_pose(Pose(core.advice_cooldown_m + 1.0, 0))
    core.tick()
    core._advice_stop_since = time.time() - core.advice_trigger_hold_sec - 0.1

    assert core.take_advice_request() is not None


def test_request_is_suppressed_when_the_budget_is_used_up() -> None:
    """呼び出し予算を使い切ったら要求しない（従来の回避だけで走る）."""
    core = _blocked_core()
    core.advice_budget_total = 1
    _trigger_ready(core)
    core.take_advice_request()
    core.submit_advice(None, 'timeout')
    core._advice_last_call_time = None
    core._advice_stop_since = time.time() - core.advice_trigger_hold_sec - 0.1

    assert core.take_advice_request() is None


def test_advice_is_discarded_when_the_front_clears() -> None:
    """前方が空いたら、保持していた助言を捨てる.

    状況が変わった後の助言で回避を始めないようにする。
    """
    core = _blocked_core()
    core.submit_advice(AdviceDecision(action='avoid', side='L', offset_m=1.0,
                                      confidence=0.9))
    core.update_hint_stat(False, 0.0, 0.0, True)
    core.tick()

    assert core._advice_result is None


def test_advice_offset_is_clamped_by_the_measured_clearance() -> None:
    """助言が実測空き幅を超える場合、その幅で打ち切る."""
    core = _blocked_core(left_open=2.0)
    core.update_layer_clearance(0.6, 0.6)
    advice = AdviceDecision(action='avoid', side='L', offset_m=1.8, confidence=0.9)

    started = core._start_avoidance_sequence(
        core.route.waypoints[0], Pose(0, 0), 1.0, 1.0, advice)

    assert started is True
    assert abs(core.last_applied_offset_m) == pytest.approx(0.6)


def test_advice_for_a_side_without_room_falls_back_to_the_legacy_choice() -> None:
    """余裕が無い側を助言された場合、従来の左右選択へ退避する."""
    core = _blocked_core(left_open=0.0, right_open=2.0)
    advice = AdviceDecision(action='avoid', side='L', offset_m=1.0, confidence=0.9)

    started = core._start_avoidance_sequence(
        core.route.waypoints[0], Pose(0, 0), 1.0, 1.0, advice)

    assert started is True
    assert core.last_applied_offset_m > 0.0  # 右回避は last_applied_offset_m が正


def test_wait_advice_enters_the_waiting_state() -> None:
    """待機の助言を受けると待機状態へ入り、目標を変えない."""
    core = _blocked_core()
    core.update_pose(Pose(0, 0))
    advice = AdviceDecision(action='wait', wait_sec=5.0, is_moving=True,
                            obstacle_class='person', confidence=0.9)

    output = core._enter_waiting_obstacle(advice)

    assert core.status == FollowerStatus.WAITING_OBSTACLE
    assert output.target_pose == core.last_target
    assert core.obstacle_wait_until > time.time()


def test_wait_time_is_capped_by_the_limit() -> None:
    """助言された待機時間は上限で打ち切る."""
    core = _blocked_core()
    core._enter_waiting_obstacle(AdviceDecision(action='wait', wait_sec=600.0))

    assert core.obstacle_wait_until <= time.time() + core.max_obstacle_wait_sec + 0.1


def test_waiting_ends_immediately_when_the_front_clears() -> None:
    """待機中に前方が空いたら、すぐ追従へ戻る（前車が動けばそのまま追従する）."""
    core = _blocked_core()
    core._enter_waiting_obstacle(AdviceDecision(action='wait', wait_sec=30.0))
    core.update_hint_stat(False, 0.0, 0.0, True)

    core.tick()

    assert core.status == FollowerStatus.RUNNING
    assert core.obstacle_wait_until is None


def test_waiting_timeout_starts_the_legacy_avoidance() -> None:
    """待機上限を過ぎても塞がったままなら、従来の回避を試みる."""
    core = _blocked_core()
    core._enter_waiting_obstacle(AdviceDecision(action='wait', wait_sec=0.0))
    core.obstacle_wait_until = time.time() - 0.1

    core.tick()

    assert core.status == FollowerStatus.AVOIDING
    assert core.last_stagnation_reason == 'wait_timeout'


def test_waiting_timeout_without_room_requests_reroute() -> None:
    """待機上限後に回避できる幅が無ければ、再経路の要求へ進む."""
    core = _blocked_core(left_open=0.0, right_open=0.0)
    core._enter_waiting_obstacle(AdviceDecision(action='wait', wait_sec=0.0))
    core.obstacle_wait_until = time.time() - 0.1

    core.tick()

    assert core.status == FollowerStatus.WAITING_REROUTE
    assert core.last_stagnation_reason == 'no_space'
