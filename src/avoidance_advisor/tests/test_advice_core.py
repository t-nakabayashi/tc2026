"""助言の検証とクランプを固定する。CLI も ROS グラフも使わない。

助言は提案値であり、検査に通った場合だけ採用する。ここでは不採用の条件と、
採用時に値がどこまで丸められるかを固定する。
"""

from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
from avoidance_advisor.advice_core import (ACTION_AVOID, ACTION_WAIT, REASON_CONFIDENCE,
                                           REASON_EMPTY, REASON_NO_SPACE, REASON_PARSE,
                                           REASON_SCHEMA, SIDE_LEFT, SIDE_NONE,
                                           SIDE_RIGHT, AdvicePolicy, AdviceRequest,
                                           build_prompt, clamp_offset,
                                           extract_json_object, parse_advice)

POLICY = AdvicePolicy(min_confidence=0.5, default_wait_sec=10.0, max_wait_sec=30.0)
REQUEST = AdviceRequest(waypoint_label='wp1', front_clearance_m=0.7,
                        min_offset_m=0.35, max_left_offset_m=1.0,
                        max_right_offset_m=0.5,
                        context_lines=('layer=urg nearest_x_m=0.70',))


def advice_json(**overrides) -> str:
    payload = {'action': 'avoid', 'side': 'left', 'offset_m': 0.8, 'wait_sec': 0.0,
               'obstacle_class': 'cone', 'is_moving': False, 'confidence': 0.9,
               'rationale': '正面にコーン'}
    payload.update(overrides)
    import json
    return json.dumps(payload, ensure_ascii=False)


def test_valid_avoid_advice_is_accepted() -> None:
    """スキーマどおりの回避助言は、方向とオフセットを保って採用される."""
    advice, reason = parse_advice(advice_json(), REQUEST, POLICY)

    assert reason == ''
    assert advice.action == ACTION_AVOID
    assert advice.side == SIDE_LEFT
    assert advice.offset_m == pytest.approx(0.8)
    assert advice.obstacle_class == 'cone'


def test_wait_advice_uses_clamped_wait_time() -> None:
    """待機助言の待機時間は上限で打ち切る."""
    advice, reason = parse_advice(advice_json(action='wait', wait_sec=120.0),
                                  REQUEST, POLICY)

    assert reason == ''
    assert advice.action == ACTION_WAIT
    assert advice.wait_sec == POLICY.max_wait_sec


def test_wait_advice_without_wait_time_uses_default() -> None:
    """待機時間が得られない場合は既定値を使う."""
    advice, _ = parse_advice('{"action": "wait", "confidence": 0.8}', REQUEST, POLICY)

    assert advice.wait_sec == POLICY.default_wait_sec


def test_offset_is_clamped_to_the_side_limit() -> None:
    """助言が上限を超える場合、その側の上限で打ち切る."""
    advice, _ = parse_advice(advice_json(side='right', offset_m=2.0), REQUEST, POLICY)

    assert advice.side == SIDE_RIGHT
    assert advice.offset_m == pytest.approx(REQUEST.max_right_offset_m)


def test_offset_is_raised_to_the_minimum() -> None:
    """最小オフセットより小さい助言は最小値まで引き上げる."""
    advice, _ = parse_advice(advice_json(offset_m=0.05), REQUEST, POLICY)

    assert advice.offset_m == pytest.approx(REQUEST.min_offset_m)


def test_side_without_room_is_rejected() -> None:
    """上限が最小オフセットに満たない側の助言は採用しない.

    上限を下回る回避量では経路境界を越える恐れがあるため、丸めて採用せず不採用とする。
    呼び出し側は従来の回避へ退避する。
    """
    request = AdviceRequest(min_offset_m=0.35, max_left_offset_m=0.2,
                            max_right_offset_m=0.0)

    advice, reason = parse_advice(advice_json(side='left'), request, POLICY)

    assert advice is None
    assert reason == REASON_NO_SPACE


def test_low_confidence_advice_is_rejected() -> None:
    """確信度が下限に満たない助言は採用しない."""
    advice, reason = parse_advice(advice_json(confidence=0.2), REQUEST, POLICY)

    assert advice is None
    assert reason == REASON_CONFIDENCE


def test_missing_confidence_is_treated_as_schema_error() -> None:
    """確信度が無い応答はスキーマ違反とする（既定値で採用しない）."""
    advice, reason = parse_advice('{"action": "wait"}', REQUEST, POLICY)

    assert advice is None
    assert reason == REASON_SCHEMA


def test_unknown_action_is_rejected() -> None:
    """定義されていない行動種別は採用しない."""
    advice, reason = parse_advice(advice_json(action='follow'), REQUEST, POLICY)

    assert advice is None
    assert reason == REASON_SCHEMA


def test_empty_output_is_rejected() -> None:
    """出力が空なら不採用とする（CLI の失敗と区別しない）."""
    assert parse_advice('   ', REQUEST, POLICY) == (None, REASON_EMPTY)


def test_non_json_output_is_rejected() -> None:
    """JSON を含まない出力は不採用とする."""
    advice, reason = parse_advice('左へ 0.8 m 避けてください', REQUEST, POLICY)

    assert advice is None
    assert reason == REASON_PARSE


def test_json_in_code_fence_with_surrounding_text_is_extracted() -> None:
    """囲み文や前後の説明が付いた出力からも JSON を取り出す.

    CLI は説明を添えることがある。出力形式の指示に従わなかった場合でも、
    値が取り出せるなら採用する。
    """
    text = f'判断しました。\n```json\n{advice_json()}\n```\n以上です。'

    advice, reason = parse_advice(text, REQUEST, POLICY)

    assert reason == ''
    assert advice.side == SIDE_LEFT


def test_extract_json_skips_invalid_leading_braces() -> None:
    """壊れた JSON の後に正しい JSON がある場合は後者を採る."""
    payload = extract_json_object('{壊れている} ' + advice_json())

    assert payload['action'] == 'avoid'


def test_clamp_rejects_non_finite_offset() -> None:
    """有限でないオフセットは採用しない."""
    assert clamp_offset(SIDE_LEFT, float('inf'), REQUEST) == (SIDE_NONE, 0.0)


def test_clamp_uses_absolute_value_with_the_given_side() -> None:
    """符号付きで渡された値は絶対値を採り、方向は side を正とする."""
    side, offset = clamp_offset(SIDE_RIGHT, -0.4, REQUEST)

    assert side == SIDE_RIGHT
    assert offset == pytest.approx(0.4)


def test_prompt_contains_limits_and_output_format() -> None:
    """要求文に採用可能な範囲、観測値、出力形式の指示が含まれる."""
    prompt = build_prompt(REQUEST, 'layers.png', 'camera.png')

    assert 'layers.png' in prompt
    assert 'camera.png' in prompt
    assert 'layer=urg nearest_x_m=0.70' in prompt
    assert '0.35' in prompt and '1.00' in prompt
    assert 'JSON オブジェクトを 1 つだけ出力する' in prompt


def test_prompt_omits_missing_images() -> None:
    """画像が無い場合でも要求文を作れる（観測値だけで判断させる）."""
    prompt = build_prompt(REQUEST, None, None)

    assert 'LiDAR 水平スライス画像' not in prompt
    assert '画像は取得できていない' in prompt
    assert 'front_clearance_m=0.70' in prompt
