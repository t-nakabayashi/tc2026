# -*- coding: utf-8 -*-
"""AI 助言の要求文の組み立てと、応答の検証・クランプを行う ROS 非依存のコア.

助言は提案値であり、そのまま制御へ渡さない。本モジュールは受け取った文字列から
JSON を取り出し、スキーマと値域を検査し、採用可能な範囲へクランプした結果だけを返す。
検査に通らない場合は理由を付けて不採用とし、呼び出し側は従来の回避へ退避する。
"""

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple
import json
import math
import re

# 行動種別。tc_route_msgs/AvoidanceAdvice の定数と対応する。
ACTION_UNKNOWN = 0
ACTION_AVOID = 1
ACTION_WAIT = 2

SIDE_RIGHT = -1
SIDE_NONE = 0
SIDE_LEFT = 1

_ACTION_BY_NAME = {'avoid': ACTION_AVOID, 'wait': ACTION_WAIT}
_SIDE_BY_NAME = {'left': SIDE_LEFT, 'right': SIDE_RIGHT, 'none': SIDE_NONE}

# 不採用の理由。route_follower のログと `AvoidanceAdvice.request_id` で対応付ける。
REASON_EMPTY = 'empty_output'
REASON_PARSE = 'parse_error'
REASON_SCHEMA = 'schema_error'
REASON_CONFIDENCE = 'low_confidence'
REASON_NO_SPACE = 'no_feasible_offset'


@dataclass(frozen=True)
class AdviceRequest:
    """助言を求めるときの状況.

    Attributes:
        waypoint_label (str): 追従中の waypoint のラベル.
        front_clearance_m (float): 観測した前方余裕距離 [m].
        hint_left_offset_m (float): obstacle_monitor の左オフセット中央値 [m].
        hint_right_offset_m (float): obstacle_monitor の右オフセット中央値 [m].
        min_offset_m (float): 採用可能な最小オフセット [m].
        max_left_offset_m (float): 左へ採用可能な最大オフセット [m].
        max_right_offset_m (float): 右へ採用可能な最大オフセット [m].
        context_lines (Sequence[str]): 層ごとの観測値を表す構造化テキスト.
    """

    waypoint_label: str = ''
    front_clearance_m: float = math.inf
    hint_left_offset_m: float = 0.0
    hint_right_offset_m: float = 0.0
    min_offset_m: float = 0.35
    max_left_offset_m: float = 0.0
    max_right_offset_m: float = 0.0
    context_lines: Sequence[str] = field(default_factory=tuple)


@dataclass(frozen=True)
class AdvicePolicy:
    """助言を採用する条件.

    Attributes:
        min_confidence (float): 採用に必要な確信度の下限.
        default_wait_sec (float): 待機時間が得られなかった場合に使う値 [s].
        max_wait_sec (float): 待機時間の上限 [s].
    """

    min_confidence: float = 0.5
    default_wait_sec: float = 10.0
    max_wait_sec: float = 30.0


@dataclass
class Advice:
    """検証とクランプを終えた助言."""

    action: int = ACTION_UNKNOWN
    side: int = SIDE_NONE
    offset_m: float = 0.0
    wait_sec: float = 0.0
    is_moving: bool = False
    confidence: float = 0.0
    obstacle_class: str = ''
    rationale: str = ''


def extract_json_object(text: str) -> Optional[Dict[str, Any]]:
    """文字列から JSON オブジェクトを 1 つ取り出す.

    CLI は囲み文のコードブロックや前後の説明を付けることがあるため、最初に現れる
    対応の取れた `{...}` を走査して取り出す。見つからない場合は None を返す。
    """
    if not text:
        return None
    stripped = re.sub(r'^\s*```(?:json)?|```\s*$', '', text.strip(),
                      flags=re.MULTILINE)
    start = stripped.find('{')
    while start >= 0:
        depth = 0
        in_string = False
        escaped = False
        for index in range(start, len(stripped)):
            char = stripped[index]
            if in_string:
                if escaped:
                    escaped = False
                elif char == '\\':
                    escaped = True
                elif char == '"':
                    in_string = False
                continue
            if char == '"':
                in_string = True
            elif char == '{':
                depth += 1
            elif char == '}':
                depth -= 1
                if depth == 0:
                    try:
                        value = json.loads(stripped[start:index + 1])
                    except json.JSONDecodeError:
                        break
                    return value if isinstance(value, dict) else None
        start = stripped.find('{', start + 1)
    return None


def parse_advice(text: str, request: AdviceRequest,
                 policy: AdvicePolicy) -> Tuple[Optional[Advice], str]:
    """CLI の出力から助言を取り出し、検証とクランプを行う.

    Args:
        text (str): CLI の標準出力.
        request (AdviceRequest): 要求時の状況. クランプの上下限に使う.
        policy (AdvicePolicy): 採用条件.

    Returns:
        Tuple[Optional[Advice], str]: 採用できる助言と空文字、または None と不採用理由.
    """
    if text is None or not text.strip():
        return None, REASON_EMPTY
    payload = extract_json_object(text)
    if payload is None:
        return None, REASON_PARSE

    action = _ACTION_BY_NAME.get(str(payload.get('action', '')).strip().lower())
    if action is None:
        return None, REASON_SCHEMA

    confidence = _as_float(payload.get('confidence'), default=None)
    if confidence is None:
        return None, REASON_SCHEMA
    confidence = min(max(confidence, 0.0), 1.0)
    if confidence < policy.min_confidence:
        return None, REASON_CONFIDENCE

    advice = Advice(
        action=action,
        confidence=confidence,
        is_moving=bool(payload.get('is_moving', False)),
        obstacle_class=str(payload.get('obstacle_class', ''))[:64],
        rationale=str(payload.get('rationale', ''))[:512],
    )

    if action == ACTION_WAIT:
        wait = _as_float(payload.get('wait_sec'), default=policy.default_wait_sec)
        advice.wait_sec = min(max(wait, 0.0), policy.max_wait_sec)
        return advice, ''

    side = _SIDE_BY_NAME.get(str(payload.get('side', '')).strip().lower())
    if side is None:
        return None, REASON_SCHEMA
    offset = _as_float(payload.get('offset_m'), default=None)
    if offset is None:
        return None, REASON_SCHEMA

    side, offset = clamp_offset(side, offset, request)
    if side == SIDE_NONE:
        return None, REASON_NO_SPACE
    advice.side = side
    advice.offset_m = offset
    return advice, ''


def clamp_offset(side: int, offset_m: float, request: AdviceRequest) -> Tuple[int, float]:
    """助言されたオフセットを採用可能な範囲へ収める.

    上限は要求側が決めた左右それぞれの最大値であり、助言がこれを超える場合は上限で
    打ち切る。上限が最小オフセットに満たない側は採用できないため `SIDE_NONE` を返す。
    符号付きの値が来た場合は絶対値を採り、方向は `side` を正とする。

    Args:
        side (int): 助言された方向.
        offset_m (float): 助言された横オフセット [m].
        request (AdviceRequest): 採用可能な範囲を持つ要求.

    Returns:
        Tuple[int, float]: 採用する方向と横オフセット [m].
    """
    if side == SIDE_NONE:
        return SIDE_NONE, 0.0
    limit = (request.max_left_offset_m if side == SIDE_LEFT
             else request.max_right_offset_m)
    limit = max(float(limit), 0.0)
    if limit < request.min_offset_m or limit <= 0.0:
        return SIDE_NONE, 0.0
    value = abs(float(offset_m))
    if not math.isfinite(value):
        return SIDE_NONE, 0.0
    value = max(value, float(request.min_offset_m))
    return side, min(value, limit)


def build_prompt(request: AdviceRequest, layer_image: Optional[str],
                 camera_image: Optional[str]) -> str:
    """CLI へ渡す要求文を組み立てる.

    画像は読み取るファイルのパスとして渡し、寸法は本文の数値で渡す。画像から距離を
    読ませないのは、読み取り精度に制御が依存しないようにするためである。
    出力は JSON 1 個に限定し、前後の説明を付けないことを明示する。
    """
    lines: List[str] = [
        '自律移動ロボットが前方の障害物で停止した。回避の判断を助けてほしい。',
        '',
        '## 入力',
    ]
    if layer_image:
        lines += [
            f'- LiDAR 水平スライス画像: {layer_image}',
            '  左から順に床上高さの低い層。各層は同じ縮尺・同じ原点で、画像下端中央が'
            'ロボット中心、上が進行方向、左が車体左側。赤点が障害物、黒枠が車体、'
            'グリッドは 1 m 間隔。',
        ]
    if camera_image:
        lines += [f'- フロントカメラ画像: {camera_image}']
    if not layer_image and not camera_image:
        lines += ['- 画像は取得できていない。以下の観測値だけで判断する。']
    lines += [
        '',
        '## 観測値（画像より優先してこの数値を使う）',
    ]
    lines += [f'- {line}' for line in request.context_lines]
    lines += [
        f'- front_clearance_m={_fmt(request.front_clearance_m)}',
        f'- hint_left_offset_m={_fmt(request.hint_left_offset_m)}',
        f'- hint_right_offset_m={_fmt(request.hint_right_offset_m)}',
        f'- 採用可能な左オフセット: {_fmt(request.min_offset_m)} 〜 '
        f'{_fmt(request.max_left_offset_m)} m',
        f'- 採用可能な右オフセット: {_fmt(request.min_offset_m)} 〜 '
        f'{_fmt(request.max_right_offset_m)} m',
        '',
        '## 判断の方針',
        '- 静止した障害物（ロードコーン、車止めのようなポール、草など）は回避する。',
        '- 人や他のロボットのように動く障害物は待機し、状況が改善するのを待つ。',
        '- 同じ方向へ進む他のロボットは追い越さない。待機を選ぶ。',
        '- 採用可能な範囲に収まらない回避量は指定しない。',
        '- 判断できない場合は confidence を低くする。',
        '',
        '## 出力',
        'JSON オブジェクトを 1 つだけ出力する。前後に説明や囲み文字を付けない。',
        '{"action": "avoid" または "wait", "side": "left" / "right" / "none", '
        '"offset_m": 数値, "wait_sec": 数値, "obstacle_class": 文字列, '
        '"is_moving": 真偽値, "confidence": 0.0〜1.0, "rationale": 文字列}',
        'action が "wait" のとき side と offset_m は無視される。',
        'obstacle_class は cone / pole / robot / person / grass / unknown のいずれか。',
        'rationale は 1 文で簡潔に書く。',
    ]
    return '\n'.join(lines)


def _as_float(value: Any, default: Optional[float]) -> Optional[float]:
    """数値へ変換する。変換できない場合は default を返す."""
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    if math.isnan(result):
        return default
    return result


def _fmt(value: float) -> str:
    return 'inf' if not math.isfinite(value) else f'{value:.2f}'
