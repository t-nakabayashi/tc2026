"""つくば2026 本走行コースの登録内容を検査する.

選択課題Aに対応するルートと対応しないルートの双方を扱う。

登録した経路が route_survey の成果物と同一であること、route_config.yaml から
全域を固定ブロックとして組み立てられること、起動カードが実在するファイルを
指していることを確認する。経路の値そのもの（回避幅・停止位置）の妥当性は
実機走行で確認する範囲であり、ここでは扱わない。
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest
import yaml

PACKAGE_DIR = Path(__file__).resolve().parents[1]
WORKSPACE_DIR = PACKAGE_DIR.parents[1]
sys.path.insert(0, str(PACKAGE_DIR))
sys.path.insert(0, str(WORKSPACE_DIR / "src" / "geo_pose_converter"))

from geo_pose_converter.geo_core import load_projection_config_from_yaml  # noqa: E402
from route_planner.route_builder import RouteBuilder  # noqa: E402

ROUTE_DIR = PACKAGE_DIR / "routes" / "tsukuba2026"
# 経路は選択課題Aのゲート位置 (WP0738) で2ファイルに分かれている。連結すると
# route_survey の成果物と一致する。
ROUTE_CSV_PARTS = (ROUTE_DIR / "fixed" / "part1.csv", ROUTE_DIR / "fixed" / "part2.csv")
SPLIT_LABEL = "WP0738"
SURVEY_CSV = (
    WORKSPACE_DIR / "src" / "route_survey" / "results" / "tsukuba2026_waypoints.csv"
)
PROJECTION_YAML = (
    WORKSPACE_DIR / "src" / "geo_pose_converter" / "params" / "default.yaml"
)

# 最大waypoint間隔。build_route.py が0.75 m以下へ等分補間する。
MAX_SPACING_M = 0.75


def _build(config_name: str) -> list:
    """指定した route_config から waypoint 列を組み立てる."""

    projection = load_projection_config_from_yaml(str(PROJECTION_YAML))
    builder = RouteBuilder(
        str(ROUTE_DIR / config_name),
        csv_base_dir=str(ROUTE_DIR),
        projection=projection,
    )
    builder.load()
    return builder.build_route("", "", []).waypoints


@pytest.fixture(scope="module")
def route_waypoints() -> list:
    """課題A非対応のルートの waypoint 列を返す."""

    return _build("route_config.yaml")


@pytest.fixture(scope="module")
def task_a_waypoints() -> list:
    """課題A対応のルートの waypoint 列を返す."""

    return _build("route_config_task_a.yaml")


def test_registered_csv_parts_concatenate_to_route_survey_result() -> None:
    """分割した経路CSVを連結すると route_survey の成果物と一致する."""

    head, *rest = [path.read_text(encoding="utf-8") for path in ROUTE_CSV_PARTS]
    joined = head + "".join(text.split("\n", 1)[1] for text in rest)

    assert joined == SURVEY_CSV.read_text(encoding="utf-8")


def test_split_point_is_not_a_stop_waypoint() -> None:
    """分割点が停止点でないことを確かめる.

    ブロック末尾の waypoint は姿勢が直前点からの方位で上書きされるため、停止点を
    分割点にすると停止時の向きが変わる。
    """

    last = ROUTE_CSV_PARTS[0].read_text(encoding="utf-8").strip().splitlines()[-1]
    fields = last.split(",")
    assert fields[0] == SPLIT_LABEL
    assert fields[7] == "0" and fields[8] == "0"


def test_route_config_covers_whole_course_as_fixed(route_waypoints: list) -> None:
    """2つの固定ブロックが全域として読み込まれる."""

    assert len(route_waypoints) == 3704
    assert route_waypoints[0].label == "WP0000"
    assert route_waypoints[-1].label == "E17"
    assert all(wp.segment_is_fixed for wp in route_waypoints)
    assert [wp.label for wp in route_waypoints].count(SPLIT_LABEL) == 1


def test_route_keeps_stop_attributes_and_spacing(route_waypoints: list) -> None:
    """停止属性と点間隔が生成時のまま保たれている."""

    assert sum(1 for wp in route_waypoints if wp.line_stop) == 16
    assert not any(wp.signal_stop for wp in route_waypoints)

    spacing = [
        math.dist(
            (previous.pose.position.x, previous.pose.position.y),
            (current.pose.position.x, current.pose.position.y),
        )
        for previous, current in zip(route_waypoints, route_waypoints[1:])
    ]
    assert max(spacing) <= MAX_SPACING_M + 1e-6


def test_launch_cards_point_to_existing_files() -> None:
    """route_planner と route_manager の起動カードが揃っている."""

    planner_card = yaml.safe_load(
        (PACKAGE_DIR / "params" / "tsukuba2026.yaml").read_text(encoding="utf-8")
    )["route_planner"]["ros__parameters"]
    assert (PACKAGE_DIR / planner_card["config_yaml_path"]).is_file()
    assert (PACKAGE_DIR / planner_card["csv_base_dir"]).is_dir()

    manager_card_path = (
        WORKSPACE_DIR / "src" / "route_manager" / "params" / "tsukuba2026.yaml"
    )
    manager_card = yaml.safe_load(manager_card_path.read_text(encoding="utf-8"))
    assert "route_manager" in manager_card


def test_task_a_route_inserts_variable_block_without_copying_csv(
    route_waypoints: list, task_a_waypoints: list
) -> None:
    """課題A対応のルートが、同じ固定CSVの間に可変ブロックを挟んで組み立つ."""

    task_a_config = yaml.safe_load(
        (ROUTE_DIR / "route_config_task_a.yaml").read_text(encoding="utf-8")
    )["blocks"]
    fixed_segments = [b["segment_id"] for b in task_a_config if b["type"] == "fixed"]
    assert fixed_segments == [path.name and f"fixed/{path.name}" for path in ROUTE_CSV_PARTS]

    labels = [wp.label for wp in task_a_waypoints]
    gate_first, gate_last = labels.index("G"), len(labels) - 1 - labels[::-1].index("G")
    section = task_a_waypoints[gate_first : gate_last + 1]
    # 非対応のルートはそのまま残り、分割点の直後へ課題A区間が挿入される。
    assert len(task_a_waypoints) == len(route_waypoints) + len(section)
    assert labels[gate_first - 1] == SPLIT_LABEL
    assert "O11" in labels
    assert not any(wp.segment_is_fixed for wp in section[1:-1])


def test_task_a_section_starts_at_the_split_point() -> None:
    """課題A区間の入口が分割点と同じ位置にあり、そこで左折する."""

    waypoints = _build("route_config_task_a.yaml")
    labels = [wp.label for wp in waypoints]
    gate = labels.index("G")
    split = waypoints[gate - 1]
    assert split.label == SPLIT_LABEL
    assert math.dist(
        (split.pose.position.x, split.pose.position.y),
        (waypoints[gate].pose.position.x, waypoints[gate].pose.position.y),
    ) < 0.05


def test_task_a_section_has_avoidance_width_and_no_stop(task_a_waypoints: list) -> None:
    """課題A区間は左右1.5 mの回避幅を持ち、停止点を持たない."""

    labels = [wp.label for wp in task_a_waypoints]
    gate_first, gate_last = labels.index("G"), len(labels) - 1 - labels[::-1].index("G")
    section = task_a_waypoints[gate_first + 1 : gate_last]

    assert {(wp.right_open, wp.left_open) for wp in section} == {(1.5, 1.5)}
    assert not any(wp.line_stop or wp.signal_stop for wp in section)


def test_task_a_route_keeps_spacing(task_a_waypoints: list) -> None:
    """課題Aを挟んでも点間隔が保たれる."""

    spacing = [
        math.dist(
            (previous.pose.position.x, previous.pose.position.y),
            (current.pose.position.x, current.pose.position.y),
        )
        for previous, current in zip(task_a_waypoints, task_a_waypoints[1:])
    ]
    assert max(spacing) <= MAX_SPACING_M + 1e-6
