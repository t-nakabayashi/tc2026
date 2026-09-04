"""業務モード（実行環境×走行モード）ごとの起動プリセットと起動予定管理。

profileの選択・順序（起動予定ノード一覧）は、業務モードのプリセットから
初期化しつつユーザーが個別に取捨選択できる（screen_function_design.md 3章・
4.3節）。プリセットは「推奨候補」であり強制ではないため、`LaunchPlan` は
プリセット適用後の追加・除外・並べ替えを独立して扱う。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

ENVIRONMENTS = ['実機', 'シミュレーション', '机上確認']
DRIVE_MODES = ['手動走行', '自律走行']


@dataclass(frozen=True)
class LaunchPresetEntry:
    """プリセット1件分の起動対象指定。"""

    profile_id: str
    use_simulator_alternate: bool = False
    overrides: Dict[str, str] = field(default_factory=dict)


# docs/robot_console_gui_architecture_design.md 10章「起動グループ」および
# docs/robot_console_gui_screen_function_design.md 3.1節「業務分類」に基づく。
LAUNCH_PRESETS: Dict[Tuple[str, str], List[LaunchPresetEntry]] = {
    ('実機', '手動走行'): [
        LaunchPresetEntry('ypspur_ros2'),
        LaunchPresetEntry('drive_mode_manager'),
    ],
    ('実機', '自律走行'): [
        LaunchPresetEntry('rtk_gps_um982'),
        LaunchPresetEntry('ypspur_ros2'),
        LaunchPresetEntry('drive_mode_manager'),
        LaunchPresetEntry('route_planner'),
        LaunchPresetEntry('route_manager'),
        LaunchPresetEntry('geo_pose_converter', overrides={'enable_geo_pose_converter': 'true'}),
        LaunchPresetEntry('route_follower'),
        LaunchPresetEntry('obstacle_monitor'),
        LaunchPresetEntry('robot_navigator'),
        LaunchPresetEntry('road_blockage_detector'),
        LaunchPresetEntry('traffic_signal_recognizer'),
    ],
    ('シミュレーション', '手動走行'): [
        LaunchPresetEntry('obstacle_route_sim'),
        LaunchPresetEntry('drive_mode_manager'),
    ],
    ('シミュレーション', '自律走行'): [
        LaunchPresetEntry('obstacle_route_sim'),
        LaunchPresetEntry('drive_mode_manager'),
        LaunchPresetEntry('route_planner'),
        LaunchPresetEntry('route_manager'),
        LaunchPresetEntry('geo_pose_converter'),
        LaunchPresetEntry('route_follower'),
        LaunchPresetEntry('obstacle_monitor'),
        LaunchPresetEntry('robot_navigator'),
    ],
    ('机上確認', '手動走行'): [
        LaunchPresetEntry('drive_mode_manager', overrides={'joy_input': 'ps3_joy_sim'}),
    ],
    ('机上確認', '自律走行'): [
        LaunchPresetEntry('route_planner'),
        LaunchPresetEntry('route_manager'),
        LaunchPresetEntry('geo_pose_converter'),
        LaunchPresetEntry('route_follower'),
        LaunchPresetEntry('drive_mode_manager', overrides={'joy_input': 'ps3_joy_sim'}),
        LaunchPresetEntry('robot_navigator', use_simulator_alternate=True),
        LaunchPresetEntry('obstacle_monitor', use_simulator_alternate=True),
        LaunchPresetEntry('road_blockage_detector', use_simulator_alternate=True),
        LaunchPresetEntry('traffic_signal_recognizer', use_simulator_alternate=True),
    ],
}


def get_preset(environment: str, drive_mode: str) -> List[LaunchPresetEntry]:
    """業務モードに対応するプリセットを返す。未定義の組み合わせは空リスト。"""

    return list(LAUNCH_PRESETS.get((environment, drive_mode), []))


@dataclass
class LaunchPlan:
    """一斉起動対象のprofile集合と順序を保持する（起動予定ノード一覧）。

    `selected_for_launch`（本リストに含まれるか）と `process_state`
    （`LaunchProfileState.status` が示す実際の起動状態）は独立して扱う。
    一覧からの除外は起動済みプロセスを停止しない（4.5節）。
    """

    ordered_profile_ids: List[str] = field(default_factory=list)

    def contains(self, profile_id: str) -> bool:
        """指定profileが起動予定に含まれるかを返す。"""

        return profile_id in self.ordered_profile_ids

    def add(self, profile_id: str) -> None:
        """起動予定の末尾へprofileを追加する（重複は無視）。"""

        if profile_id not in self.ordered_profile_ids:
            self.ordered_profile_ids.append(profile_id)

    def remove(self, profile_id: str) -> None:
        """起動予定からprofileを除外する。"""

        if profile_id in self.ordered_profile_ids:
            self.ordered_profile_ids.remove(profile_id)

    def move_up(self, profile_id: str) -> None:
        """起動順序を1つ前へ移動する。"""

        index = self._index_of(profile_id)
        if index is None or index == 0:
            return
        self.ordered_profile_ids[index - 1], self.ordered_profile_ids[index] = (
            self.ordered_profile_ids[index],
            self.ordered_profile_ids[index - 1],
        )

    def move_down(self, profile_id: str) -> None:
        """起動順序を1つ後ろへ移動する。"""

        index = self._index_of(profile_id)
        if index is None or index >= len(self.ordered_profile_ids) - 1:
            return
        self.ordered_profile_ids[index + 1], self.ordered_profile_ids[index] = (
            self.ordered_profile_ids[index],
            self.ordered_profile_ids[index + 1],
        )

    def apply_preset(self, entries: List[LaunchPresetEntry]) -> None:
        """プリセット内容で起動予定を置き換える。"""

        self.ordered_profile_ids = [entry.profile_id for entry in entries]

    def clear(self) -> None:
        """起動予定を空にする。"""

        self.ordered_profile_ids = []

    def _index_of(self, profile_id: str) -> Optional[int]:
        try:
            return self.ordered_profile_ids.index(profile_id)
        except ValueError:
            return None
