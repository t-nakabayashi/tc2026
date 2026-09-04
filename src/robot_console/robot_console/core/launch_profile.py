"""起動対象ノードのprofile定義を読み込み・保持するモジュール。

起動対象はコード内の固定リストではなく `config/node_launch_profiles.yaml` から
生成する。profile追加・変更時に本モジュールおよびUIコードの改修が不要になるよう、
profile定義に必要な情報はすべてYAML側に持たせる。
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional

try:
    from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
except ImportError:  # pragma: no cover - ament_index_python が無い環境向けフォールバック
    PackageNotFoundError = Exception  # type: ignore[assignment,misc]
    get_package_share_directory = None  # type: ignore[assignment]

try:
    import yaml
except ImportError:  # pragma: no cover - PyYAML が無い環境では JSON のみ対応
    yaml = None  # type: ignore[assignment]

from ..utils import NodeLaunchStatus

PACKAGE_NAME = 'robot_console'
DEFAULT_PROFILE_RELATIVE_PATH = Path('config') / 'node_launch_profiles.yaml'


@dataclass
class LaunchProfile:
    """起動対象ノード1件分のprofile定義。"""

    profile_id: str
    category: str
    display_name: str
    package: str
    launch_file: str
    param_argument: Optional[str] = None
    param_package: Optional[str] = None
    default_param: Optional[str] = None
    launch_order: int = 0
    startup_group: Optional[str] = None
    health_topics: List[str] = field(default_factory=list)
    alternate_launch_file: Optional[str] = None
    launch_toggle_label: Optional[str] = None
    simulator_package: Optional[str] = None
    simulator_launch_file: Optional[str] = None
    simulator_argument_map: Dict[str, str] = field(default_factory=dict)
    user_arguments: List[str] = field(default_factory=list)
    default_arguments: Dict[str, str] = field(default_factory=dict)


class LaunchProfileError(Exception):
    """profile定義の読み込み・検証エラー。"""


def _default_profile_path() -> Optional[Path]:
    """既定のprofile定義ファイルパスを解決する。"""

    if get_package_share_directory is not None:
        try:
            share_dir = Path(get_package_share_directory(PACKAGE_NAME))
            candidate = share_dir / DEFAULT_PROFILE_RELATIVE_PATH
            if candidate.exists():
                return candidate
        except PackageNotFoundError:
            pass
    source_candidate = (
        Path(__file__).resolve().parent.parent.parent / DEFAULT_PROFILE_RELATIVE_PATH
    )
    if source_candidate.exists():
        return source_candidate
    return None


def _parse_profile_entry(entry: Dict[str, object], *, index: int) -> LaunchProfile:
    """YAML中の1エントリを LaunchProfile へ変換する。"""

    required_fields = ('profile_id', 'category', 'display_name', 'package', 'launch_file')
    missing = [name for name in required_fields if not entry.get(name)]
    if missing:
        raise LaunchProfileError(
            f"profiles[{index}] に必須項目が不足しています: {', '.join(missing)}"
        )

    health_topics = entry.get('health_topics') or []
    if not isinstance(health_topics, list):
        raise LaunchProfileError(f"profiles[{index}].health_topics はリストである必要があります")

    user_arguments = entry.get('user_arguments') or []
    if not isinstance(user_arguments, list):
        raise LaunchProfileError(f"profiles[{index}].user_arguments はリストである必要があります")

    default_arguments = entry.get('default_arguments') or {}
    if not isinstance(default_arguments, dict):
        raise LaunchProfileError(f"profiles[{index}].default_arguments は辞書である必要があります")

    simulator_argument_map = entry.get('simulator_argument_map') or {}
    if not isinstance(simulator_argument_map, dict):
        raise LaunchProfileError(
            f"profiles[{index}].simulator_argument_map は辞書である必要があります"
        )

    return LaunchProfile(
        profile_id=str(entry['profile_id']),
        category=str(entry['category']),
        display_name=str(entry['display_name']),
        package=str(entry['package']),
        launch_file=str(entry['launch_file']),
        param_argument=entry.get('param_argument'),
        param_package=entry.get('param_package'),
        default_param=entry.get('default_param'),
        launch_order=int(entry.get('launch_order', 0)),
        startup_group=entry.get('startup_group'),
        health_topics=[str(topic) for topic in health_topics],
        alternate_launch_file=entry.get('alternate_launch_file'),
        launch_toggle_label=entry.get('launch_toggle_label'),
        simulator_package=entry.get('simulator_package'),
        simulator_launch_file=entry.get('simulator_launch_file'),
        simulator_argument_map={str(k): str(v) for k, v in simulator_argument_map.items()},
        user_arguments=[str(arg) for arg in user_arguments],
        default_arguments={str(k): str(v) for k, v in default_arguments.items()},
    )


class LaunchProfileStore:
    """起動対象profile定義を読み込み・保持する。"""

    def __init__(self, profile_path: Optional[Path] = None) -> None:
        self._profile_path = profile_path or _default_profile_path()
        self._profiles: List[LaunchProfile] = []
        self._validation_errors: List[str] = []

    @property
    def profile_path(self) -> Optional[Path]:
        """読み込み対象のprofile定義ファイルパスを返す。"""

        return self._profile_path

    @property
    def validation_errors(self) -> List[str]:
        """読み込み時に検出した検証エラーの一覧を返す。"""

        return list(self._validation_errors)

    def load(self) -> List[LaunchProfile]:
        """profile定義ファイルを読み込み LaunchProfile の一覧を返す。"""

        self._profiles = []
        self._validation_errors = []
        if self._profile_path is None or not self._profile_path.exists():
            self._validation_errors.append(
                f"profile定義ファイルが見つかりません: {self._profile_path}"
            )
            return self._profiles

        raw_text = self._profile_path.read_text(encoding='utf-8')
        content = self._parse_text(raw_text)
        if content is None:
            self._validation_errors.append(
                f"profile定義ファイルを解析できません: {self._profile_path}"
            )
            return self._profiles
        if not isinstance(content, dict) or not isinstance(content.get('profiles'), list):
            self._validation_errors.append(
                "profile定義ファイルの形式が不正です（'profiles' キーのリストが必要です）"
            )
            return self._profiles

        profiles: List[LaunchProfile] = []
        seen_ids = set()
        for index, entry in enumerate(content['profiles']):
            if not isinstance(entry, dict):
                self._validation_errors.append(f"profiles[{index}] は辞書である必要があります")
                continue
            try:
                profile = _parse_profile_entry(entry, index=index)
            except LaunchProfileError as exc:
                self._validation_errors.append(str(exc))
                continue
            if profile.profile_id in seen_ids:
                self._validation_errors.append(
                    f"profile_id が重複しています: {profile.profile_id}"
                )
                continue
            seen_ids.add(profile.profile_id)
            profiles.append(profile)

        profiles.sort(key=lambda profile: profile.launch_order)
        self._profiles = profiles
        return self._profiles

    @staticmethod
    def _parse_text(raw_text: str) -> Optional[Dict[str, object]]:
        """YAML優先、失敗時はJSONとして解析する。"""

        if yaml is not None:
            try:
                content = yaml.safe_load(raw_text)
                if content is not None:
                    return content
            except yaml.YAMLError:  # type: ignore[attr-defined]
                pass
        try:
            return json.loads(raw_text)
        except json.JSONDecodeError:
            return None

    @property
    def profiles(self) -> List[LaunchProfile]:
        """読み込み済みprofile一覧を返す。"""

        return list(self._profiles)

    def get(self, profile_id: str) -> Optional[LaunchProfile]:
        """profile_idに対応するLaunchProfileを返す。"""

        for profile in self._profiles:
            if profile.profile_id == profile_id:
                return profile
        return None


@dataclass
class LaunchProfileState:
    """profile1件分の起動管理状態（`LaunchManager` の状態通知を反映する）。"""

    profile_id: str
    status: NodeLaunchStatus = NodeLaunchStatus.STOPPED
    process_id: Optional[int] = None
    simulator_status: NodeLaunchStatus = NodeLaunchStatus.STOPPED
    simulator_process_id: Optional[int] = None
    selected_param: Optional[str] = None
    use_alternate_launch: bool = False
    simulator_enabled: bool = False
    override_inputs: Dict[str, str] = field(default_factory=dict)
    error_message: str = ''
    last_action_time: Optional[datetime] = None


def build_initial_states(profiles: List[LaunchProfile]) -> Dict[str, LaunchProfileState]:
    """LaunchProfile一覧から初期状態の LaunchProfileState 辞書を生成する。"""

    return {
        profile.profile_id: LaunchProfileState(
            profile_id=profile.profile_id,
            selected_param=profile.default_param,
            use_alternate_launch=False,
            override_inputs={key: profile.default_arguments.get(key, '') for key in profile.user_arguments},
        )
        for profile in profiles
    }


def resolve_effective_overrides(
    profile: LaunchProfile, state: LaunchProfileState
) -> Dict[str, str]:
    """state.override_inputs とprofile.default_argumentsから実効override値を求める。

    state側に空でない入力があればそれを優先し、無ければprofile既定値を使う。
    どちらも無い引数は起動コマンドへ含めない。
    """

    effective: Dict[str, str] = {}
    for key in profile.user_arguments:
        value = state.override_inputs.get(key, '').strip()
        if not value:
            value = profile.default_arguments.get(key, '')
        if value:
            effective[key] = value
    return effective


def resolve_param_path(profile: LaunchProfile, param_path: Optional[str]) -> Optional[str]:
    """`param_path` をパッケージ共有ディレクトリ基準の絶対パスへ解決する。

    `LaunchProfile.default_param` / `LaunchProfileState.selected_param` は
    `'params/tsukuba.yaml'` のようなパッケージ相対のフラグメントで保持される。
    `ros2 launch` の各launchファイルはこの文字列をそのまま
    `Node(parameters=[...])` や独自の `open()` へ渡すため、相対パスのままでは
    サブプロセスの作業ディレクトリ次第で解決に失敗する。実際に起動する直前に
    本関数で絶対パスへ解決する。

    既に絶対パスの場合、`param_path` が未指定の場合、または
    `ament_index_python`/対象パッケージが利用できない場合は、
    `param_path` をそのまま返す（呼び出し元やlaunchファイル側の既定解決に委ねる）。
    """

    if not param_path or Path(param_path).is_absolute():
        return param_path
    if get_package_share_directory is None:
        return param_path
    package_name = profile.param_package or profile.package
    try:
        share_dir = Path(get_package_share_directory(package_name))
    except PackageNotFoundError:
        return param_path
    return str(share_dir / param_path)


def build_launch_args(
    profile: LaunchProfile,
    *,
    param_path: Optional[str] = None,
    use_alternate: bool = False,
    overrides: Optional[Dict[str, str]] = None,
) -> List[str]:
    """profileから `ros2 launch` 相当のコマンド引数列を組み立てる。

    `LaunchManager`（実行）と起動・設定タブの起動内容プレビュー（表示）の
    双方から参照される、コマンド組み立ての唯一の実装である。
    """

    launch_file = profile.launch_file
    if use_alternate and profile.alternate_launch_file:
        launch_file = profile.alternate_launch_file
    args = ['ros2', 'launch', profile.package, launch_file]
    if param_path and profile.param_argument:
        args.append(f'{profile.param_argument}:={param_path}')
    if overrides:
        for key, value in overrides.items():
            args.append(f'{key}:={value}')
    return args


def build_simulator_launch_args(
    profile: LaunchProfile, overrides: Optional[Dict[str, str]] = None
) -> List[str]:
    """profileのsimulator代替launch向けコマンド引数列を組み立てる。

    `overrides` のうち `profile.simulator_argument_map` に掲載のあるキーだけを
    変換して転送する（simulator側launchが受け付けない引数を渡さないため）。
    """

    simulator_package = profile.simulator_package or profile.package
    args = ['ros2', 'launch', simulator_package, profile.simulator_launch_file or '']
    if overrides:
        for key, value in overrides.items():
            sim_key = profile.simulator_argument_map.get(key)
            if sim_key is None:
                continue
            args.append(f'{sim_key}:={value}')
    return args
