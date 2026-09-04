# CLAUDE.md

このファイルは、Claude Code がこの ROS 2 ワークスペースで作業する際の共通指示である。
GitHub リポジトリに登録し、開発メンバ間で共通利用することを前提とする。

このファイルは常に読み込まれるため、常にどんな作業にも適用される方針のみを記載する。
特定の作業に関する詳細な手順・規約は `.claude/skills/` 配下の各スキルに切り出しており、
作業内容に応じて Claude Code が自動的に該当スキルを読み込む。

- ROS 2 パッケージの新規作成・ファイル配置・命名に迷ったら `ros2-package-layout` スキル
- `colcon build` の実行・ビルド失敗調査・pytest の追加/実行は `build-and-test` スキル
- ローカルでの `ros2 run` / `ros2 launch` などの実行確認は `ros2-local-run` スキル
- パッケージ詳細設計書や `docs/` 配下の技術文書の作成・更新は `design-doc-template` スキル
- GUI・フロントエンド実装は `frontend-design` スキル（Anthropic 公式プラグイン。導入方法は
  [README.md](README.md) の「Claude Code スキル設定」を参照）

---

# Common Instructions

- 回答、作業計画、作業報告、警告、補足説明は日本語で行う。
- 技術用語、API 名、ROS メッセージ名、クラス名、関数名は英語表記を維持してよい。
- 実装や修正にあたって、省略、要約、抽象化で必要な処理を落とさない。
- 変更前に、対象ファイル、変更方針、確認方法を簡潔に説明する。
- 機能追加・修正時は、既存のディレクトリ構成、命名規則、コメント粒度、docstring 形式、ログ出力方針と整合させる。
- 関係ないコメント、空行、フォーマット、命名を変更しない。
- リポジトリ外のファイルやディレクトリを前提にした実装、設定、ドキュメントを追加しない。
- 絶対パスは記述しない。必要な場合は、リポジトリルートからの相対パスまたはプレースホルダを使う。
- `build/`, `install/`, `log/`, `.git/` は編集しない。ただし `colcon build` の再確認に
  必要なクリーンとして、`build/`, `install/`, `log/` 配下の生成物を削除することは許容する。
- `sudo`、ファイル削除、破壊的操作は明示指示がある場合のみ行う。
- 生成物、キャッシュ、一時ファイルをリポジトリ管理対象として追加しない。
- このリポジトリは ROS 2 の colcon ワークスペースであり、`build/`, `install/`, `log/` や
  submodule (`third_party/` 配下) がワークスペース直下のパスに依存する。git worktree による
  並行作業は、ビルド成果物や submodule の状態がワークツリーごとに分裂し不整合の原因になるため
  基本的に使わない。並行作業が必要な場合はブランチ切り替えや別 clone を検討する。
- ドキュメント本文やコードコメントには、セッション中のユーザーとのやり取り、指摘を受けて
  修正した経緯、実装当初の判断ミスなど、作業プロセスに関する記述を残さない。決定した仕様・
  現状・理由のみを記載する（理由: 会話の経緯は読み手にとってノイズであり、コードや文書の
  寿命を通じて陳腐化する）。

---

# Workspace Structure Instructions

このワークスペースは、ROS 2 の colcon ワークスペースとして扱う。

```text
.
├── CLAUDE.md
├── README.md
├── requirements.txt
├── .claude/
│   └── skills/
├── docs/
├── src/
├── build/
├── install/
└── log/
```

- `src/` 配下に ROS 2 パッケージを配置する。
- `src/<package_name>/` を 1 つの ROS 2 パッケージの基本単位として扱う。
- `requirements.txt` はワークスペース共通の Python pip 依存関係を管理するファイルとして扱う。
- ワークスペース全体に関わる資料は `docs/` 配下に置く。
- 特定パッケージに閉じる資料は `src/<package_name>/docs/` 配下に置く。
- `build/`, `install/`, `log/` は colcon の生成物として扱い、直接修正しない。ただし
  ビルド確認のためのクリーンでは削除してよい。
- 新規ファイルを追加する場合は、パッケージ構成の推奨レイアウト（`ros2-package-layout`
  スキル参照）に従い、配置理由を作業報告に含める。

---

# 実行環境（Python venv）

ROS関連Pythonパッケージ（rclpy, python3-pyqt5等）はaptでシステムPythonへ
入る一方、requirements.txtのpipパッケージはUbuntu 24.04のPEP 668制約により
システムPythonへ直接pip installできない。そのため `--system-site-packages`
を有効にしたPython venvの使用を前提とする。配置場所は開発者ごとに異なって
よく、本ファイルやリポジトリ内のスクリプトで特定の絶対パスを前提にしない。

- Python/pytest/colcon等を実行する前に、venvが有効化されているか
  （`$VIRTUAL_ENV`が設定されているか）を確認する。
- 有効化方法が不明な場合はパスを推測せず、ユーザーに確認する。

---

# Python Instructions

## Style

- Google Python Style Guide を基本にする。
- PEP 8 に従う。
- 1 行は原則 100 文字以内にする。
- クラス間は空行 2 行、関数間は空行 1 行を基本とする。
- 既存ファイルの書式がある場合は、既存書式との整合を優先する。

## Type Hints

- すべての関数・メソッドに可能な限り型ヒントを付ける。
- 戻り値がない場合は `-> None` を明記する。
- 型が複雑になる場合は、読みやすい型定義や補助型を検討する。

## Naming

- クラス名は `PascalCase` とする。
- 関数名、メソッド名、変数名は `snake_case` とする。
- 定数は `UPPER_CASE` とする。
- 公開 API 名や ROS interface 名の命名規則は `ros2-package-layout` スキルを参照する。

## Comments and Docstrings

- コメント、docstring、ログメッセージは日本語を優先する。
- 英語は固有名詞、外部 API、ROS メッセージ名など技術的に必要な箇所に限定する。
- コメントは、処理の目的、背景、例外条件、アルゴリズム選定理由が分かる粒度で記述する。
- 自明な処理をなぞるだけのコメントは追加しない。
- 既存の日本語コメント・docstring を不要に書き換えない。

Docstring は以下の形式を基本とする。

```python
def compute_distance(self, pose_a: Pose, pose_b: Pose) -> float:
    """2 点間のユークリッド距離を算出する.

    Args:
        pose_a (Pose): 始点の位置姿勢.
        pose_b (Pose): 終点の位置姿勢.

    Returns:
        float: pose_a から pose_b までの距離 [m].
    """
```

## Imports

import 順序は以下を基本とする。

1. 標準ライブラリ
2. 外部ライブラリ
3. ROS 2 関連ライブラリ
4. 同一パッケージ内モジュール

```python
import math

import numpy as np

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node

from <package_name>.<module_name> import SomeClass
```

## ROS 2 Python Node Policy

- ROS ノードでは `self.get_logger()` を使う。
- ノードは、起動、主要状態遷移、終了時に必要なログを出す。
- 内部状態確認は `debug` を使う。
- 通常の進行報告は `info` を使う。
- 想定内の異常や再試行は `warn` を使う。
- 明確な障害は `error` を使う。
- 致命的停止は `fatal` を使う。
- QoS、パラメータ、トピック名、タイマー周期は、既存設計または関連ドキュメントと整合させる。

---

# C++ Instructions

- C++ 実装では、既存の CMake 設定、依存関係、命名規則に従う。
- C++ ソースはパッケージ内の `src/` 配下に配置する。
- 依存関係の追加・確認手順は `ros2-package-layout` スキルを参照する。
- ROS 2 の publisher/subscriber/service/action、パラメータ、QoS は関連ドキュメントと整合させる。

---

# 実機安全規則（要約）

安全規則には性質の異なる 2 段階がある。詳細と最新の正は常に `ros2-local-run` スキルを参照する。

- **無条件で禁止**（ユーザーの指示があっても行わない）: `ypspur_ros2` の起動、
  `ypspur-coordinator` の起動、実機のロボットを実際に動かす操作、`/cmd_vel` を
  実機 driver に接続する構成の起動。実施してよいのは各ノードの単体動作確認と
  シミュレーションによる結合動作確認に限る。
- **ユーザーが明示的に指示した場合のみ行う**: `rtk_gps_um982` などの実センサ、実カメラ、
  LiDAR、Gazebo、RViz、外部デバイスを使う動作確認。
- ローカル環境での `ros2 run` / `ros2 launch` / `ros2 topic` / `ros2 service` などの
  実行確認手順、ログ配置、GUI/headless の判断、トラブルシュートの詳細は `ros2-local-run`
  スキルを参照する。

---

# スキル運用方針

- 新しいスキルを追加する際は、一般的な作法（見た目のデザイン、可視化、文書生成などで
  既に汎用スキルがカバーしている内容）を重複させない。
- プロジェクト固有スキルは、このリポジトリでしか分からない制約（データ契約、ファイル配置、
  安全規則、既存規約との整合）に絞って記述する。
