---
name: ros2-package-layout
description: ROS 2 パッケージ (src/<package_name>/) の推奨ディレクトリ構成、必須/条件付きファイル、命名規則、責務分担を定義する。新しい ROS 2 パッケージを追加するとき、既存パッケージに launch/params/msg/srv/docs/tests/tools などのファイルを追加するとき、ファイルの配置場所やファイル名 (*_node.py, *_core.py, test_*.py 等) に迷ったとき、パッケージの依存関係 (package.xml, setup.py, CMakeLists.txt, requirements.txt) を変更するときは必ずこのスキルを参照する。
---

# ROS 2 パッケージ構成

このワークスペースの ROS 2 パッケージは、既存パッケージ名や個別構成を前提にせず、
役割に応じて必要なものだけを置く。以下は推奨レイアウトであり、既存パッケージの
構成を不用意に崩さないことを優先する。既存の命名規則がある場合は、既存規則との
整合を新規追加より優先する。

## 推奨レイアウト

```text
src/<package_name>/
├── package.xml
├── setup.py
├── setup.cfg
├── CMakeLists.txt
├── resource/
│   └── <package_name>
├── <package_name>/
│   ├── __init__.py
│   ├── <node>_node.py
│   └── *_core.py
├── src/
├── launch/
│   └── <name>.launch.py
├── params/
│   └── <name>.yaml
├── msg/
│   └── <Name>.msg
├── srv/
│   └── <Name>.srv
├── docs/
├── tests/
│   └── test_*.py
├── tools/
├── routes/
├── maps/
├── models/
├── rviz/
└── third_party/
```

## Required Items

- `package.xml` はすべての ROS 2 パッケージで必須とする。
- `ament_python` パッケージでは、`setup.py` と `resource/<package_name>` を必須とする。
- `ament_cmake` パッケージ、C++ 実装を含むパッケージ、`msg` / `srv` を定義するパッケージでは、`CMakeLists.txt` を必須とする。
- Python モジュールを持つパッケージでは、`<package_name>/` と `<package_name>/__init__.py` を配置する。
- C++ 実装を持つパッケージでは、実装ファイルを `src/` 配下に配置する。

## Conditional Items

- `setup.cfg` は `ament_python` パッケージで使用することを推奨する。
- `launch/` は launch ファイルを提供する場合に配置する。
- `params/` は ROS 2 パラメータ YAML を提供する場合に配置する。
- `msg/` は独自メッセージを定義する場合に配置する。
- `srv/` は独自サービスを定義する場合に配置する。
- `docs/` はパッケージ固有の設計書、検討記録、仕様変更メモを置く場合に配置する。
- `tests/` は pytest ベースのテストを置く場合に配置する。
- `tools/` は開発、検証、可視化、変換用の補助スクリプトを置く場合に配置する。
- `routes/` は経路データを置く場合に配置する。
- `maps/` は地図データを置く場合に配置する。
- `models/` は学習済みモデルや推論用モデルを置く場合に配置する。
- `rviz/` は RViz 設定を置く場合に配置する。
- `third_party/` は外部コードや外部ライブラリを同梱する場合に配置する。

## Protected and Generated Items

- `__pycache__/`, `.pytest_cache/`, `*.egg-info/` は生成物として扱い、追加・編集しない。
- `third_party/` 配下は原則として編集しない。変更が必要な場合は、影響範囲と理由を明確にする。
- `routes/`, `maps/`, `models/` はデータ領域として扱い、明示指示がある場合を除き編集しない。

## Directory Roles

- `package.xml`, `setup.py`, `setup.cfg`, `CMakeLists.txt` はパッケージ定義、依存関係、エントリポイント、ビルド設定を管理する。
- `resource/<package_name>` は `ament_python` のパッケージ登録用ファイルとして維持する。
- `launch/` には launch 単位の起動構成を置く。
- `params/` には ROS 2 パラメータ YAML を置く。
- Python パッケージディレクトリには、ノード、ROS 非依存コアロジック、内部モジュールを置く。
- `*_core.py` には、可能な限り ROS 非依存の処理、状態管理、計算ロジックを置く。
- `<node>_node.py` には、ROS 通信、パラメータ、QoS、ログ、タイマー、publisher/subscriber/service/action などを置く。
- `src/` には C++ の実装ファイルを置く。
- `msg/`, `srv/` には ROS interface 定義を置く。
- `docs/` には設計書、検討記録、仕様変更メモを置く。
- `tests/` には pytest 互換のテストを置く。
- `tools/` には開発・検証・可視化・変換用の補助スクリプトを置く。

## Naming Rules

- ノードファイル名は `<name>_node.py` を基本とする。
- コア処理ファイル名は機能名に対応した `*_core.py` を基本とする。
- launch ファイル名は `<name>.launch.py` とする。
- パラメータファイル名は内容を表す `<name>.yaml` とする。
- テストファイル名は `test_*.py` とする。
- ROS トピック名は lower_case とスラッシュ区切りを基本とする。
- 既存の公開 API 名や ROS interface 名は不用意に変更しない。
- 既存ファイルの命名規則がある場合は、既存規則との整合を優先する。

## Development Policy

- パッケージ構成を不用意に崩さない。
- コード変更と設計書・検討記録の整合を保つ。パッケージ詳細設計書のテンプレートは
  `design-doc-template` スキルを参照する。
- 依存関係を追加する場合は、`package.xml`, `setup.py`, `CMakeLists.txt` の必要箇所を確認する。
- pip install が必要な Python モジュールを追加する場合は、ワークスペース直下の `requirements.txt` にも記述する。
- ROS 非依存で表現できるロジックは、ノード本体ではなく `*_core.py` などの独立したモジュールに寄せる。
- ノード本体は ROS 入出力とライフサイクル管理を中心に薄く保つ。
- 新規パッケージを追加する場合は、上記の推奨構成に従う。
