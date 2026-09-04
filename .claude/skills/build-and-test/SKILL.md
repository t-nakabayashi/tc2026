---
name: build-and-test
description: colcon によるワークスペース/パッケージのビルド手順と、pytest によるテスト実行・追加時の作法を定義する。colcon build を実行するとき、ビルド失敗を調査するとき、build/install/log をクリーンするか判断するとき、tests/test_*.py を追加・修正するとき、pytest を実行して結果を報告するときは必ずこのスキルを参照する。
---

# ビルド

- このワークスペースは ROS 2 Jazzy を前提とする。
- ビルド前に、各自の開発環境に合わせて ROS 2 環境を有効化する。
- 開発時の基本ビルドは以下を使う。

```bash
colcon build --symlink-install
```

- 変更対象が明確な場合は、まず対象パッケージだけをビルドする。

```bash
colcon build --symlink-install --packages-select <package_name>
```

- ビルド確認では、`--symlink-install` 付きと無しの両方を確認する。

```bash
colcon build --symlink-install --packages-select <package_name>
colcon build --packages-select <package_name>
```

- `--symlink-install` 付きと無しを切り替えて再ビルドする場合、既存生成物により
  symlink 作成や通常 install が競合することがある。その場合は、確認対象パッケージに
  対応する `build/<package_name>/`, `install/<package_name>/`, `log/` 配下の生成物を
  クリーンしてから再実行してよい。
- 複数パッケージやワークスペース全体の確認でパッケージ単位のクリーンでは解消できない場合は、
  `build/`, `install/`, `log/` を colcon 生成物として削除してから再実行してよい。
- クリーンを行った場合は、削除対象と理由を作業報告に明記する。
- ワークスペース全体の確認が必要な場合も、`--symlink-install` 付きと無しの両方を確認する。

```bash
colcon build --symlink-install
colcon build
```

- ビルド失敗時は、最初の本質的なエラーを特定してから修正する。
- エラー出力を要約だけで判断せず、該当ファイル、行番号、依存関係、entry point、import error を確認する。
  `colcon build` が成功していても `install/` への反映が古いままだと `ros2 launch` 時に
  `executable not found` のような形で顕在化することがあるため、ビルド成功メッセージと
  `install/<package_name>/` 配下の対象ファイルの存在を合わせて確認する。
- 確認結果を報告する場合は、実行したコマンド、成功/失敗、失敗時の原因、未確認事項を明記する。

# テスト

このスキルが扱うのは pytest で書く自動テストに限る。対象は `*_core.py` など ROS 非依存
ロジックの単体テストを中心とするが、疑似 publisher/subscriber でノードの入出力を検証する
自動テストを書く場合も、この節の作法に従う。

ノードを実際に起動してインタラクティブに動作確認する（自動テスト化しない）場合、複数
ノードを組み合わせて確認する場合、Gazebo シミュレーションで確認する場合は自動テストの
範囲外であり、`ros2-local-run` スキルを参照する。

- テストは `tests/test_*.py`（パッケージによっては `tools/tests/test_*.py`）に配置する。
- `pytest` 互換の構文を使う。
- 既存テストの意図を尊重する。
- 失敗テストを単に削除・緩和しない。
- まず `*_core.py` など ROS 非依存ロジックの単体テストを優先する。
- テストでは、境界値、異常系、状態遷移、パラメータ差分を確認する。
- 外部環境に依存するテストは、実行できない環境で失敗し続けないよう skip 条件を検討する。
- テスト追加後は、可能な範囲で以下を実行する。

```bash
pytest
```

- 対象パッケージが明確な場合は、以下を使う。

```bash
pytest src/<package_name>/tests
```

- ワークスペース直下の `pytest.ini` が `testpaths` を管理しているため、新規パッケージに
  テストを追加した場合は `testpaths` への追記が必要かどうかを確認する。
- テスト結果を報告する場合は、実行コマンド、成功/失敗、失敗時の原因、未確認範囲を明記する。
