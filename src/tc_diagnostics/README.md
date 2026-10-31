# tc_diagnostics

## 概要

各ノードが `/diagnostics` へ自己申告診断を配信するための共通ヘルパである。
ノードは持たず、他パッケージから import して使う。

`DiagnosticStatus.name` の命名規約と配信周期・QoS を本パッケージが一手に引き受け、
呼び出し側は観点ごとに判定結果を渡すだけでよい。判定ロジックは各ノードの責務であり、
本パッケージは持たない。

## 前提

ROS 2 Jazzy。依存は `rclpy` と `diagnostic_msgs`。

`diagnostic_updater` は使わない。依存とノードごとの定型コードを増やさずに name 規約を
1 か所で強制するためである。

## ビルド

ワークスペースルートで他パッケージと同時にビルドされる。

```bash
colcon build --symlink-install --packages-select tc_diagnostics
```

## 使い方

ノードの初期化時に `DiagnosticReporter` を生成し、観点ごとに `report()` を呼ぶ。
publisher とタイマーは生成時に用意され、既定 1 Hz で最新の申告値をまとめて配信する。

```python
from tc_diagnostics import ASPECT_DEVICE, ERROR, OK, DiagnosticReporter

self._diagnostics = DiagnosticReporter(self)
self._diagnostics.report(ASPECT_DEVICE, OK, 'ポートを開いた', {'port': port})
self._diagnostics.report(ASPECT_DEVICE, ERROR, '受信が途絶した')
```

主処理に固有の判定を持たないノードは `report_alive(reporter)` で `liveness` だけを申告する。

## 公開 API

| 名前 | 用途 |
| --- | --- |
| `DiagnosticReporter` | 観点ごとの申告値を保持し、周期配信する |
| `report_alive` | `liveness` の OK だけを申告する最小の呼び出し |
| `compose_name` | `<ノード名>/<観点>` 形式の name を組み立てる |
| `build_status` / `build_array` | `DiagnosticStatus` / `DiagnosticArray` を規約どおりに構成する |
| `worst_level` | 複数の level から最も重いものを選ぶ |
| `ASPECT_LIVENESS` / `ASPECT_DEVICE` / `ASPECT_QUALITY` | 観点の定数 |
| `OK` / `WARN` / `ERROR` / `STALE` | level の定数 |

観点は定数を使う。任意の文字列も渡せるが、空文字と `/` を含む値は `DiagnosticNameError` に
なる。観点を増やす場合は `diagnostic_core.py` へ定数として定義し、表記の揺れを防ぐ。

## パラメータ

ROS パラメータは持たない。配信周期は `DiagnosticReporter(node, period_s=...)` で指定する。
既定は 1.0 秒であり、公称レート 1 Hz に対応する。

## 確認方法

```bash
ros2 topic echo /diagnostics --once
```

`name` が `<ノード名>/<観点>` の形式であること、判定に使うノードから届いていることを確認する。

単体の検査は `tests/test_diagnostic_core.py` で行う。

## 関連文書

- [ノード健全性監視設計](../../docs/ノード健全性監視設計.md)：二層構成の理由、name 規約、健全性の合成規則
- [トピック通信規約](../../docs/トピック通信規約.md)：配信様式と QoS の導出規則
