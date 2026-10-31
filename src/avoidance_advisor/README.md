# avoidance_advisor

## 1. 概要
停止中の回避について、AI エージェントへ方向とオフセットの助言を求める action server。
推論は機上 PC で `claude -p` 相当の CLI を起動して行う。
助言は提案値であり、検証とクランプを通ったものだけを `advice_valid=true` で返す。

## 2. 前提
- `tc_route_msgs`、`tc_perception_msgs`、`tc_diagnostics`、`cv_bridge` を使う。
- 入力は `lidar_layer_slicer` の層画像・層観測値と、カメラ画像である。
  どれも受信していなくても要求は処理でき、その場合は観測値なしで判断した助言になる。
- CLI は機上 PC に導入済みであること。導入していない環境では、要求は失敗として返る
  （ノードは落ちない）。`enabled:=false` で起動すれば要求自体を拒否する。
- CLI の実行にはネットワークが要る。圏外・回線不良は失敗として扱う。
- 課金と速度制限を伴う。呼び出しの間隔と回数の制御は要求側（route_follower）が持つ。
  本ノードは同時実行を 1 件に制限するだけである。

## 3. ビルド
```bash
colcon build --packages-select tc_route_msgs tc_perception_msgs avoidance_advisor
source install/setup.bash
```

## 4. 起動
```bash
ros2 launch avoidance_advisor avoidance_advisor.launch.py

# CLI を使えない環境で、要求を必ず拒否させる
ros2 launch avoidance_advisor avoidance_advisor.launch.py enabled:=false
```

## 5. パラメータ
利用者が変更しうるものを示す。全パラメータは
[詳細設計書](docs/avoidance_advisor_詳細設計書.md) 4 章を参照する。

| 名前 | 型 | 既定値 | 単位 | 変更すると何が変わるか |
| --- | --- | --- | --- | --- |
| `enabled` | bool | `true` | - | false で要求を拒否する |
| `cli_command` | string[] | `["claude", "-p", ...]` | - | 起動するコマンド。`{prompt}` が要求文へ置換される |
| `cli_timeout_sec` | double | `20.0` | s | 応答を待つ上限。超えるとプロセスを終了する |
| `workspace_root` | string | `""` | - | 作業ディレクトリの親。空なら一時ディレクトリ配下 |
| `min_confidence` | double | `0.5` | - | これ未満の確信度の助言を採用しない |
| `default_wait_sec` | double | `10.0` | s | 待機時間が得られない場合に使う値 |
| `max_wait_sec` | double | `30.0` | s | 待機時間の上限 |
| `image_max_age_sec` | double | `5.0` | s | これより古い画像は要求文へ含めない |

## 6. 確認方法
```bash
# 助言を 1 件要求する
ros2 action send_goal /request_avoidance_advice \
  tc_route_msgs/action/RequestAvoidanceAdvice \
  '{waypoint_label: wp1, waypoint_left_open_m: 0.8, waypoint_right_open_m: 0.5,
    front_clearance_m: 0.7, min_offset_m: 0.35, max_offset_m: 3.0}'

# 配信された助言
ros2 topic echo /avoidance_advice
```
CLI を呼ばずに動作を確かめる場合は、JSON を返すスクリプトを `cli_command` に指定する。

```bash
ros2 run avoidance_advisor avoidance_advisor \
  --ros-args -p 'cli_command:=["/path/to/fake_cli.sh","{prompt}"]'
```

各要求の要求文、入力画像、CLI の出力は作業ディレクトリ（既定は一時ディレクトリ配下の
`avoidance_advisor/<request_id>/`）へ残る。`request_id` は `/avoidance_advice` と
action の result にも入るため、後から対応付けられる。

単体確認は `python3 -m pytest src/avoidance_advisor/tests` で行う。
助言の検証とクランプ、CLI のタイムアウト・取り消し・子プロセスの終了を固定している。
実際の AI エージェントを使った判断の妥当性は確認していない。

## 7. 関連文書
- [詳細設計書](docs/avoidance_advisor_詳細設計書.md)
- [tc_route_msgs](../tc_route_msgs/README.md)
- [lidar_layer_slicer](../lidar_layer_slicer/README.md)
