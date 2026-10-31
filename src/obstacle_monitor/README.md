# obstacle_monitor パッケージ README

## 概要
`obstacle_monitor` は `/scan`（LiDAR）から前方障害物を解析し、
`tc_route_msgs/ObstacleAvoidanceHint` を `/obstacle_avoidance_hint` として配信するノードです。
`/sensor_viewer`（`sensor_msgs/Image`）で LaserScan と判定結果の可視化も提供します。動作確認用に
`laser_scan_simulator` ノードも同梱しています。

## 主な機能
- `/scan` の ±90° 以内の点群からロボット幅帯を抽出し、左右の回避オフセットを算出。
- ロボット幅帯内の距離分位点と `stop_dist_m` の比較で閉塞状況を検知（角度くさびではない）。
- `/obstacle_avoidance_hint` に front_blocked・front_clearance_m・左右オフセットを配信。
- `/sensor_viewer` に LaserScanViewer 互換の bgr8 画像を配信し、
  `/localization/pose_enu`・`/active_target` を重畳して目標方向を可視化。
- `hint_range_m` が `max_obstacle_distance_m` 未満の場合は警告しつつ自動で補正。

## 起動方法
### launch を用いた起動
```bash
ros2 launch obstacle_monitor obstacle_monitor.launch.py \
  scan_topic:=/scan hint_topic:=/obstacle_avoidance_hint
```
- launch 引数で購読・配信トピック名を remap できます。
- `params/default.yaml` が既定パラメータを提供します。

### 実行ファイルを直接起動
```bash
ros2 run obstacle_monitor obstacle_monitor
```
- `laser_scan_simulator` を併用する場合は別ターミナルで
  `ros2 run obstacle_monitor laser_scan_simulator` を起動し、`/localization/pose_enu` を供給してください。

## 外部インタフェース
### Subscriber
| 名称 | 型 | 説明 | QoS |
|------|----|------|-----|
| `/scan` | `sensor_msgs/LaserScan` | LiDAR入力。 | BEST_EFFORT / VOLATILE / depth=1 |
| `/localization/pose_enu` | `geometry_msgs/PoseWithCovarianceStamped` | 現在姿勢。viewer で目標線を描画するために利用。 | RELIABLE / VOLATILE / depth=10 |
| `/active_target` | `geometry_msgs/PoseStamped` | 現在の目標位置。viewer の矢印描画に利用。 | RELIABLE / VOLATILE / depth=10 |

### Publisher
| 名称 | 型 | 説明 | QoS |
|------|----|------|-----|
| `/obstacle_avoidance_hint` | `tc_route_msgs/ObstacleAvoidanceHint` | front_blocked・front_clearance_m・左右オフセット[m] を配信。 | BEST_EFFORT / VOLATILE / depth=1 |
| `/sensor_viewer` | `sensor_msgs/Image` | LaserScanViewer 相当のデバッグ画像（bgr8）。 | RELIABLE / VOLATILE / depth=1 |

> サービス・アクションは提供しません。

## パラメータ
既定値は、ノードの宣言と `params/default.yaml` で 3 件が異なる。launch は既定で YAML を
読み込むため、`ros2 launch` では YAML 側が有効になる。

| 名称 | 型 | 宣言 | YAML | 概要 |
|------|----|------|------|------|
| `front_cone_half_deg` | double | `10.0` | `10.0` | **読み込まれていない。** 判定は車幅帯で行うため、変えても効かない。
| `stop_dist_m` | double | `1.0` | `0.75` | 車幅帯内の距離分位点がこの値以下で閉塞と判定する [m]。
| `front_clearance_percentile` | double | `5.0` | `5.0` | 前方閉塞判定で使用する距離分位点[%]。
| `front_clearance_min_points` | int | `5` | `5` | 前方閉塞判定を行うために必要な最小点数。
| `max_obstacle_distance_m` | double | `1.5` | `1.5` | 点群の抽出上限 [m]。検知範囲はこの値で決まる。
| `front_block_ignore_dist_m` | double | `0.15` | `0.15` | 点群の抽出下限（前方距離）[m]。
| `hint_range_m` | double | `max_obstacle_distance_m` | `5.0` | front_clearance を通知する最大距離 [m]。抽出上限より小さいと警告して補正する。
| `robot_width_m` | double | `0.8` | `0.6` | ロボットの全幅 [m]。回避オフセットの閾値と閉塞判定の帯幅に使う。
| `avoid_offset_min_m` | double | `0.75` | `0.75` | 左右オフセットの下限値 [m]。
| `viewer_map_range_m` | double | `8.0` | `8.0` | viewer の描画範囲（x:0..範囲, y:±範囲/2）。
| `viewer_pixel_pitch` | int | `100` | `100` | viewer のピクセル密度 [pix/m]。
| `viewer_window` | string | `"laserscan"` | `"laserscan"` | ウィンドウ名として読み込むが、OpenCV の表示は無効化してある。画像は topic のみ。
| `viewer_resize_px` | int | `500` | `500` | 配信画像のリサイズサイズ [px]。

## 動作確認手順
1. `robot_simulator` などから `/localization/pose_enu`・`/active_target` を配信する。
2. `laser_scan_simulator` を起動し、LiDAR の疑似データを `/scan` に配信する。
3. `obstacle_monitor` を launch し、`/obstacle_avoidance_hint` の `front_blocked` や
   `left_offset_m` が障害物に応じて変化することを確認する。
4. `rqt_image_view` 等で `/sensor_viewer` を参照し、点群・目標矢印が描画されていることを確認する。

## デバッグのヒント
- `hint_range_m` < `max_obstacle_distance_m` の場合は WARN が出力され、内部で自動補正されます。
- viewer 画像が真っ白な場合は `/scan` の距離が範囲外になっていないか確認してください。
- `front_blocked` が常に `False` の場合は `robot_width_m`（判定に使う帯幅）と `stop_dist_m`、
  `front_clearance_min_points` の組み合わせを見直します。`front_cone_half_deg` は効きません。

## 制約

点数不足では非閉塞・距離infを返すため、欠測と空き空間を区別できない。
前方点の抽出上限は `max_obstacle_distance_m` であり、`hint_range_m` だけを増やしても
遠方の検知範囲は広がらない。
2D スキャンの前方±90°のみを使い、TF 変換を行わない。低い段差、旋回中の車体外周、
後方の障害物は検知しない。

## 関連文書

判定ロジックと制約の詳細は
[obstacle_monitor 詳細設計書](docs/obstacle_monitor_詳細設計書.md) を参照する。
