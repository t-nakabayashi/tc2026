# lidar_layer_slicer

## 1. 概要
MID-360 の点群と topURG の `/scan` から、床面基準の水平スライス層を作る。
層ごとの点数・最近傍距離・左右の横移動余裕を `ObstacleLayers` として配信し、
同じ層を並べた AI 入力用の可視化画像を配信する。
停止判定は行わない。前方閉塞は `obstacle_monitor` が `/scan` のみから判定する。

## 2. 前提
- `tc_perception_msgs`、`tc_diagnostics`、`cv_bridge`、`tf2_ros` を使う。
- `base_link` から LiDAR フレームへの TF が必要である。実機は `icart_bringup` の
  `hardware.launch.py` が `hardware.yaml` の取付値で静的 TF を配信する。
  TF が取得できない間、点群層は作られない（`point_count` は 0 になる）。
- 実機は livox driver の `CustomMsg`、模擬構成は Gazebo bridge の `PointCloud2` を入力にする。
- 実機センサを起動する手順は含まない。本パッケージ単体の確認は bag 再生または
  模擬構成で行う。

## 3. ビルド
```bash
colcon build --packages-select tc_perception_msgs lidar_layer_slicer
source install/setup.bash
```

## 4. 起動
```bash
# 実機構成（livox CustomMsg を入力にする既定）
ros2 launch lidar_layer_slicer lidar_layer_slicer.launch.py

# 模擬構成（PointCloud2 を入力にする）
ros2 launch lidar_layer_slicer lidar_layer_slicer.launch.py \
  points_topic:=/mid360/livox/lidar/points
```
`cloud_message_type` を `pointcloud2` にすると `points` を購読する。既定は `custom` で、
`livox_ros_driver2` のメッセージを取り込めない環境では自動的に `points` へ切り替える。

## 5. パラメータ
利用者が変更しうるものを示す。全パラメータは
[詳細設計書](docs/lidar_layer_slicer_詳細設計書.md) 4 章を参照する。

| 名前 | 型 | 既定値 | 単位 | 変更すると何が変わるか |
| --- | --- | --- | --- | --- |
| `cloud_layer_heights_m` | double[] | `[0.35, 0.50]` | m | 点群から切り出す層の床上高さ |
| `cloud_layer_thickness_m` | double[] | `[0.10, 0.10]` | m | 層の厚さ。厚くすると点は増え高さ分解能は落ちる |
| `scan_layer_height_m` | double | `0.25` | m | scan 層として記録する床上高さ（切り出しには使わない） |
| `forward_range_m` | double | `2.0` | m | 横移動余裕を評価する前方距離 |
| `robot_width_m` | double | `0.6` | m | 車体全幅。左右余裕と合わせて占有幅を決める |
| `side_margin_m` | double | `0.1` | m | 車体幅へ加える左右の余裕 |
| `max_offset_m` | double | `3.0` | m | 探索する横移動量の上限 |
| `accumulate_frames` | int | `3` | 枚 | 点群層に使う累積フレーム数 |
| `view_pixel_pitch` | int | `120` | pix/m | 可視化画像の解像度 |

## 6. 確認方法
```bash
# 層の観測値
ros2 topic echo /obstacle/layers --once

# 可視化画像
ros2 run rqt_image_view rqt_image_view /obstacle/layer_view
```
`point_count` が 0 の層は未観測であり、障害物が無いことを意味しない。
TF が無い、入力が届いていない、その高さが観測範囲外のいずれでも 0 になる。

単体確認は `python3 -m pytest src/lidar_layer_slicer/tests` で行う。
層の切り出し、横移動余裕、画像の座標写像を固定している。
実機センサ、Gazebo、実際の取付での妥当性は確認していない。

## 7. 記録済み bag からの再現

手動走行で記録した bag から、前方へ障害物が近づいた場面の 3 層画像・カメラ画像・
構造化テキストを書き出す。層の高さと厚さの確認と、AI へ渡す入力の事前評価に使う。

```bash
python3 src/lidar_layer_slicer/tools/replay_bag_layers.py <bag のディレクトリ> \
  --output <出力先> --limit 20
```
bag の配置はリポジトリ外であり、引数で与える。既定値は持たない。
取付姿勢は bag の `/tf_static` を使うため、記録時の取付で再現される。

場面は `/scan` から計算した直進帯の最近傍距離が `--near-threshold`（既定 2.0 m）以下に
なった時刻で選ぶ。記録時の obstacle_monitor の設定に依存させないためである。
`--require-stop` を付けると停止中の場面だけに絞るが、手動走行では運転者が障害物へ
近づく前に避けるため、場面が得られないことがある。

多数の場面をまとめて数える場合は `analyze_bag_layers.py` を使う。層ごとに点が得られた
割合、層から求めた横移動余裕、記録時の `obstacle_avoidance_hint` との差を集計し、
場面ごとの値を CSV へ書き出す。

```bash
python3 src/lidar_layer_slicer/tools/analyze_bag_layers.py <bag のディレクトリ> \
  --near-threshold 3.0 --min-interval-sec 1.0 --csv <出力.csv>
```
点群は場面の直前 `--accumulate-sec` 分だけを復号するため、長い bag でも数十秒で終わる。

## 8. 関連文書
- [詳細設計書](docs/lidar_layer_slicer_詳細設計書.md)
- [トピック通信規約](../../docs/トピック通信規約.md)
- [tc_perception_msgs](../tc_perception_msgs/README.md)
- [obstacle_monitor](../obstacle_monitor/README.md)
