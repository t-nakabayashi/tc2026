# tc_perception_msgs パッケージ README

## 概要
`tc_perception_msgs` は、カメラ画像認識の結果とLiDAR水平スライスの観測結果を、表示側・
判断側へ渡すためのメッセージ型を定義するインタフェース専用パッケージです。
認識ノード (`traffic_signal_recognizer`・`road_blockage_detector`) は重畳画像を配信せず、
本パッケージの `PerceptionOverlay` だけを配信します。画像への重畳は `robot_console` の `ConsoleCore` が行い、Qt UI と HTML UI が
同一の描画結果を共有します。

この分担により、ネットワークを流れるカメラ画像は `/usb_cam/image_raw` の 1 本だけになり、
rosbag には生画像と構造化された認識結果が残るため、後から描画方法を変えて再現できます。

## 主な機能
- 認識結果の重畳表示用メッセージ (`PerceptionOverlay`) の定義。
- 重畳する検出 1 件分 (`OverlayDetection`) の定義。
- LiDAR水平スライス各層の観測結果 (`ObstacleLayers`・`ObstacleLayer`) の定義。

## 起動方法
本パッケージはノードを持たず、`colcon build` により他パッケージと同時にビルドされます。
インタフェースの確認には以下を利用してください。
```bash
ros2 interface show tc_perception_msgs/msg/PerceptionOverlay
ros2 interface show tc_perception_msgs/msg/OverlayDetection
ros2 interface show tc_perception_msgs/msg/ObstacleLayers
```

## 外部インタフェース
本パッケージはトピックやサービスサーバを提供しませんが、以下のメッセージ型を配布します。

| メッセージ | 用途 |
| --- | --- |
| `PerceptionOverlay` | 元画像フレームに紐づく認識結果一式（検出・判定・状態）。 |
| `OverlayDetection` | 画像へ重畳する検出 1 件（矩形・ラベル・スコア・採用可否）。 |
| `ObstacleLayers` | 水平スライス各層の点数・最近傍距離・左右の横移動余裕。 |
| `ObstacleLayer` | 1 層分の観測結果（`ObstacleLayers.layers` の配列要素）。 |

### フレームの紐付け
`PerceptionOverlay.header` には、判定根拠となった元画像フレームの `stamp` / `frame_id` を
そのまま引き継ぎます。`yolo_detector` が `Detection2DArray.header` へ元画像の header を
複製しているため、認識ノードはそれを転記するだけで紐付けが成立します。

認識処理は生画像より低いレートで動作し得るため、生画像の全フレームに対して
`PerceptionOverlay` が対応するとは限りません。表示側はこの前提で、直近の結果を保持しながら
描画する必要があります（保持時間・同期判定の扱いは `robot_console` 側の責務）。

## 配信トピック（利用側の既定値）

| トピック | 配信元 |
| --- | --- |
| `/perception/traffic_signal/overlay` | `traffic_signal_recognizer` |
| `/perception/road_blockage/overlay` | `road_blockage_detector` |
| `/obstacle/layers` | `lidar_layer_slicer` |

制御経路で使う `/sig_recog`・`/road_blocked` は従来どおり各認識ノードが配信します。
`PerceptionOverlay.decision` にも判定値を持たせているのは、表示上の検出枠と判定が
必ず同一フレームの評価結果であることを保証するためです。

### 水平スライスの位置付け
`ObstacleLayers` は観測値であり、停止判定には使いません。前方閉塞 (`front_blocked`) は
`obstacle_monitor` が topURG の `/scan` のみから判定します。スライスの用途は、AI 助言へ渡す
状況説明と、回避幅の上限を実測から与えることの 2 つです。

`point_count` が 0 の層は未観測であり、障害物が無いことを意味しません。MID-360 は仰角
-7°〜+52° を前下がり取付で使うため、観測できる高さ帯が距離に依存します。
`required_*_offset_m` が inf の層は、その層では通行可能な横移動量が見つからないことを表します。
