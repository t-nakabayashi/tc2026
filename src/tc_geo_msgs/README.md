# tc_geo_msgs

## 概要

地理座標（WGS84 LLH）と ENU 投影条件を、走行系・表示系・記録系で共有するための
interface 定義パッケージである。ノードを持たない。

## 前提

ROS 2 Jazzy。ビルド依存は `std_msgs` と `rosidl_default_generators`。

## ビルド

ワークスペースルートで他パッケージと同時にビルドされる。

```bash
colcon build --symlink-install --packages-select tc_geo_msgs
```

## メッセージ一覧

| メッセージ | 用途 | 主な配信元 |
| --- | --- | --- |
| `GeoPoint` | WGS84 の緯度経度高度。単独では配信せず `GeoPose` へ埋め込む | — |
| `GeoPose` | 緯度経度と真北基準の方位。`GeoPoseWithQuality` と `Waypoint.geo_pose` へ埋め込む | — |
| `GeoPoseWithQuality` | 地理姿勢に測位品質と融合状態を添えたもの | `geo_pose_converter` |
| `MapProjection` | ENU と LLH を相互変換する投影条件。経路にも埋め込む | `geo_pose_converter` |
| `FusionState` | GNSS・LIO 融合の状態。表示と制御判断の双方に使う | `gnss_lio_fusion` |

各メッセージの配信様式、公称レート、QoS の例外は `.msg` の先頭コメントに宣言する。
規則は [トピック通信規約](../../docs/トピック通信規約.md) を正とする。

## 確認方法

```bash
ros2 interface list | grep tc_geo_msgs
ros2 interface show tc_geo_msgs/msg/GeoPoseWithQuality
```

## 関連文書

- [tc_geo_msgs 詳細設計書](docs/tc_geo_msgs_詳細設計書.md)：各 field の意味と単位、設計上の制約
- [LLH・ENUトピックとメッセージ仕様](../../docs/LLH・ENUトピックとメッセージ仕様.md)：座標の使い分けと配信責務
