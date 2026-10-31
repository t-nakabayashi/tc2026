# rtk_gps_um982_msgs

## 概要

Unicore UM982 受信機ドライバが配信する測位品質・補正状態のメッセージ定義パッケージである。
ノードを持たない。

## 前提

ROS 2 Jazzy。ビルド依存は `std_msgs` と `rosidl_default_generators`。

## ビルド

ワークスペースルートで他パッケージと同時にビルドされる。ドライバより先に構築する。

```bash
colcon build --symlink-install --packages-select rtk_gps_um982_msgs rtk_gps_um982
```

## メッセージ一覧

| メッセージ | 用途 | 配信元 |
| --- | --- | --- |
| `RtkStatus` | RTK 測位状態、衛星数、HDOP、二アンテナ方位、基線長、RTCM 補正の受信状況 | `rtk_gps_um982` |

`rtk_state` は数値定数（`STATE_UNKNOWN` から `STATE_RTK_FIX`）と、ライブラリが返す
生文字列 `rtk_state_raw` の両方を持つ。判定には数値を使い、生文字列はログと表示に使う。

方位は真北基準の時計回り [deg] である。ENU の yaw（東 0、反時計回り）とは基準が異なる。

配信様式、公称レート、QoS の例外は `.msg` の先頭コメントに宣言する。
規則は [トピック通信規約](../../docs/トピック通信規約.md) を正とする。

## 確認方法

```bash
ros2 interface show rtk_gps_um982_msgs/msg/RtkStatus
```

## 関連文書

- [rtk_gps_um982](../rtk_gps_um982/README.md)：ドライバの起動とパラメータ
- [rtk_gps_um982 詳細設計書](../rtk_gps_um982/docs/rtk_gps_um982_詳細設計書.md)：各 field の算出元と制約
