# rtk_gps_um982 設計書

## 1. 役割と構成

UM982のシリアル入力から位置・デュアルアンテナ方位・RTK状態をROS 2 Jazzyへ配信する。
`third_party/UM982-RTK-GPS-Library` をgit submoduleとして同梱し、
`ntrip_client.py` の `CorrectedUM982Client` がNTRIP受信処理を補う。
時刻配信有効時は `ClockRelayClient` が単一のシリアル読み取りからRMCを分配する。
車体座標への補正とGNSS/LIO融合は `gnss_lio_fusion` が担当する。
本ノードはTFや融合位置を配信しない。

## 2. 公開インタフェース

launch既定のノードは `/rtk_gps/rtk_gps_um982_node`。
トピックはノード名を含まず、実機・模擬とも `/rtk_gps/` 配下に公開する。
パブリッシャのQoSはRELIABLE・VOLATILE・depth 10。

| トピック | 型 | 配信条件 |
|---|---|---|
| `/rtk_gps/fix` | `sensor_msgs/NavSatFix` | 有効位置かつ最低品質以上の位置コールバック |
| `/rtk_gps/heading` | `sensor_msgs/Imu` | 同上。方位欠測は共分散で未提供と表現 |
| `/rtk_gps/rtk_status` | `rtk_gps_um982_msgs/RtkStatus` | 同上 |
| `/rtk_gps/ntrip_status` | `std_msgs/String` | 1 HzのJSON。位置欠測と独立 |
| `/rtk_gps/time_sync` | `std_msgs/String` | 時刻配信有効時、1 HzのJSON |

位置・方位・RTK状態には同じstampと `frame_id` を付ける。
`min_fix_for_publish` 未満や無効位置ではこの3種を配信しない。
そのため `rtk_status` の無更新を品質良好と解釈してはならない。

## 3. メッセージ変換

`converters.py` が以下を行う。

- NavSatFixの緯度・経度・高度は受信機の値を使う。
  水平分散は `(HDOP × hdop_sigma)²`、垂直分散はその4倍で、VDOPは使わない。
  RTK FLOAT/FIXはいずれもGBAS_FIX、DGPSはSBAS_FIX、単独測位はFIX、未知はNO_FIXとなる。
- headingは真北から時計回りの角度で、ENU yawは `π/2 − heading_rad`。
  pitchを使用し、rollは0と仮定する。yaw/pitch分散は各標準偏差をradへ変換した二乗、
  値がない場合は0.01。rollの分散は1e6とする。
- 方位がない場合は `orientation_covariance[0]=-1`。
  角速度・加速度も未計測で、それぞれの共分散先頭を-1とする。
- RtkStatusの状態はUNKNOWN=0、STANDALONE=1、DGPS=2、RTK_FLOAT=3、RTK_FIX=4。
  生の状態文字列、衛星数、HDOP、方位・pitchと各標準偏差、基線長、補正経過時間、
  RTCM累積バイト、位置を格納する。フィールド定義は
  [RtkStatus.msg](../../rtk_gps_um982_msgs/msg/RtkStatus.msg)を正とする。

## 4. 設定と起動

設定の正本は [config/default.yaml](../config/default.yaml)。
launchでは `config` でYAMLを指定する。`serial_port` や `ntrip_host` というlaunch引数はない。
YAMLのノードキーは `/rtk_gps/rtk_gps_um982_node` とする。

```bash
ros2 launch rtk_gps_um982 rtk_gps_um982.launch.py config:=<設定YAMLの絶対パス>
```

主なパラメータは [README](../README.md)に示す。
YAMLのシリアル既定はUSB固定名、ノード単独の既定は `/dev/ttyUSB0`。
launchの `time_sync` と `chrony_socket` 引数はYAML内の該当設定より優先する。
パラメータは起動時に読み込む。ランタイムの `ros2 param set` で
接続先や品質フィルタの実処理が切り替わる仕組みはない。

実機一式は [共通起動](../../icart_bringup/README.md)から使用する。
同じシリアルポートを複数ノードや端末プログラムで同時に開かない。

## 5. NTRIPと診断

NTRIP有効時はhostとmountpointが必須。欠ける場合はエラーログを出して接続しない。
クライアントはHTTP応答とRTCM3のフレーミング・CRCを検査し、有効な補正だけをシリアルへ渡す。
有効RTCMが15秒途絶した接続を切り、失敗時は1秒から最大30秒へ待機を延ばして再接続する。
TLSとHTTP chunked転送は非対応。

`/rtk_gps/ntrip_status` は接続状態、CRC確認済みRTCM量・受信速度・経過時間、
再接続回数・エラー種別、起動時の基地局表示情報を返す。認証情報は含めない。
位置トピックとこの診断を合わせて、測位欠測と補正通信の欠測を確認する。
`/rtk_gps/time_sync` はRMC鮮度・SOCK送信数・拒否数・エラーを返す。
`/diagnostics` の配信は実装されていない。

## 6. 確認方法

`test/` で変換・品質フィルタ、分割したNTRIP入力、無効応答、診断値、時刻処理を確認する。
実機では完全名で位置・方位・RTK状態・NTRIP診断を照合し、アンテナ配置と方位の符号を確認する。
FIX到達や方位の精度は通信の成功だけでは保証されない。

```bash
colcon test --packages-select rtk_gps_um982
colcon test-result --verbose
```

## 7. 時刻同期

PCはchronyで外部NTPに同期し、icart-clock.serviceがsoftware PTPでMID360へPC時刻を配信する。
GNSSドライバは受信機の測定UTCをROSメッセージへ保持する。RMC SOCKはnoselectでPC同期元から除外する。
共通実機起動はtime_sync.enabled=true、stamp_source=gnss_utc、transport_delay_ms=0を強制する。

### GNSS時刻

有効RMCから日付を確定し、GGAの測定時分秒と結合する。アンカーが2秒より古い場合は位置配信を止める。
time_sync_coreはチェックサム・状態・暦日を検査する。単一のシリアル読み取りからRMCを分配し、
chrony SOCK送信はnonblockingで再試行する。/rtk_gps/time_syncに鮮度・送信数・拒否数・SOCKエラーを出力する。

| stamp_source | 動作 |
|---|---|
| gnss_utc | PositionData.timestampにtransport_delay_msを加算する。共通起動は加算0 |
| ros_time | clock.now()で受信処理時刻を付ける。共通起動では使わない |
| pps_edge | PPS処理は未実装でclock.now()へフォールバック。共通起動では使わない |

方位と位置の計測epochは厳密には結合されていない。独立基準での絶対時刻精度は未検証。

### PTPサービスと起動監視

NTP選択・応答鮮度・残補正・推定誤差を確認してPTPを配信する。
AF_PACKETで点群・IMUを受動監視し、PTP種別・受信継続・受信時刻差・逆行を判定する。
0.1 ms以下の微小逆行は記録のみ。原時刻は書き換えない。
共通bringupは同期成立後に車輪・FAST-LIO・融合・走行制御を起動し、運用中の条件喪失でshutdownする。
PyQt5 GUIは未成立・喪失・監視更新停止を全タブ上部に警告する。
設定・自動処理・起動操作・復旧・確認範囲は[時刻同期と確認手順](PTP試験手順.md)を参照する。
