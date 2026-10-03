# rtk_gps_um982

Unicore UM982 デュアルアンテナ RTK GNSS 受信機を ROS 2 (Jazzy) で扱うドライバパッケージ。
[UM982-RTK-GPS-Library](https://github.com/t-nakabayashi/UM982-RTK-GPS-Library) を内部利用し、
シリアル接続 + NTRIP 受信を 1 ノードで完結させる。

## パブリッシュするトピック

| Topic                       | Type                                | 説明                              |
| --------------------------- | ----------------------------------- | --------------------------------- |
| `/rtk_gps/fix`                     | `sensor_msgs/NavSatFix`             | 緯度経度高度、HDOP 由来の共分散   |
| `/rtk_gps/heading`                 | `sensor_msgs/Imu`                   | デュアルアンテナの orientation (REP-103 ENU) |
| `/rtk_gps/rtk_status`              | `rtk_gps_um982_msgs/RtkStatus`      | RTK 種別・衛星数・baseline・RTCM 累計バイト等 |

launchの既定namespaceは `rtk_gps`、ノード名は `rtk_gps_um982_node`。
トピックはノード名を含まず、実機・模擬とも `/rtk_gps/` 配下に公開する。
`/rtk_gps/ntrip_status` は常時、`/rtk_gps/time_sync` は時刻配信有効時に、診断JSONを `std_msgs/String` で配信する。

## 必要環境

- Ubuntu 24.04
- ROS 2 Jazzy (`/opt/ros/jazzy` インストール済)
- Python 3.12
- UM982 受信機 (USB-Serial or 直結 UART)

## ビルド手順

```bash
cd ~/colcon_ws
# 初回 / 更新時に submodule を取得
git submodule update --init --recursive

# 必要なら apt で依存をインストール
sudo apt install python3-serial

colcon build --packages-select rtk_gps_um982_msgs rtk_gps_um982
source install/setup.bash
```

## 起動

### シンプル起動 (デフォルト)

```bash
ros2 launch rtk_gps_um982 rtk_gps_um982.launch.py
```

### 自前 YAML を渡す

```bash
ros2 launch rtk_gps_um982 rtk_gps_um982.launch.py \
    config:=/path/to/my_params.yaml
```

### ノード直接起動

```bash
ros2 run rtk_gps_um982 rtk_gps_um982_node \
    --ros-args -p serial.port:=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 -p output_rate_hz:=20
```

## パラメータ

`config/default.yaml` がlaunchで使う設定。ノードをYAMLなしで直接起動した場合の
`serial.port` は `/dev/ttyUSB0` なので、直接起動ではデバイスを明示する。
パラメータは起動時に読み取り、接続先などの変更にはノードの再起動が必要。主なもの:

| パラメータ            | 型     | 既定値          | 説明                                                  |
| --------------------- | ------ | --------------- | ----------------------------------------------------- |
| `serial.port`         | string | `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0` | YAMLのシリアル設定                              |
| `serial.baud`         | int    | `115200`        | ボーレート                                            |
| `output_rate_hz`      | int    | `10`            | UM982 の出力レート                                    |
| `frame_id`            | string | `gps_link`      | 全 publish の `header.frame_id`                       |
| `stamp_source`        | string | `gnss_utc`      | `gnss_utc` / `ros_time` / `pps_edge`                  |
| `transport_delay_ms`  | int    | `0`             | `gnss_utc` 用の受信遅延補正 (負値で前倒し)            |
| `ntrip.enabled`       | bool   | `false`         | NTRIP クライアント有効化                              |
| `ntrip.host`          | string | `""`            | caster ホスト                                         |
| `ntrip.port`          | int    | `2101`          | caster ポート                                         |
| `ntrip.mountpoint`    | string | `""`            | mountpoint                                            |
| `ntrip.user`          | string | `""`            | ユーザ名                                              |
| `ntrip.password`      | string | `""`            | パスワード。共有・Git管理対象外の設定で管理する        |
| `publish.navsatfix`   | bool   | `true`          | `/rtk_gps/fix` を publish するか                             |
| `publish.imu_heading` | bool   | `true`          | `/rtk_gps/heading` を publish するか                         |
| `publish.rtk_status`  | bool   | `true`          | `/rtk_gps/rtk_status` を publish するか                      |
| `min_fix_for_publish` | string | `standalone`    | `none`/`standalone`/`dgps`/`float`/`fix`              |
| `hdop_sigma`          | float  | `1.0`           | NavSatFix の position_covariance スケール             |

## 時刻同期

共通実機起動は外部NTP→PC chrony→software PTP→MID360を使用する。
GNSSメッセージには受信機の測定UTCを付ける。PCのUSB受信時刻への置き換えは行わない。

| パラメータ | ノード単独の既定 | 共通実機起動 |
|---|---|---|
| time_sync.enabled | false | true：RMC日付をGGAへ適用し、SOCKへ診断入力 |
| time_sync.chrony_socket | /run/chrony/um982.sock | 同じ。chrony側はnoselect |
| stamp_source | gnss_utc | gnss_utcを強制 |
| transport_delay_ms | 0 | 0を強制 |

NTP同期とPTP配信はOSサービス、起動待機と同期喪失時の終了は共通bringupが担当する。
GUIは実機モードで時刻同期警告を表示する。
[構成・導入・確認手順](docs/PTP試験手順.md)を参照する。

## トラブルシューティング

### `Permission denied: '/dev/ttyUSB0'`

dialout グループに入っていない。

```bash
sudo usermod -aG dialout $USER
# ログアウト/ログインし直す
```

### `Failed to start UM982Client` (ノードが即落ち)

- ケーブル / 給電を確認
- `dmesg | tail` で `ttyUSBx` が認識されているか確認
- `screen /dev/ttyUSB0 115200` 等で生 NMEA が流れているか確認

### NTRIP に繋がらない

- `/rtk_gps/rtk_status` の `rtcm_bytes_received` が増えていなければ caster 到達不可
- ホスト/ポート/mountpoint/credentials を確認
- ファイアウォール (caster は TCP 2101 が多い)

### RTK Fix にならない

- `/rtk_gps/rtk_status.correction_age_s` を見て補正が新しい (~1-3s) か確認
- アンテナの空が見えているか、マルチパス源 (建物近接) がないか
- 基準局までの距離 (10km 程度まで RTK Fix 期待、それ以上は厳しい)

## テスト

```bash
colcon test --packages-select rtk_gps_um982
colcon test-result --verbose
```

`test/` でメッセージ変換、NTRIPの受信処理・診断、時刻処理を確認する。
受信機・基地局・実アンテナを含む動作確認は別途行う。

## 設計

詳細は [`docs/design.md`](docs/design.md) を参照。

## ライセンス

MIT

## 公開RTK局とNTRIP受信

icart_bringupの地域別設定で稲城・つくばの公開局、補正なし、独自局を選択できる。
NTRIP v1受信はワークスペース側アダプターでTCP分割と先頭データ保持を処理し、
CRC一致のRTCM3のみシリアルへ渡す。無効なHTTP応答やSOURCETABLEを成功と扱わない。
15秒有効RTCMが来ない接続は再接続する。ライブラリsubmoduleにはワークスペース側の変更を加えず、アダプターを使用する。
TLS・HTTP chunked必須のサービスは非対応。実機FIXは別途確認する。

### 基地局診断

`/rtk_gps/ntrip_status` (`std_msgs/String` JSON) を1 Hzで配信する。
NTRIP接続状態、CRC確認済みRTCM量・速度・最終受信経過、再接続回数、通信エラー種別を
位置コールバックとは独立して送る。認証情報は含めない。
`ntrip.station_id`, `ntrip.station_label`, `ntrip.site` はUI向けの表示名（任意）。
UI詳細は [基地局表示](../robot_console/docs/gnss_station_ui.md)を参照。
