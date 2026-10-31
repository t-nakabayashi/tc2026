# rtk_gps_um982 詳細設計書

## 1. 目的・スコープ

本書は `rtk_gps_um982` パッケージの内部設計を定める。対象はドライバノード
`rtk_gps_um982_node` と、PTP 試験用の補助である。

トピック名、型、パラメータの一覧と起動方法は [README](../README.md) を正とする。設定値の
正本は [config/default.yaml](../config/default.yaml) である。時刻同期の運用手順は
[PTP 試験手順](PTP試験手順.md)を正とする。本書はそれらを再掲せず、**配信条件、メッセージ
変換の規則、NTRIP の判定、時刻の扱い、制約**を定める。

UM982 のシリアル入力から位置・デュアルアンテナ方位・RTK 状態を配信する。責務は次の 4 点で
ある。

- シリアルから受けた測位結果を ROS メッセージへ変換して配信する。
- NTRIP で受けた補正データを検査して受信機へ渡す。
- 補正通信と時刻配信の状態を診断として配信する。
- 時刻配信が有効なとき、RMC を chrony へ分配する。

非スコープは、車体座標への補正、GNSS と LIO の融合、TF の配信である。いずれも
`gnss_lio_fusion` が担う。**本ノードは TF も融合位置も配信しない。**

`third_party/UM982-RTK-GPS-Library` を git submodule として同梱し、`ntrip_client.py` の
補正クライアントが NTRIP 受信処理を補う。

記載内容はすべて実装済みである。独立基準での絶対時刻精度は未検証である（9.3 節）。

## 2. 構成とファイル配置

| モジュール | 責務 |
| --- | --- |
| `driver_node.py` | ROS 層。パラメータ、配信、診断、タイマー |
| `converters.py` | **ROS 非依存**。測位結果から ROS メッセージへの変換 |
| `ntrip_client.py` | NTRIP v1 の受信、RTCM3 のフレーミングと CRC 検査、再接続 |
| `time_sync_core.py` | **ROS 非依存**。RMC の検査と chrony SOCK への分配 |
| `time_sync_client.py` | 単一のシリアル読み取りから RMC を分配するクライアント |
| `clock_service.py` | 時刻同期サービスの補助 |
| `ptp_trial.py` | PTP の試験・監視 |

| パス | 用途 |
| --- | --- |
| `rtk_gps_um982/*.py` | 上記モジュール |
| `launch/rtk_gps_um982.launch.py` | 起動 |
| `config/default.yaml` | 設定の正本 |
| `third_party/UM982-RTK-GPS-Library/` | 受信機ライブラリ（submodule） |
| `test/` | 単体試験 |
| `docs/PTP試験手順.md` | 時刻同期の運用手順 |

**変換とRMC 検査は ROS 非依存である。** `rclpy` を初期化せずに単体試験できる。

## 3. 外部インタフェース仕様

一覧は [README](../README.md) を正とする。本章は配信条件と QoS を定める。

### 3.1 QoS

| 対象 | 設定 | 選定理由 |
| --- | --- | --- |
| すべての配信 | RELIABLE / VOLATILE / depth 10 | 測位結果と診断を取りこぼさないため |

### 3.2 トピック名

launch 既定のノードは `/rtk_gps/rtk_gps_um982_node` である。**トピック名はノード名を
含まず、実機・模擬とも `/rtk_gps/` 配下に公開する。** ノードは相対名で宣言し、namespace の
付与で `/rtk_gps/` 配下へ入る。

| トピック | 型 | 配信条件 |
| --- | --- | --- |
| `fix` | `sensor_msgs/NavSatFix` | 有効位置かつ最低品質以上の位置コールバック |
| `heading` | `sensor_msgs/Imu` | 同上。方位欠測は共分散で未提供と表現 |
| `rtk_status` | `rtk_gps_um982_msgs/RtkStatus` | 同上 |
| `ntrip_status` | `std_msgs/String` | 1 Hz の JSON。**位置欠測と独立** |
| `time_sync` | `std_msgs/String` | 時刻配信有効時、1 Hz の JSON |

3 種の配信は `publish.navsatfix`、`publish.imu_heading`、`publish.rtk_status` で個別に
無効化できる。無効にした種別は publisher を作らない。

### 3.3 位置系 3 種の配信条件

位置・方位・RTK 状態には**同じ stamp と `frame_id`** を付ける。次のいずれかに当たると
**3 種すべてを配信しない。**

1. 時刻配信が有効で、RMC アンカーから日付を確定できない（7.4 節）
2. 測位結果が無効
3. RTK 状態が `min_fix_for_publish` 未満（7.1 節）

**したがって `rtk_status` の無更新を品質良好と解釈してはならない。** 品質が低いときこそ
配信が止まる。測位の欠測は `ntrip_status` と併せて判断する（`ntrip_status` は位置とは
独立に 1 Hz で出る）。

### 3.4 診断

`/diagnostics` へ 2 つの観点を報告する。

| 観点 | 内容 |
| --- | --- |
| デバイス | 起動時に受信機への接続を OK として報告する。hardware_id はシリアルポート |
| 品質 | NTRIP の状態から 1 Hz で報告する。状態が未知なら WARN |

品質の報告値には NTRIP の状態と RTCM 累積バイトを入れる。監視側の扱いは
[ノード健全性監視設計](../../../docs/ノード健全性監視設計.md)を参照する。

### 3.5 画面・可視化

ノードは GUI を持たない。`robot_console` と `drive_mode_manager` の状態画面が
`rtk_status` と `ntrip_status` を購読して表示する。時刻同期の警告表示は
`robot_console` が別経路（ファイル）で行う。

## 4. 設定の所在

設定の正本は [config/default.yaml](../config/default.yaml) である。launch では `config` で
YAML を指定する。

```bash
ros2 launch rtk_gps_um982 rtk_gps_um982.launch.py config:=<設定YAMLの絶対パス>
```

**`serial_port` や `ntrip_host` という launch 引数はない。** 接続先は YAML で与える。
YAML のノードキーは `/rtk_gps/rtk_gps_um982_node` とする。

| パラメータ群 | 効く処理 |
| --- | --- |
| `serial.port`、`serial.baud` | 受信機への接続 |
| `output_rate_hz` | 受信機への出力レート設定 |
| `frame_id` | 位置系 3 種の header |
| `stamp_source`、`transport_delay_ms` | stamp の決め方（7.3 節） |
| `time_sync.enabled`、`time_sync.chrony_socket` | RMC の分配（7.4 節） |
| `ntrip.enabled`、`ntrip.host`、`ntrip.port`、`ntrip.mountpoint`、`ntrip.user`、`ntrip.password` | NTRIP 接続（7.5 節） |
| `ntrip.station_id`、`ntrip.station_label`、`ntrip.site` | 診断に載せる基地局の表示情報 |
| `publish.*` | 配信の有効化（3.2 節） |
| `min_fix_for_publish` | 品質フィルタ（7.1 節） |
| `hdop_sigma` | 水平分散の係数（7.2 節） |

### 4.1 既定値の差と優先順位

**シリアルポートの既定が 2 系統ある。** YAML の既定は USB の固定名、ノード単独の宣言既定は
`/dev/ttyUSB0` である。`ros2 launch` では YAML が有効になる。

launch の `time_sync` と `chrony_socket` 引数は、**YAML 内の該当設定より優先する。**

### 4.2 変更できない範囲

**パラメータは起動時に読み込む。** ランタイムの `ros2 param set` で接続先や品質フィルタの
実処理が切り替わる仕組みはない。

NTRIP の再接続の待機時間、有効 RTCM の途絶判定、RMC アンカーの有効期限はいずれも実装内の
固定値である。値は 7 章に置く。

## 5. データモデル・内部状態

| 状態 | 初期値 | 更新契機 | クリア条件 |
| --- | --- | --- | --- |
| 受信機クライアント | 起動時に生成・接続 | — | 終了時に停止 |
| RMC アンカー（日付と受信時刻） | なし | 有効 RMC の受信 | なし（古さで無効扱い） |
| RMC の送信数・拒否数・SOCK エラー | 0 / 0 / なし | 分配のたび | なし |
| NTRIP の接続状態 | 未接続 | 接続・切断 | — |
| RTCM の累積バイト | 0 | CRC 一致フレームの受領 | なし |
| 再接続回数、エラー種別 | 0 / なし | 再接続のたび | なし |
| 最終有効 RTCM 時刻 | なし | CRC 一致フレームの受領 | 接続のたびに初期化 |
| 再接続の待機時間 | 1 秒 | 失敗ごとに倍、成功で 1 秒へ戻す | — |

**測位結果を保持しない。** 位置コールバックのたびに変換して配信し、前回値を残さない。
下流が古さを判定するための stamp は付けるが、ノード側では鮮度を判定しない。

## 6. 処理フローと異常時の挙動

### 6.1 起動

1. パラメータを読む。
2. 配信が有効な種別の publisher を作る。
3. 時刻配信が有効なら、RMC 分配器と `time_sync` の publisher・1 Hz タイマーを作る。
4. 受信機クライアントを生成して接続し、位置コールバックを登録する。
5. 診断にデバイス接続を OK として報告する。
6. NTRIP が有効なら接続を開始する。
7. `ntrip_status` の 1 Hz タイマーを作る。

### 6.2 位置コールバック

受信機ライブラリの**シリアル読み取りスレッドから呼ばれる。**

1. 時刻配信が有効なら、RMC アンカーで日付を確定して stamp を作る。**確定できなければ
   何も配信せずに戻る。**
2. 測位結果が無効なら戻る。
3. 品質フィルタを通らなければ戻る（7.1 節）。
4. stamp を決める（7.3 節）。
5. 有効な種別について変換して配信する。

### 6.3 異常時の挙動

| 事象 | 挙動 |
| --- | --- |
| シリアルを開けない | 受信機クライアントの生成で失敗する。ノードは起動しない |
| 測位結果が無効 | 位置系 3 種を配信しない（3.3 節） |
| 品質が `min_fix_for_publish` 未満 | 同上 |
| RMC アンカーが古い | 同上。`time_sync` では鮮度 false を報告する |
| 方位が欠測 | `heading` は配信するが、姿勢の共分散先頭を −1 として未提供を表す（7.2 節） |
| NTRIP の host または mountpoint が空 | **error を出して接続しない。** 測位は続く |
| NTRIP の HTTP 応答が不正 | 接続を閉じ、待機して再接続する |
| RTCM の CRC 不一致 | そのフレームを捨てる。受信機へ渡さない |
| 有効 RTCM の途絶 | 接続を切って再接続する（7.5 節） |
| chrony SOCK への送信失敗 | 拒否数・エラーを記録し、nonblocking で再試行する |
| 終了時に受信機の停止が例外を投げる | warn を出して処理を続ける |

**測位と補正通信は独立に失敗し得る。** 位置トピックと `ntrip_status` を併せて、測位欠測と
補正通信の欠測を切り分ける。

### 6.4 終了処理

受信機クライアントを停止し、RMC 分配器を閉じてからノードを破棄する。**終了時に安全値を
配信しない。** 最後の測位結果が下流に残る。

### 6.5 ログ

受信機への接続と NTRIP の接続状況を `info` で出す。NTRIP の設定不足は `error`、
受信機停止時の例外は `warn` を使う。`debug` と `fatal` は使わない。

異常の通知は主に診断と `ntrip_status` の JSON で行う。

## 7. 判定ロジックと閾値

### 7.1 品質フィルタ

RTK 状態を順序のある 5 段（未知 → 単独 → DGPS → FLOAT → FIX）として扱い、
`min_fix_for_publish`（既定 `standalone`）以上であれば配信する。

受信機が返す状態文字列は複数の表記を受け付ける。`rtk_float` と `float` はいずれも FLOAT、
`rtk_fix` と `fix` はいずれも FIX へ写像する。

### 7.2 メッセージ変換

**NavSatFix**

| 項目 | 規則 |
| --- | --- |
| 緯度・経度・高度 | 受信機の値をそのまま使う |
| 水平分散 | `(HDOP × hdop_sigma)²` |
| 垂直分散 | 水平分散の 4 倍。**VDOP は使わない** |
| 状態 | FLOAT と FIX はいずれも `GBAS_FIX`、DGPS は `SBAS_FIX`、単独測位は `FIX`、未知は `NO_FIX` |

**FLOAT と FIX を状態で区別できない。** 区別が必要な下流は `rtk_status` を見る。

**Imu（方位）**

| 項目 | 規則 |
| --- | --- |
| 方位の定義 | 真北から時計回りの角度 |
| ENU yaw への変換 | `π/2 − heading_rad` |
| pitch | 使用する |
| roll | **0 と仮定する** |
| yaw / pitch の分散 | 各標準偏差を rad へ変換した二乗。値がなければ 0.01 |
| roll の分散 | 1e6（未計測を表す大きな値） |
| 方位が欠測 | 姿勢の共分散先頭を −1 |
| 角速度・加速度 | **未計測。** それぞれの共分散先頭を −1 |

**RtkStatus**

状態は未知 0、単独 1、DGPS 2、FLOAT 3、FIX 4 とする。生の状態文字列、衛星数、HDOP、
方位・pitch と各標準偏差、基線長、補正経過時間、RTCM 累積バイト、位置を格納する。
補正経過時間が不明な場合は −1.0 とする。フィールド定義は
[RtkStatus.msg](../../rtk_gps_um982_msgs/msg/RtkStatus.msg)を正とする。

### 7.3 stamp の決め方

| `stamp_source` | 動作 |
| --- | --- |
| `gnss_utc` | 測位結果の時刻に `transport_delay_ms` を加算する。**共通起動は加算 0** |
| `ros_time` | 受信処理時刻を付ける。共通起動では使わない |
| `pps_edge` | **PPS 処理は未実装で、受信処理時刻へフォールバックする。** 共通起動では使わない |

共通実機起動は `time_sync.enabled=true`、`stamp_source=gnss_utc`、
`transport_delay_ms=0` を強制する。

**方位と位置の計測 epoch は厳密には結合されていない。** 同じ stamp を付けるが、受信機内部
での計測時刻が一致することを確認していない。

### 7.4 GNSS 時刻と RMC の分配

有効な RMC から日付を確定し、GGA の測定時分秒と結合して UTC を作る。**アンカーが 2 秒より
古い場合は位置の配信を止める。** 日付のない時刻を配信しないためである。

RMC の検査はチェックサム・状態・暦日の 3 点で行う。**単一のシリアル読み取りから RMC を
分配する。** 受信機を 2 回開かないためである。

chrony SOCK への送信は nonblocking で行い、失敗時は再試行する。`time_sync` の JSON には
RMC の鮮度・経過秒・送信数・拒否数・SOCK エラーを入れる。**精度の検証済みフラグは常に
false である。**

RMC SOCK は chrony 側で `noselect` とし、**PC の同期元から除外する。** PC は chrony で
外部 NTP に同期する。

### 7.5 NTRIP の受信と再接続

NTRIP v1 のみを扱う。**TLS と HTTP chunked 転送は非対応である。**

1. HTTP 応答の先頭行を検査する。`ICY` または `HTTP/1.0` / `HTTP/1.1` で、状態コードが
   200 であることを要求する。
2. TCP の分割・結合された入力から RTCM3 のフレームを復元し、**CRC-24Q が一致したフレーム
   だけ**受信機へ渡す。
3. 有効な RTCM が **15 秒**途絶した接続を切る。
4. 再接続の待機は **1 秒から始め、失敗ごとに倍にして最大 30 秒**とする。成功すると 1 秒へ
   戻す。

`ntrip_status` の JSON には接続状態、CRC 確認済み RTCM の量・受信速度・経過時間、
再接続回数、エラー種別、起動時の基地局表示情報を入れる。**認証情報は含めない。**

### 7.6 PTP と起動監視

PC は chrony で外部 NTP に同期し、別サービスが software PTP で LiDAR へ PC 時刻を配信
する。NTP の選択・応答鮮度・残補正・推定誤差を確認してから PTP を配信する。

AF_PACKET で点群・IMU を受動監視し、PTP 種別・受信継続・受信時刻差・逆行を判定する。
**0.1 ms 以下の微小な逆行は記録のみとし、原時刻を書き換えない。**

共通 bringup は同期成立後に車輪・FAST-LIO・融合・走行制御を起動し、運用中の条件喪失で
停止する。GUI は未成立・喪失・監視更新停止を全タブ上部に警告する。

設定・自動処理・起動操作・復旧・確認範囲は
[PTP 試験手順](PTP試験手順.md)を正とする。

## 8. 並行性・周期・起動・依存関係

### 8.1 周期

| 項目 | 値 |
| --- | --- |
| 位置系 3 種の配信 | 受信機の出力レート（`output_rate_hz`、既定 10 Hz） |
| `ntrip_status` と品質診断 | 1 Hz |
| `time_sync` | 1 Hz |
| 有効 RTCM の途絶判定 | 15 秒（固定） |
| 再接続の待機 | 1〜30 秒（固定、指数） |
| RMC アンカーの有効期限 | 2 秒（固定） |

### 8.2 並行性

スレッドが 3 系統ある。

| スレッド | 役務 |
| --- | --- |
| ROS の executor | タイマー（診断と 2 つの JSON 配信）、spin |
| 受信機ライブラリのシリアル読み取り | 位置コールバックの呼び出し、RMC の分配 |
| NTRIP クライアント | 接続・受信・CRC 検査・再接続 |

**位置系 3 種は ROS のコールバックではなく、シリアル読み取りスレッドから配信する。**
`rclpy` の publisher はスレッドセーフであるため直接呼ぶ。executor の周期とは独立に、
受信機のレートで配信される。

RMC の分配もシリアル読み取りスレッドで行う。SOCK 送信を nonblocking にするのは、
**この スレッドを塞がないため**である。塞がると位置の配信が止まる。

### 8.3 起動

entry point はドライバノードである。launch の引数は [README](../README.md) を正とする。

**同じシリアルポートを複数ノードや端末プログラムで同時に開かない。** 受信機は 1 プロセス
からのみ開く。時刻配信が有効な場合も、単一の読み取りから RMC を分配する（7.4 節）。

実機一式は [共通起動](../../icart_bringup/README.md)から使用する。

### 8.4 依存関係

`ament_python` でビルドする。**依存はすべて実行時依存として宣言する。** `rclpy`、
`tc_diagnostics`、`sensor_msgs`、`std_msgs`、`geometry_msgs`、`rtk_gps_um982_msgs`、
`python3-serial` である。

受信機ライブラリは `third_party/UM982-RTK-GPS-Library` の submodule を使う。**submodule を
初期化していない環境では import に失敗する。** `git submodule update --init --recursive`
が必要である。

`ntrip_client.py` と `time_sync_core.py` は標準ライブラリのみで動く。CRC-24Q を自前で
計算し、外部ライブラリに依存しない。

## 9. 確認方法と未確認範囲

### 9.1 自動確認

`test/` が ROS 非依存または subprocess 分離で動く。実機デバイスは開かない。

| 試験 | 固定する内容 |
| --- | --- |
| `test_converters.py` | メッセージ変換と品質フィルタ。分散の計算、状態の写像、方位欠測の表現 |
| `test_driver_topics.py` | 名前空間 `/rtk_gps` 配下の公開名で配信すること。**受信機クライアントを差し替えて実機を開かない** |
| `test_ntrip_client.py` | 分割された TCP 入力からの RTCM 復元、CRC 不一致の破棄、無効な HTTP 応答の扱い |
| `test_ntrip_status.py` | 診断 JSON の値と、認証情報を含めないこと |
| `test_time_sync.py` | RMC の検査（チェックサム・状態・暦日）とアンカーの有効期限 |
| `test_clock_service.py`、`test_clock_setup.py`、`test_ntp_poll_config.py` | 時刻同期サービスの設定と起動条件 |
| `test_ptp_trial.py` | PTP の判定（種別・受信継続・時刻差・逆行） |

`test_driver_topics.py` は **subprocess で実行する。** `rclpy.init()` が生成する DDS の
内部スレッドが `rclpy.shutdown()` 後も残留し、親プロセスに残ると後続の `os.fork()` を
使う試験がデッドロックするためである。規則と検査はワークスペース直下の `conftest.py` を
参照する。

```bash
colcon test --packages-select rtk_gps_um982
colcon test-result --verbose
```

### 9.2 手動で確認する範囲

実機では完全名で位置・方位・RTK 状態・NTRIP 診断を照合し、次を確認する。

- アンテナ配置と方位の符号
- `min_fix_for_publish` による配信の停止と再開
- NTRIP の接続成立、CRC 一致フレームの受領、途絶からの再接続
- NTRIP の設定不足（host / mountpoint が空）で error が出て測位は続くこと
- 時刻配信有効時の RMC 鮮度と chrony SOCK の送信数
- `/diagnostics` の報告（デバイスと品質）

**FIX 到達や方位の精度は通信の成功だけでは保証されない。**

### 9.3 未確認範囲

- **独立基準での絶対時刻精度は未検証である。** `time_sync` の精度検証済みフラグは常に
  false である（7.4 節）。
- **方位と位置の計測 epoch の一致を確認していない**（7.3 節）。
- 垂直分散を水平分散の 4 倍とする近似（7.2 節）が実測の鉛直誤差と合うかを確認していない。
- `hdop_sigma` の既定値が実機の誤差分布に合うかを確認していない。
- 15 秒の RTCM 途絶判定と 30 秒上限の再接続待機が、実際の基地局・回線に対して適切かを
  確認していない。
- `pps_edge` は未実装であり、確認していない（7.3 節）。
- 長時間運用での再接続回数とメモリ使用を確認していない。

## 10. 影響範囲・未決事項

### 10.1 影響範囲

`rtk_gps_um982_msgs/RtkStatus` のフィールドを変更する場合、`robot_console`、
`drive_mode_manager` の状態画面、`gnss_lio_fusion` を併せて更新する。

`/rtk_gps/` 配下の公開名（3.2 節）は、`geo_pose_converter`、`gnss_lio_fusion`、
`robot_console`、`drive_mode_manager` が前提にする。namespace を変える場合は購読側の
remap を同時に更新する。

**`gnss_lio_fusion` は `NavSatFix` と `RtkStatus` を直接読む。** `geo_pose_converter` を
経由しない。変換規則（7.2 節）を変える場合は両方の購読側を確認する。

`ntrip_status` と `time_sync` の JSON の形は `robot_console` の表示が前提にする。
**認証情報を含めない規則**は維持する。

submodule のポインタを進める場合、位置コールバックの構造体のフィールドが変わる可能性が
あるため、変換処理と試験を併せて確認する。

### 10.2 未決事項

| 件 | 選択肢 | blocker |
| --- | --- | --- |
| 7.3 節: `pps_edge` が未実装である | PPS の処理を実装する／選択肢から削る | ではない。共通起動では使わない |
| 7.4 節: 絶対時刻精度の検証手段がない | 独立基準での測定手順を用意する／検証済みフラグを削る | ではない |
| 7.2 節: FLOAT と FIX を NavSatFix の状態で区別できない | 区別を入れる／`rtk_status` を正として現状を維持する（現状） | ではない |
| 7.2 節: 垂直分散が水平分散の 4 倍の近似である | VDOP を使う／実測から係数を決める | ではない |
| 4.2 節: パラメータが起動時のみ読み込まれる | 動的再設定に対応する／再起動を前提として明示する（現状） | ではない |
