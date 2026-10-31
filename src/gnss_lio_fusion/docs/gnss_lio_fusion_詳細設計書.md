# gnss_lio_fusion 詳細設計書

## 1. 目的・スコープ

gnss_lio_fusionは、GNSSの絶対位置・二アンテナ方位とFAST-LIOの相対移動を融合する。
対象はi-Cart miniの平面走行で、厳しめセンサ条件を評価の基準にする。
本書は既存geo_pose_converter、obstacle_route_simの詳細設計書、
FASTLIOセンサ誤差モデル評価、全域FASTLIO点群地図評価を参照する。
実機校正済みの誤差モデル、3D点群地図の最適化は対象外とする。
車輪は異常LIOの代替相対運動として使用し、常時独立観測として二重投入しない。

本パッケージはROSパラメータの一覧をREADMEに持たない。**設定値の正本は本書の4章である。**
運用手順と起動例は[README](../README.md)、評価方法と判定範囲は
[実装評価](実装評価.md)と[方位実装評価](方位実装評価.md)を正とする。

要求と方針は次のとおりである。

GNSSの品質指標はFIX/FLOAT、衛星数、受信機が推定したアンテナ間距離とする。
FIX/FLOAT 以外は位置・方位の両観測を棄却し、残存する方位精度値で補正しない。
基線長は起動設定を初期参考値とし、良好観測から基準値を学習する。
単体設定は0.5 m、共通実機設定は0.15 m（広域許容範囲0.105〜0.195 m）である。
低信頼時はLIO優先で減速し、自己位置の不確かさだけを理由に停止しない。
非数値・物理的に不可能なジャンプ・センサ出力の完全途絶は精度低下と区別する。
URGによる停止や非常停止を解除しない。測位と障害物回避は別責務とする。

記載内容はすべて実装済みである。実機rosbagによる校正は行っていない（9.3節）。

## 2. 構成とファイル配置

baseline_coreは良好観測から基準距離・ばらつきを推定する。
fusion_coreは状態[x, y, yaw]と3×3共分散で相対運動を予測する。
heading_coreは位置観測から独立した方位更新を担当し、位置biasによる誤旋回を防ぐ。
motion_guard_coreは車輪とLIOの相対移動を比較し、異常時に連続な予測入力へ切り替える。
fusion_nodeは時刻同期・取付補正・ROS通信・記録・速度上限制御を担当する。
補正量制限と適応baselineの状態遷移を専用EKFで扱う。
**FAST-LIO本体へGPSをフィードバックせず、登録済み点群地図も書き換えない。**

| 配置 | 役割 |
| --- | --- |
| gnss_lio_fusion/baseline_core.py | ROS非依存の基準距離学習 |
| gnss_lio_fusion/fusion_core.py | ROS非依存の平面予測・位置更新 |
| gnss_lio_fusion/motion_guard_core.py | LIO/車輪の運動照合・異常時退避 |
| gnss_lio_fusion/heading_core.py | 円周上の方位更新・観測反転棄却 |
| gnss_lio_fusion/fusion_node.py | ROS入出力・遅延処理・減速 |
| gnss_lio_fusion/gravity_alignment_node.py | 鉛直基準の確定とLIO姿勢の水平化（6.5節） |
| params/default.yaml、launch/fusion.launch.py | 設定・融合のみの起動 |
| tests/ | 異常系・遷移・座標と時刻の検証、既知の欠陥の固定 |
| docs/ | 設計・確認方法 |

実機と模擬環境で共有する責務のため、シミュレータtools内ではなく独立パッケージとする。
模擬GPSと評価ツールはobstacle_route_sim内に配置する。生成物はGitに追加しない。

## 3. 外部インタフェース仕様

単独起動・共通実機起動とも `/rtk_gps/fix` と `/rtk_gps/rtk_status` へ接続する。

### 3.1 QoS

**購読センサはRELIABLE / VOLATILE / depth 30、出力と指令はdepth 10とする。** センサの
depthを大きく取るのは、時刻順処理バッファへ入れる前に取りこぼさないためである。

GNSSとLIOのclockが異なる状態では使用しない。**受信時刻で観測時刻を上書きしない。**

### 3.2 トピック

| 名前 | 型 | 方向・用途 |
| --- | --- | --- |
| /ypspur_ros/odom | nav_msgs/Odometry | 購読、車輪の相対運動と異常時退避 |
| /lio/odometry | nav_msgs/Odometry | 購読、IMU位置のLIO姿勢（水平化後） |
| /rtk_gps/fix | sensor_msgs/NavSatFix | 購読、マスターGNSS位置 |
| /rtk_gps/rtk_status | rtk_gps_um982_msgs/RtkStatus | 購読、同一stampの状態・衛星数・baseline・方位 |
| /localization/pose_enu | geometry_msgs/PoseWithCovarianceStamped | 配信、車軸基準のmap平面姿勢 |
| /fusion/status | std_msgs/String | JSON診断、基準距離・状態・不確かさ・棄却数 |
| /fusion/gnss_dropout_active | std_msgs/Bool | 配信、GNSS途絶模擬の状態（6.6節） |
| /cmd_vel/autonomous | geometry_msgs/Twist | 購読、自律指令 |
| /cmd_vel/fusion_limited | geometry_msgs/Twist | 配信、速度制限後の自律指令 |

**独自msg・service・actionは持たない。真値、建物配置、予定経路を融合ノードへ入力しない。**

UM982 driverはfix/statusに同じstampを使用する。**異なるstampの別ドライバは未対応である。**
ROS側のstampが一致していても位置と方位が同時刻の観測である保証はない（10.2節）。

`/fusion/status` に出す項目は、heading_sigma_deg、heading_reason、
heading_accepted/rejected、sigma、mode、速度上限、基準距離、ready、再取得数、棄却数、
遅延数、motion_source、wheel_fallback_count、motion_rejected_count、
motion_window_rejectionsである。

### 3.3 画面・可視化

obstacle_route_sim/tools/review_fusion_trial.pyで同一走行のGPS・LIO・融合を真値時刻で
比較する。融合・GPSに真値位置合わせを行わず、LIOのみ初回XY/yawを合わせる。
0.5秒以上の入力間隔を跨ぐ補間は精度比較から除外し、件数と最大間隔を別記する。
経路横ずれは連続線分への最短距離で、車体中心の指標である。
方位のRMSE・P95・最大誤差も最短角で算出する。

robot_consoleは融合yawと推定方位σ・baselineを表示し、GNSS北CW方位と区別する。
会話内画像を用意し、専門形式やPC内のファイルリンクだけで閲覧を完結させない。

## 4. 設定の所在

本章が設定値の正本である。**パラメータは起動時に読み、動的変更を行わない。**
EKF係数は正の有限値を要求する。

| 名前 | 既定値・単位 | 意味 |
| --- | --- | --- |
| origin_latitude/longitude/altitude、map_yaw_offset_rad | 0、度/m/rad | ProjectionConfigと同じ地図原点。起動時に必ず設定する |
| master_forward_m/left_m/height_m、lio_forward_m/left_m/height_m | 単体の高さ既定0.7 / 0.6 m | 車軸から主アンテナ／IMU原点への3D位置。実機は生成設定を使用 |
| lio_mount_roll_deg/pitch_deg/yaw_deg | 0 deg | IMUの車体に対する取付角 |
| publish_base_tf | false | map→base_linkの平面TFを配信 |
| gnss_heading_offset_deg / lio_yaw_offset_deg | 0 / 0 deg | ENUへ変換したyawに加える取付補正。反時計回り正 |
| wheel_fallback_enabled | true | 車輪照合・退避。falseならLIO単独予測 |
| buffer_s | 0.35 s | GNSS遅延を吸収する時刻順処理バッファ、0より大きく2秒以下 |
| output_log / baseline_cache | 空文字 | 任意の記録先・前回参考値保存先。空なら書き込まない |
| baseline.initial_m / minimum_m / maximum_m | 0.5 / 0.35 / 0.65 m | 初期参考値と広い物理範囲 |
| baseline.min_satellites | 14 | FIX時の学習に必要な衛星数 |
| baseline.initialize_s / reacquire_s / adapt_tau_s | 2 / 8 / 60 s | 初期学習・新候補再取得・通常更新時定数 |
| baseline.tolerance_min_m / tolerance_max_m | 0.01 / 0.03 m | 許容幅の下限・上限 |
| baseline.stable_sigma_m | 0.004 m | MADから求める安定候補のばらつき上限 |
| position_sigma_fix / position_sigma_float | 0.15 / 3.0 m | 観測標準偏差の下限 |
| heading_sigma_fix_deg / heading_sigma_float_deg | 1 / 15 deg | 方位観測標準偏差の下限 |
| lio_position_sigma_per_sqrt_m | 0.12 | 移動距離に応じた位置過程雑音係数 |
| lio_yaw_sigma_per_sqrt_s | 0.025 | 時間に応じたyaw過程雑音係数 |
| max_lio_speed_mps / max_lio_yaw_rate | 2 m/s / 2 rad/s | 相対移動の異常判定に使用 |
| max_lio_gap_s / gps_timeout_s | 1.5 / 1.5 s | LIO更新途絶・良好GPS期限 |
| correction_speed_mps / correction_yaw_rate | 0.5 m/s / 0.12 rad/s | GPSによる位置・方位補正の上限 |
| recovery_s | 2 s | 大きく食い違う良好GPSを再取得する安定期間 |

**共分散は実機で校正済みの誤差上限ではない。減速判定の暫定指標として扱う。**

速度上限、Mahalanobis距離、方位の棄却角、MotionGuardの照合閾値はパラメータを持たない。
値と理由は7章に置く。

## 5. データモデル・内部状態

LIOの3D姿勢からIMU取付角を除き、車体姿勢で回転したIMU位置オフセットを引いて車軸位置を求める。
GNSSは受信機yawと時刻内挿した車体roll/pitchで主アンテナの3Dオフセットを回転して補正する。
実機の各値は[ハードウェア設定](../../icart_bringup/docs/実機ハードウェア統合.md)から生成する。

地図yawオフセットを位置と方位の両方に適用する。yawは±πに正規化する。
**z・roll・pitchは融合状態に含めず、出力共分散には大きな値を設定する。**

| 保持する状態 | 寿命 |
| --- | --- |
| LIO履歴 | 5秒 |
| status | 100件 |
| baseline候補 | 最大約9秒 |
| MotionGuardの照合履歴 | 1.5秒 |

**baseline保存値は次回の参考値にしか使わず、ready状態を持ち越さない。**

## 6. 処理フローと異常時の挙動

### 6.1 周期処理

受信→stamp順に待機→GPS観測時刻へLIOを内挿→相対移動のEKF予測→GPS更新→
バッファ内の最新LIOまで予測→ENU poseと診断を配信する。

**同じ最新stampを繰り返し配信しない。**

### 6.2 状態遷移

| 状態 | 条件・動作 |
| --- | --- |
| GPS_LIO | 最近の良好GPSを使用でき、LIO更新が有効 |
| LIO_PRIORITY | 良好GPSがないが、LIOの相対運動を採用可能 |
| LIO_FAULT | 不可能なLIOジャンプ等を検出。異常変位を捨て、新原点から次の相対運動を待つ |
| GPS_WHEEL / WHEEL_PRIORITY | 車輪退避中（7.5節） |
| WAIT_ALIGNMENT | 鉛直基準が未確定（6.5節） |
| WAIT_INITIAL_FIX | 良好FIX待ち（6.5節） |

### 6.3 速度上限

| 条件 | 上限 |
| --- | --- |
| 良好GPSかつ位置σ0.5 m未満、方位σ5度未満 | 1.1 m/s |
| FLOATなど低信頼時 | 0.6 m/s |
| 位置σが1 m以上 | 0.25 m/s |
| 車輪退避中 | 0.4 m/s |

**直進・旋回を同率で縮小し曲率を保つ。** 自己位置の不確かさだけを理由に停止しない。

### 6.4 異常時の挙動

| 事象 | 挙動 |
| --- | --- |
| LIOも車輪も内挿できないGPS、status不一致、処理済み時刻より古い観測 | 棄却して件数を記録する |
| LIOが途絶して0.4秒超 | 新しい有効な車輪odomがあれば車輪相対運動で継続する |
| LIOと車輪の双方が `max_lio_gap_s` 以上古い | pose配信を止める |
| 有効な運動を選べず出力が古い | 速度指令を0にする |
| 非有限姿勢、範囲外baseline、逆行時刻、入力不足 | 除外する |
| 始動時に位置・方位を初期化できない | poseを出さない |
| 保存したbaseline値が不正 | warnを出す |
| `buffer_s` を超えた通信遅延 | 再処理せずlate_eventsに記録する |

**車輪スリップや車輪とLIOの共通誤差は、この照合だけでは区別できない。**

FAST-LIO内部の点群照合残差・退化指標はまだ入力しておらず、**LIO判定は時刻・変位の監視で
ある。**

起動・終了・状態遷移はinfoを出す。

### 6.5 重力整列と初期化

共通bringupとfusion.launchはgravity_alignment_nodeを起動し、FAST-LIOの
/lio/odometry_rawを水平化した/lio/odometryへ接続する。
約2秒の静止IMU・LIO・車輪観測から鉛直基準を確定する。
位置・姿勢・pose共分散を回転し、**body点群とchild座標のtwistは回さない。**
/cloud_registeredはraw worldのままである。

基準未確定時はWAIT_ALIGNMENT、良好FIX待ちはWAIT_INITIAL_FIXを診断表示し、**自律指令を
0にする。**

**LIO再起動・時刻逆行・frame変更・1.5秒超のLIO断で基準を破棄する。**
準備解除やheartbeat途絶で融合履歴を捨て、新しい基準と初期FIXを待つ。

### 6.6 GNSS途絶模擬

R1（Joyボタン5）保持中は**融合へのGNSS入力のみ破棄する。**
解除・Joy受信0.5秒途絶で再開し、切替時に未処理GNSSと品質キャッシュを捨てる。
判定は単調時計で行う。**生GNSS・NTRIP・LIO・車輪を停止しない。**
/fusion/gnss_dropout_activeで模擬状態を配信する。

### 6.7 終了処理

記録をcloseし、確定したbaseline参考値を指定されたcacheへ原子的に保存する。

## 7. 判定ロジックと閾値

### 7.1 baselineの学習

baselineはFIX・衛星数 `baseline.min_satellites` 以上・物理範囲内の観測で学習する。
**15標本以上**、中央値・MADによるばらつきが安定し、`baseline.initialize_s` 以上続けば
初期値を確定する。
既存値から離れた観測は別候補とし、`baseline.reacquire_s` の安定後に再取得する。
FLOATや**0.5秒超**の間隔で連続候補をクリアする。通常更新は `baseline.adapt_tau_s` の
時定数とし、異常値で許容幅を無制限に広げない。

baselineスコアは `exp(−0.5×(偏差/許容幅)²)` で、**方位の重みに強く、位置には弱く反映する。**

### 7.2 EKFの予測と位置更新

EKFは前回LIO座標を、その時点の推定方位との差で回転した相対変位を積分する。
**同じIMUを独立観測として二重投入しない。** GNSS位置・方位の共分散を状態、衛星数、
baselineスコアで調整し、受信機共分散が下限より大きければ尊重する。

位置残差の**Mahalanobis二乗が25**を超えたFIXは一旦棄却するが、良好GPSが `recovery_s`
継続すれば再取得を許す。FLOATは観測分散を残差に応じて増やし、**弱い絶対位置の拘束として
残す。** 連続GPS観測の急な**2 m超**の飛びも再取得待ちにする。

補正は**1回のdtを最大0.1秒**として制限し、gain自体を縮小してJoseph形式で共分散更新する。

### 7.3 方位の独立更新

初期化はFIX・**8衛星以上**・baselineスコア**0.05以上**・受信機方位σが0より大きく
**15度以下**を要求する。

**方位は位置の品質ゲートより先に独立更新する。** 0/NaN等の不明σは方位更新に使わない。
8衛星未満、baselineスコア0.05未満、方位σ**30度超**の観測も使わない。
±πをまたぐ差は最短角で処理し、予測と**45度**を超えて食い違う方位は棄却する。
連続観測の変化が最大旋回速度と観測分散から算出した許容値を超える場合も棄却する。
**棄却観測を直後の正常観測の判定基準にしない。**

LIOの時刻・変位ゲートで異常を検出した後は、方位再取得を許可する。
良好FIXの方位が `recovery_s` 間、最大旋回速度と整合して継続したときだけ45度超も補正対象に
戻す。**位置・方位とも補正速度上限を維持するため、急な大角度の真値旋回には復帰時間が
かかる。LIO異常が検出されないまま45度以上乖離した場合の自動再初期化は対象外である。**

**位置更新からyawへのgainを使わず、位置更新時に位置・方位の相互共分散をリセットする。**
これは厳密な最適同時推定ではなく、共通GPS biasの影響を限定する保守的な分離更新である。

### 7.4 相対運動の選択

車輪は**最大0.2秒間隔**内でLIO/GPSと同じ時刻へ内挿する。前回採用時刻と一致しない
odom基準からの差分を二重積分しない。両odomはそれぞれの前回yawでbody相対変位へ変換する。

不適格とする運動は、`max_lio_speed_mps`×dt**+0.15 m**、`max_lio_yaw_rate`×dt
**+0.15 rad**、最大間隔 `max_lio_gap_s` を越えるものである。

同時刻の車輪とLIOが位置**0.08 m+0.6 m/s×dt**または角度**0.08 rad+0.3 rad/s×dt**を超えて
食い違う場合はLIOを疑い、車輪の運動を選ぶ。**1秒間**の整合が戻るまで車輪を保持する。
各絶対odom原点へコピーせず、採用した相対変位を連続座標へ積分する。

MotionGuardは短い差分に加え、**約1秒の相対運動差**も照合する。履歴上限1.5秒、
並進**0.12+0.18dt m**・方位**0.1+0.1dt rad**、整合復帰待ち1秒とする。

**これは独立した絶対正解との照合ではない。車輪スリップ時に正しいLIOを除外し得る。
LIOの穏やかな漂流や両入力の共通異常を見抜く保証もない。MotionGuardの閾値は未校正である。**

LIO不在中のGNSS lever arm補正は**roll/pitch=0を仮定**し、高さの二乗を位置観測分散へ
加算する。

ログのkind=lioは未加工LIO、kind=wheelは車輪、kind=motionは採用済み相対運動を意味する。
**精度比較のLIO欄へ車輪退避後のmotionを混ぜない。**

## 8. 並行性・周期・起動・依存関係

### 8.1 周期と並行性

**単一executorでcallbackを実行し、共有状態の並行変更を行わない。**
**20 ms timer**でバッファを処理する。use_sim_timeはシミュレーション時だけtrueとする。

### 8.2 起動

```bash
ros2 launch gnss_lio_fusion fusion.launch.py projection_params:=<ENU原点を含むYAML> use_sim_time:=true
```

ドライバ、FAST-LIO、geo_pose_converterの原点設定とremapは利用側で揃える。
**GNSS単独位置は/gnss/pose_enuへ出し、/localization/pose_enuは融合ノードだけが配信する。**
自律mux入力cmd_vel/autonomousを/cmd_vel/fusion_limitedへremapして減速を有効にする。
**マニュアル指令やモード切替を融合ノードから変更しない。**

### 8.3 依存関係

ROS Jazzy、rclpy、標準ROSメッセージ、rtk_gps_um982_msgs、geo_pose_converter、NumPyを使用する。
NumPyの依存はワークスペースのrequirements.txtで管理する。
通常とsymlinkの両方式でgnss_lio_fusion、obstacle_route_simをビルドする。

## 9. 確認方法と未確認範囲

### 9.1 自動確認

baselineの初期取得、運搬後変化、FLOAT凍結、衛星不足、外れ値、NaN、逆行時刻を検証する。
GPS途絶中の継続、GPS復帰の補正上限、共分散正定性、LIOジャンプ、方位折返し、
測定時刻内挿、IMU取付高さの補正を単体テストする。

既知の欠陥（10.2節）は `tests/test_known_limitations.py` が**望ましい挙動をassertして
`xfail(strict=True)` で固定する。** 修正されるとxpassになり試験が失敗するため、そこで
初めてマーカーと10.2節の記述を外す。

実際のsubscribe/publish経路を確認する試験は、pytestの親プロセスではなくsubprocessで
ROSを起動する。ROS contextの初期化で生じるDDSスレッドは`rclpy.shutdown()`後も残留し、
親プロセスに残ると後続の`pytest-forked`が`os.fork()`した子プロセスをデッドロックさせる。
規則の検査はワークスペース直下の`conftest.py`が行う。

### 9.2 模擬試験

模擬試験は独立DDS domain/Gazebo partition、期限付きで実行し、起動したプロセスを回収する。
完走、実際のゴール距離、接触、最大横ずれ、±1 m以内の割合、測位誤差を分けて報告する。
**完走しても経路外へ出た試験を安全な走行として合格扱いにしない。**

検証はobstacle_route_simのevaluate_terrain_trial.pyに--gnss --fusion --fastlioを指定する。

### 9.3 未確認範囲

- **FIXでも2本が同方向へずれる場合、正常baselineだけではGPSの共通誤差を見抜けない。**
- **GPS/FLOATとLIOの双方が悪い区間で経路内に留まる保証はない。実機rosbagによる校正が
  必要である。**
- LIO内部の照合品質、車輪スリップ判定、TF二層化、点群地図のグラフ最適化は実装していない。
- 再取得した距離は新しい安定観測であり、**物理距離が独立に測定・保証されたという意味では
  ない。**
- MotionGuardの閾値（7.4節）は未校正である。
- 共分散は実機で校正済みの誤差上限ではない（4章）。

## 10. 影響範囲・未決事項

### 10.1 影響範囲

既存公開msgと既存ノードの通常起動を変更しない。融合は明示的に選択した場合だけ使用する。
ENU pose追従へ接続し、GPS補正量を制限する。

**map/odom TFの二層化による局所制御の移行は未実装で、TFによる座標ジャンプ隔離が完成した
とは扱わない。**

GNSS simulatorのbaseline誤差の既定値は0で、融合試験時にのみ指定する。

`/localization/pose_enu` の配信元は本ノードのみとする。既存の配信元は停止またはremapする。

### 10.2 未決事項

既知の欠陥は `tests/test_known_limitations.py` が固定する（9.1節）。

#### 10.2.1 GPS方位の鮮度が入力境界で検査されない

`um982/client.py` の `get_position` は最新GGAとキャッシュ済みUNIHEADINGを結合するが、
両者の時刻差を検査せず、出力timestampにGGAの時刻だけを使う。`driver_node.py` の
`_on_position` はその時刻をfix/statusへ共通設定する。**ROS側のstampが一致していても、
位置と方位が同時刻の観測である保証はない。**

人工入力で位置stamp=1000、方位stamp=900を与えると、100秒古い方位90°とσ=1°がFIXの
位置stamp=1000へ結合される。方位文だけが途絶しても位置文が続けばこの状態は検出できない。
旋回時に古い方位で推定を引き戻し、方位σを過小評価し得る。

方位の観測時刻・鮮度・解品質を入力境界で保持し、位置との時間差を検査する必要がある。
GNSS全体を遮断するR1試験ではこの状態を再現できない。方位だけの途絶と旋回中の異時刻を
分けて試験する。

#### 10.2.2 相対運動を採用できない入力から自動復帰しない

`motion_guard_core.py` の `select` は入力ごとの基準を更新した後、採用変位がなければ
融合側stampを更新せずreturnする。次回は入力基準stampと融合stampが一致しないため、
正常な差分も計算対象から外れ続ける。

人工入力で0.2秒に車輪欠測、0.3秒にLIOジャンプと車輪復帰を与えると、0.4〜5.0秒に両入力を
正常化しても `NO_MOTION` が継続し、採用stampは0.2秒のまま止まる。下流は出力鮮度監視で
停止する方向だが、重力整列のリセット等が起きない限り自然復帰しない。

入力を再び差分計算できる基準を確立する処理と、未観測移動の扱いを定める必要がある。
姿勢を架空に更新せずに復帰する試験を追加する。

#### 10.2.3 その他の未決事項

| 件 | 選択肢 | blocker |
| --- | --- | --- |
| 7.3節: LIO異常が検出されないまま45度以上乖離した場合の自動再初期化がない | 乖離の継続で再初期化する／現状どおり手動復帰とする | ではない |
| 3.2節: 異なるstampの別ドライバに未対応 | 位置と方位を別時刻として扱う／UM982前提を明示する（現状） | ではない |
| 10.1節: TF二層化が未実装 | map/odomの二層化で座標ジャンプを隔離する／現状どおりmap姿勢を直接追従に使う | ではない |
