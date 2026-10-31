# FAST-LIOのセンサ誤差モデル

## 1. 対象と仮定

入力点群とIMUへ誤差・欠測を加える。推定poseを加工して目標の誤差へ合わせる処理はない。
field_assumedとconservativeは試験用の仮定で、実機データから校正したモデルや誤差の上限保証ではない。
実機への適用精度とシミュレーション内の比較を区別する。

## 2. 公開仕様の根拠

- [Livox Mid-360 公式仕様](https://www.livoxtech.com/mid-360/specs):
  10 m・反射率 80%・25℃で測距ランダム誤差 1σ≦2 cm、角度 1σ<0.15°。
  検出距離は反射率 10%で40 m、80%で70 m（100 klx）。搭載 IMU 表記は ICM40609。
- [TDK ICM-40609-D 公式仕様](https://www.invensense.tdk.com/en-us/products/6-axis/icm-40609-d):
  gyro 雑音密度 4.5 mdps/√Hz、加速度 100 μg/√Hzを参考にする。
  同系列の公開値であり、Mid-360 内の型番末尾・帯域・フィルタ構成との一致は未確認である。

200 Hz 出力・等価雑音帯域 100 Hz を仮定すると、密度×√帯域は gyro 約0.000785 rad/s、
加速度約0.00981 m/s²となる。これを field_assumed の白色雑音の目安とした。
帯域100 Hzは製品設定の測定値ではない。Gazebo 側の既存雑音にも重ねるので、
合成標準偏差は gyro 約0.000825 rad/s、加速度約0.01020 m/s²になる。
固定 bias、時間変化、欠測率、遠方減衰、時刻誤差は以下に示す試験上の仮定である。

## 3. 条件とモデル

| 項目 | reference | field_assumed（標準） | conservative |
| --- | --- | --- | --- |
| 追加測距σ | 0 | 0.02＋0.0003 max(r−10,0) m | 0.04＋0.0006 max(r−10,0) m |
| 方位・仰角σ | 0 | 各0.10° | 各0.15° |
| 基本欠測率 | 0 | 8% | 20% |
| 遠方減衰の開始／尺度 | なし | 30／25 m | 25／15 m |
| 最大距離 | 70 m | 70 m | 55 m |
| 距離外れ値の確率 | 0 | 0.01% | 0.1% |
| 外れ値の追加距離 | なし | ±1 m 一様分布 | 同左 |
| 追加加速度白色σ | 0 | 0.01 m/s² | 0.02 m/s² |
| 追加角速度白色σ | 0 | 0.0008 rad/s | 0.0016 rad/s |
| 固定加速度 bias の各軸範囲 | 0 | ±0.04 m/s² | ±0.08 m/s² |
| 固定角速度 bias の各軸範囲 | 0 | ±0.0004 rad/s | ±0.001 rad/s |
| 時変加速度 bias の定常σ | 0 | 0.005 m/s² | 0.015 m/s² |
| 時変角速度 bias の定常σ | 0 | 0.0001 rad/s | 0.0003 rad/s |
| bias 相関時定数 | ― | 120 s | 120 s |
| IMU 各軸 scale 誤差の範囲 | 0 | ±0.1% | ±0.3% |
| IMU stamp 固定ずれ | 0 | ＋2 ms | ＋5 ms |
| IMU stamp 一様 jitter 範囲 | 0 | ±0.5 ms | ±1.5 ms |

r は元の距離である。点の保持確率は
(1−基本欠測率)×exp(−(max(r−減衰開始,0)/尺度)²) とし、最大距離の外は削除する。
例えば field_assumed の保持確率は40 mで約78%、70 mで約7%になる。
これは低反射・遮蔽・欠測が混じる観測の便宜的モデルで、各物体の反射率を再現していない。
角度誤差は観測値の方向に与え、変更後の光線を再 ray tracing する処理ではない。
外れ値の確率は保持後点に対する仮定で、製品の全発射に対する false alarm rate と同義ではない。
conservative の外れ値率は通常仕様より高く、悪条件・混合反射を想定したストレス注入である。

時間変化 bias は b次=exp(−dt/τ)b前＋σ√(1−exp(−2dt/τ))N(0,1) の OU 過程を使う。
無制限に増えるランダムウォークにせず、ゆっくり変化する誤差として扱う。
固定 bias と scale は起動時に seed から決め、LiDAR / IMU の乱数系列は分離する。
同じ入力・同じ seed に対する出力は再現できるが、Gazebo / DDS の実行順まで固定するものではない。

IMU covariance には追加白色雑音の分散のみ加算する。未知 bias の真値は推定器に渡さない。
IMU の観測値は元時刻で劣化させた後、stamp をずらす。通信遅延のキュー処理ではない。
200 Hz の通常入力では jitter 差が5 ms未満なので stamp の順序を維持する。
LiDAR の stamp は保持する。点内時刻・走査歪み・非反復走査は再現しない。
動く障害物は共通デジタルツインの歩行者モデルで別に扱う。
この誤差注入はLIO入力が対象で、URG・GNSSの誤差条件は別に設定する。

## 4. 実装・起動

- tools/lio_noise_core.py: ROS 非依存の誤差生成。
- tools/lio_sensor_adapter_node.py: LiDAR / IMU に適用して LIO へ配信。
- tools/evaluate_terrain_trial.py: profile / seed を指定し、sensor_noise.json に係数と実現値を保存。
- tools/review_lio_noise_trials.py: 全 seed の誤差を比較し PNG を出力。
- tests/test_lio_noise.py: 分布・seed・欠測・時刻・相対誤差のテスト。

既存検証補助と同じ tools に配置する。新しい外部依存はない。
fastlio_sim.launchの標準profileはfield_assumed。追加誤差なしの比較にはreferenceを明示する。
共通デジタルツインの既定はconservativeである。
FAST-LIO 本体の推定設定は全条件で同じであり、ノイズ条件ごとに調整していない。

```bash
ros2 launch obstacle_route_sim fastlio_sim.launch.py world:=<IMU追加済みtrial.sdf> noise_profile:=field_assumed noise_seed:=1
python3 src/obstacle_route_sim/tools/evaluate_terrain_trial.py --world <試験フォルダ> --gnss --fastlio <実行ファイル> --lio-noise-profile conservative --lio-noise-seed 1 --timeout 110
python3 src/obstacle_route_sim/tools/review_lio_noise_trials.py <比較試験ルート>
```

profile / seed は起動時に確定する。実行中変更には対応しない。
誤差モデルはセンサtopic・frame・整定待機と独立に選択する。
真値の軌跡や目標誤差を参照して誤差量を操作しない。


## 5. 比較方法

同じ地形・走行方式でprofileとseedを変え、入力件数・欠落・推定誤差・走行結果を比較する。
sensor_noise.jsonに使用係数と実現したbias等を残す。
review_lio_noise_trials.pyで条件間、review_full_lio_trial.pyで全経路と相対誤差を確認する。

初回原点合わせと全軌跡の剛体整合を分け、後者だけで絶対位置が正しいとは判断しない。
縮尺変更や鏡映を使わず、5秒相対誤差も併記して内部変形と固定座標ずれを区別する。
模擬走行の完了とLIOの精度は別の判定である。
