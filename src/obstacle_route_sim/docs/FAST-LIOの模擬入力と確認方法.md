# FAST-LIOの模擬入力と確認方法

## 構成

prepare_fastlio_trial.pyが地形をコピーしてIMUとLIO用LiDAR設定を加え、
fastlio_sim.launch.pyがGazebo、bridge、lio_sensor_adapter_node、FAST-LIOを接続する。
LiDAR/IMUから推定し、FAST-LIOへGNSSや真値poseを入力しない。

adapterは整定待ち・非有限点と範囲外点の除去・選択した誤差モデルを適用する。
点群はXYZIの瞬時観測で、点内時刻やMid-360の非反復走査を再現しない。
専用の模擬設定は同位置・同方向のLiDAR/IMU、T=0・R=Iを前提とする。
実機の内部較正・取付値をそのままこのモデルへ適用しない。
Top-URG相当のscanは独立したセンサで、Mid-360の投影ではない。

## 入力準備と起動

```bash
python3 src/obstacle_route_sim/tools/prepare_fastlio_trial.py \
  --source <地形試験ディレクトリ> --output <新規LIO試験ディレクトリ>
ros2 launch obstacle_route_sim fastlio_sim.launch.py \
  world:=<新規LIO試験ディレクトリ>/trial.sdf domain_id:=86 noise_profile:=reference
```

車体・センサ設定は生成SDFとtrial.json、推定設定はparamsの専用FAST-LIO YAMLを参照する。
`noise_profile`とseedは起動時に選ぶ。[誤差モデル](FAST-LIOのセンサ誤差モデル.md)を参照する。

## 走行評価の選択

```bash
python3 src/obstacle_route_sim/tools/evaluate_terrain_trial.py \
  --world <LIO試験ディレクトリ> --gnss --fusion \
  --fastlio <FAST-LIO実行ファイル> --lio-noise-profile conservative --timeout 600
```

`--fusion`はGNSS/LIO融合位置を使う閉ループ走行で、`--gnss`と`--fastlio`を必要とする。
`--fusion`を付けずGNSSとFAST-LIOを併用すると、GNSSで走行してLIOは比較用に計測する。
通常の操作UIを使う場合は[共通起動](../../icart_bringup/README.md)を使用する。
共通起動のFAST-LIO出力は/lio/odometry_raw→重力整列→/lio/odometryである。

plot_lio_evaluation.pyは時刻を合わせた推定比較、plot_urg_evaluation.pyはscanと回避軌跡の確認に使う。
LIOの原点合わせ、センサ原点と車軸の差、入力欠落を明示する。
LIOトピックの配信やGNSS走行の完了だけでは推定精度の合格としない。
