# GNSS/LIO 融合

FIX/FLOAT・衛星数・適応baselineでGNSSの重みを決め、FAST-LIOの相対移動と平面EKFで融合する。
FIX/FLOAT 以外の状態では位置・方位を補正しない。方位精度値が残っていても無効観測として扱う。
低信頼時はLIO優先で減速して継続する。実機の校正・完走保証を意味するものではない。

[詳細設計書](docs/gnss_lio_fusion_詳細設計書.md)に設定、QoS、座標、時刻、異常系、残課題を記載する。
[実装評価](docs/実装評価.md)に評価方法と判定範囲を記載する。

```bash
colcon build --symlink-install --packages-select gnss_lio_fusion
ros2 launch gnss_lio_fusion fusion.launch.py projection_params:=<ENU原点YAML>
```

既存の/localization/pose_enu配信元は停止またはremapして、融合出力と競合させない。
FAST-LIOのOdometryを/lio/odometry_rawへremapし、水平化後の/lio/odometryを購読する。GPSのfix/statusは同じ観測時刻で配信する。
自律muxの入力cmd_vel/autonomousを/cmd_vel/fusion_limitedへremapすると減速が有効になる。
融合launchは実機ドライバ・自律モードを起動しない。実機確認は別途行う。

検証はobstacle_route_simのevaluate_terrain_trial.pyに--gnss --fusion --fastlioを指定する。
--lio-noise-profile conservative --building-gnssで厳しめLiDAR/IMUと建物近傍FLOATを併用する。
review_fusion_trial.pyは同一融合走行のGPS/LIO観測比較と経路横ずれを出力する。


方位更新は位置観測と分離し、角度の折返し・反転・LIO異常後の再取得を扱う。
[方位実装評価](docs/方位実装評価.md)に方位の品質判定と確認方法を記載する。
実機と模擬環境の共通起動は[icart_bringup](../icart_bringup/README.md)を使う。
前master・後slaveのUM982生方位は後方を向くため、共通起動が180度補正する。
単体launchを使う場合は、受信機側の補正有無と取付方向に合わせて明示設定する。

車輪odomはLIO異常・不整合・途絶時の退避に使用する。通常はLIOを優先する。
退避中もURG停止は維持し、GPS_WHEEL/WHEEL_PRIORITYと退避回数を診断に出す。
車輪スリップを見抜けるという意味ではない。

## 傾斜したMID-360と前後アンテナ配置

`lio_mount_roll_deg/pitch_deg/yaw_deg`はIMUの車体に対する取付角、
`lio_forward_m/left_m/height_m`は車体からIMU原点への位置とする。
LIOから車体の3D姿勢を求め、レバーアームを除いた平面位置を融合する。
`master_forward_m/left_m/height_m`で主アンテナ位置を補正する。
`publish_base_tf`（既定false）をtrueにするとmap→base_link平面TFも配信する。
実機設定はicart_bringupのprepare_real_sessionが一括生成する。
LiDAR内部extrinsicに車体への取付角を足さない。

### R1によるGNSS途絶模擬

R1（SDLボタン5）を押している間だけ、融合ノードへのGNSS位置・品質内の方位入力を破棄する。
離すと即座に解除し、Joy受信が0.5秒途絶えた場合も解除する（判定は単調時計）。
受信機/NTRIP通信・生GNSSトピック・その記録は維持する。LIO/車輪入力も継続し、
融合は通常のGNSS期限切れ・復帰判定を使う。解除はRTK FIXや融合採用の即時回復を保証しない。
切替時は未処理GNSSと品質キャッシュを捨て、解除後の新規観測から再開する。
R1の既定ターボ機能は解除済み。カスタム設定でturbo_button=5を指定しないこと。
`/fusion/gnss_dropout_active`を10Hzおよびボタン更新時に配信し、ROSBAGにも記録する。
PCダッシュボード最上部・Webメイン画面に模擬中は赤、通知期限切れは黄の警告を表示する。
正常解除後は警告を隠す。通常のGNSS受信表示とは別に模擬中であることを示す。

### 重力方向をそろえた共通LIO出力

`icart_bringup` と `fusion.launch.py` は `gravity_alignment_node` を起動する。
FAST-LIOの `/Odometry` は `/lio/odometry_raw` に接続する。水平化後の
`/lio/odometry` (`lio_level` → IMU child) を融合とルート記録が共用する。
単独launchで外部FAST-LIOを使う場合も、このrawトピックへのremapが必要。

初期化済みLIO姿勢と測定時刻の近いIMU、停止中の車輪を照合し、約2秒の
連続静止・重力方向の安定を確認する。推定した固定回転を3D位置・姿勢・
pose共分散へ適用した後、既存の取付角/レバーアーム補正を行う。
走行中やGNSS OFF/ONで鉛直基準を更新しない。IMUに固定されたbody点群と
child座標のtwistは回さない。ルート記録は水平化後の車体姿勢でbody点群を
地図へ配置する。`/cloud_registered` はraw worldのままなので、水平化した
地図と混同しない。

`/lio/alignment_status` に理由と固定回転、`/lio/alignment_ready` に生存状態を
配信する。初期化待ちはGUIに「水平基準待ち（約2秒静止）」と表示し、自律指令を0に
する（手動移動は可能だが、基準確定には停止が必要）。LIO publisherの再起動、
時刻逆行、frame変更、1.5秒超のLIO断で基準を破棄する。融合は準備状態の解除や
heartbeat途絶で履歴を捨て、新しい基準とFIXを待つ。古いルートの数値は自動修正しない。
重力整列は時計同期とは別の処理である。
