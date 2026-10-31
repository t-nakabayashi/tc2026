# FAST-LIO点群地図の保存と評価

## 対象

FAST-LIOが推定姿勢で配置した観測を保存し、生成に用いたSDF表面や航空写真と比較する。
比較先は公開情報から作った試験環境であり、現地測量の独立した正解地図ではない。
同じ形状から観測を作るため、主に再構成と推定の内部整合を評価する。

記録はtools/lio_map_recorder_node.py、集積・評価はtools/evaluate_lio_map.py、
全周の事後診断はtools/review_full_lio_trial.pyが担当する。

## 3. 保存する地図の意味

FAST-LIO が各観測を自身の推定姿勢で camera_init へ変換した /cloud_registered を購読する。
観測はシミュレーション時刻で 1 秒以上離して採取し、各観測内を 0.25 m voxel で間引く。
再開・推定の補正のために真値で点を配置し直す処理は行わない。
受信 QoS は reliable depth 10、点群フレームと採取間隔を manifest.json に保存する。
1 観測ずつ npz を書き、長時間の全点群を RAM に保持しない。

終了後に全観測を同じ 0.25 m voxel で統合して fastlio_map.pcd を出力する。
各 voxel では最初の実観測点を残し、欠けた面を補完しない。
この地図は FAST-LIO 登録済み観測の集積であり、内部 ikd-tree の直接ダンプではない。
ローカル版の /Laser_map も登録済み観測の集積だが、累積表示地図であるため、
全域試験では負荷とメモリを制限する分割保存を使う。

## 4. 座標整合・評価方法

真値時系列と LIO 時系列が重なる初回時刻だけで XY、Z、yaw を整合する。
LIO は IMU 位置なので、真値モデル原点の高さに 0.6 m を加える。
roll/pitch による lever arm の厳密補正は未実施である。
全軌跡の ICP、ループ閉じ込み、GPS による後処理補正で drift を消さない。
評価用整合後の fastlio_map_enu.pcd も保存し、元座標版と区別する。

SDF の静的 visual 面を対象に、OBJ 三角面、円柱、楕円体等を約 0.5 m 間隔で標本化する。
車体は参照面に含めない。GPU LiDAR が描画形状を観測するため collision のみに限定しない。
点群から最寄りの表面標本への距離の中央値、95 percentile、RMSE、0.5 m / 1 m 以内の比率を出す。
これは厳密な point-to-mesh 距離ではなく、表面標本の疎密による正の距離誤差を含む。
標本は面上にあるため、その面への真の最短距離を小さく見積もる手法ではない。
地面点の比率で結果を良く見せないよう、モデル地表より 1 m 高い点を分けても報告する。
推定が大きくずれた点の地物分類は信頼できないため、この区分は補助指標として扱う。

航空写真には高さ着色点群と真値走行軌跡を重ねる。写真は上空視点であり、
地上 LiDAR が見える壁と屋根・樹冠は一致しない場合がある。
未観測の裏面・屋根を欠落として一律不合格にする完全性評価は行わない。
全点群の完全性を保証するためには、可視性・遮蔽を含む追加評価が必要である。

## 5. 実行手順

ROS Jazzy、obstacle_route_sim overlay、Gazebo runtime を有効化して実行する。
DDS domain と GZ_PARTITION は評価ツールが仮想環境用に隔離する。

```bash
python3 src/obstacle_route_sim/tools/prepare_waypoint_evaluation.py --source <調整地形ディレクトリ> --output <全域waypoint出力先> --start 0 --count 1142
python3 src/obstacle_route_sim/tools/prepare_fastlio_trial.py --source <全域waypoint出力先>/fixed --output <全域LIO出力先>
python3 src/obstacle_route_sim/tools/evaluate_terrain_trial.py --world <全域LIO出力先> --gnss --fastlio <fast_lioの実行ファイル> --record-lio-map --timeout 7200 --lio-noise-profile reference
python3 src/obstacle_route_sim/tools/plot_lio_evaluation.py <全域LIO出力先>
python3 src/obstacle_route_sim/tools/evaluate_lio_map.py <全域LIO出力先>
```

起動・走行・終了を期限付きで行い、保存した分割観測を維持する。
途中停止や未到達の場合に全域成功として扱わない。
全区間を初期化なしで追跡できたかは、走行進捗、受信記録、LIO 誤差を併せて判断する。

## 6. 制限と残課題

- 模擬 Mid-360 は 70 m の規則格子点群で、反射・非反復走査・点内時刻を完全再現していない。
- 模擬 IMU のノイズ・取付・時刻は仮定で、実機の校正値ではない。
- URG は高さ 0.3 m の単一走査面で、低い段差・穴・上方張り出しを網羅しない。
- 地形は DEM に基づく概略で、実際の縁石・階段・路面摩擦・通行規則を全て含まない。
- GNSS走行による比較と、--fusionを指定した融合走行を区別する。
- 模擬のROS時間と実行に要したwall時間を分け、所要時間を実機性能値としない。
