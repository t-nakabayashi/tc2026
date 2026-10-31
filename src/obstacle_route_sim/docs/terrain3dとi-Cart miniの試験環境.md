# terrain3dとi-Cart miniの試験環境

## 構成

tools/terrain3dの地形コンパイラをNode.jsから利用し、scene・メッシュ・SDF・経路を生成する。
Gazebo Harmonicの差動二輪とROSの走行スタックを接続する。
合成通路は実測した現地地形ではない。

build_terrain_trial.pyはstraight、crank、blocker等の試験を生成し、
evaluate_terrain_trial.pyが期限付き起動・計測・終了回収を行う。
preview_terrain_trial.pyは記録した真値軌跡の閲覧HTMLを生成する。
生成物はlog配下などへ保存し、Gitへ追加しない。

## モデルとセンサ

座標はx前・y左・z上、水平原点は車軸中央。エディタの+y前方とは表示時のyawで合わせる。
生成器のi-Cart miniモデルは車輪半径0.07455 m、輪間隔0.30737 m、速度上限0.9 m/sを使う。
基本センサ配置は主GNSS高さ0.7 m・後方副アンテナ間隔0.5 m、URG高さ0.3 m、MID-360高さ0.6 m。
これらは模擬モデル値であり、同梱実機hardware.yamlの車輪・取付値とは別である。
質量・慣性・接地・センサ特性は実搭載状態で校正した値ではない。

基本試験は真値由来pose、--gnssは模擬GNSS由来pose、--fusionはGNSS/LIO融合を使う。
FAST-LIO入力は[専用準備手順](FAST-LIOの模擬入力と確認方法.md)でIMU等を追加する。

## 実行

```bash
npm ci --prefix src/obstacle_route_sim/tools/terrain3d
python3 src/obstacle_route_sim/tools/build_terrain_trial.py --scenario blocker --output <新規試験先>
python3 src/obstacle_route_sim/tools/evaluate_terrain_trial.py --world <試験先> --timeout 100
python3 src/obstacle_route_sim/tools/preview_terrain_trial.py <試験先>
```

ROS環境とoverlay、Gazebo runtimeを有効化する。
不足するGazebo依存はprepare_gazebo_runtime.pyとactivate_gazebo_runtime.bashでローカル展開も可能。
この補助はUbuntu 24.04／Jazzyのapt情報を前提とする。
試験はdomainとGazebo partitionを分離し、同じdomainで重複起動しない。

## 判定と制約

到達はFINISHEDだけでなく、真値の終点距離・停止指令・odom停止・真値移動・必要topicを確認する。
接触監視は対象collisionに限定され、通知0件だけで監視正常とは証明できない。
contact_probeで陽性を確認する。これは仮想環境への指令を送る試験である。

入力途絶は--dropout、GNSSやLIOの品質条件は各オプションで選ぶ。
result.jsonとtrajectory.csv等を確認し、接触・停止・横ずれ・推定誤差を分ける。
ROS時計とwall時間を区別する。模擬ゴール半径は車体全体の公式ゴール通過判定ではない。
HTMLやPCのlocalhostはiPhoneへ自動配信されない。
