# 地理ウェイポイント・FLOAT・FAST-LIOの比較方法

prepare_waypoint_evaluation.pyで同じ元地形から経路追従・FLOAT・障害物条件の試験を準備する。
LLH CSVはroute_plannerが共通projectionからENUへ変換する。

```bash
python3 src/obstacle_route_sim/tools/prepare_waypoint_evaluation.py --source <元地形> --output <新規試験先>
python3 src/obstacle_route_sim/tools/evaluate_terrain_trial.py --world <試験先>/fixed --gnss --timeout 120
python3 src/obstacle_route_sim/tools/evaluate_terrain_trial.py --world <試験先>/float --gnss --building-gnss --timeout 120
```

FAST-LIOを比較するにはprepare_fastlio_trial.pyでIMU付き地形を作り、--fastlioを指定する。
GNSS/LIO融合で走行する場合はさらに--fusionを指定する。
GNSSで制御してLIOを記録する比較と、融合による閉ループ走行を区別する。

建物近傍のFLOATは誤差仮説であり、実受信機のmultipathを再現した保証はない。
FINISHED、真値の終点、経路横ずれ、障害物回避・接触、各推定の誤差を別に評価する。
[FAST-LIO接続](FAST-LIOの模擬入力と確認方法.md)、
[誤差モデル](FAST-LIOのセンサ誤差モデル.md)、[融合確認](../../gnss_lio_fusion/docs/実装評価.md)を参照する。

模擬GNSSは `/rtk_gps/ntrip_status` に1 Hzで `state=DISABLED` の診断JSONを配信する。
NTRIP接続を模擬せず、基地局UIで構成上の不使用と診断途絶を区別する。
