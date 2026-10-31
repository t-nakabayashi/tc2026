# 地理地図と模擬GNSS

## 構成

build_geographic_trial.pyは公開地理情報を使う試験地形を生成する。
模擬GNSSはGazebo真値からNavSatFix・heading・RtkStatusを作り、測位状態・誤差・欠測を与える。
geo_pose_converterで共通原点のENUへ変換する。
GNSS単独とGNSS/LIO融合は評価オプションで選ぶ。

## 座標と時刻

同じprojectionを経路と自己位置へ渡し、headingの北時計回りとmap yawを区別する。
アンテナ位置と車軸位置を精度比較で混同しない。
模擬ではROS時刻を使い、物理計算のpauseや低速化をwall時間で欠測と判定しない。

## 確認方法

生成した試験ディレクトリをevaluate_terrain_trial.pyの--worldへ渡し、--gnssを指定する。
--building-gnssで建物に応じたFLOAT、--dropoutで欠測を扱う。
融合には--fusionと--fastlioを併用する。
生成と実行の引数は各ツールの--help、
[融合評価](../../gnss_lio_fusion/docs/実装評価.md)を参照する。

真値と時刻を合わせて誤差を測り、RTK状態・衛星数・方位・遅延・欠測を併記する。
故障を注入しない試験で、故障停止の判定値を合否に使わない。
DEM・建物データは縁石・路面・通行帯を網羅せず、GNSS誤差も実機受信機の校正モデルではない。

模擬GNSSは `/rtk_gps/ntrip_status` に1 Hzで `state=DISABLED` の診断JSONを配信する。
NTRIP接続を模擬せず、基地局UIで構成上の不使用と診断途絶を区別する。
