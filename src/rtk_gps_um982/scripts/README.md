# scripts/ — system時刻同期サンプル

`chrony-gpsd.conf.sample`は手動設定例であり、colconによるシステムへの自動導入は行わない。
全体構成は[同期提案](../../../docs/時刻同期設計.md)を参照する。

## 入力の共有

GNSS driverとgpsdで同じttyを同時に開かない。別COMを用いるか、単一の受信処理が
NMEAを複製する仕組みを追加する。既存driverにはgpsd/chrony向け配信機能がない。
単なるsocatの直列接続だけでは複数の読み手への複製は成立しない。
NTRIP送信や設定コマンドの書き込みも、シリアル所有者に集約する。

## 適用と確認

1. chrony/gpsdを導入し、専用ポートまたは実装済みの分配先を決める。
2. gpsdの入力デバイスとSHM出力を確認する。
3. サンプルを参考にchronyのrefclockを設定する。遅延値は実測して調整する。
4. `chronyc sources -v`の選択済み`*`、`chronyc tracking`のoffsetとLeap statusを確認する。
   `#? GPS`は同期成功を意味しない。
5. 時計のstepをROS起動前に完了し、起動後もoffsetを監視する。

NMEAのみの精度はUSB・通信負荷等に依存する。必要精度に達しなければ対応PPS入力を追加する。
同一ttyの競合を避けるためだけに運用中の時刻同期を停止する構成は採用しない。
