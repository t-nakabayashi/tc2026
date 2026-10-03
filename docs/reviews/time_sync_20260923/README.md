# GNSS・PC・MID360時刻同期確認（2026-09-23）

## 結論

通常起動の同期待ち・監視を実装し、実機でGNSS→chrony→PC→PTP→MID360の動作を確認した。
ただし、初回60秒測定で点群時刻が2回逆行したため、連続運用の安定性は未認定。
次の60秒では逆行もゲート失効もなかったが、初回失敗を取り消す根拠にはしない。
GNSS絶対時刻精度も未認定で、precision_verified=falseを維持する。
車輪ドライバ・自己位置推定・走行は起動していない。

## 測定

MID360生UDPのtime_typeとタイムスタンプを読み、SO_TIMESTAMPNSのカーネル受信時刻と比較した。
この差にはLiDARのパケット生成・伝送遅延を含み、時計偏差そのものとは区別する。

| 測定 | 点群 | IMU | ゲート成立 |
|---|---|---|---|
| 初回60秒 | 125,044パケット、全てPTP、逆行2回 | 12,000パケット、全てPTP、逆行0回 | 46/60標本 |
| 追加60秒 | 125,034パケット、全てPTP、逆行0回 | 11,999パケット、全てPTP、逆行0回 | 60/60標本 |

追加測定の受信時刻差99%点は点群1.812 ms、IMU0.617 ms、最大はそれぞれ2.562 ms、0.875 ms。
最大タイムスタンプ間隔は点群0.667 ms、IMU5.965 ms。UDP連番欠落の検査は行っていない。
初回逆行の大きさ・順序入替かLiDAR時計補正かは未確定。追加測定はヘッダ記録を追加したが再発しなかった。
初回もchrony選択とPTPプロセスは継続し、パケット側の異常でゲートが閉じた。

GNSS入力を停止する故障注入では約7.0秒でGNSS条件が失効し、PTP停止・ready=false・
clock_gate watchの異常終了を確認した。これは同期喪失の監視試験であり、実車の制動時間試験ではない。

## 精度の評価と残課題

UM98のchrony残補正量はサブmsになるが、USB RMC受信遅延を含む基準への収束であり、絶対精度の証明ではない。
独立NTP源との差はおよそ30〜40 ms。比較相手の推定誤差は小さい例で約7 msで、真の偏差を確定する基準ではない。
設定したRMC precisionは0.1秒、chrony root dispersionも約0.10秒である。
仮に30〜40 msの測定時刻差があれば1 m/sで3〜4 cm相当となる。
暫定10 ms級のGNSS/LiDAR絶対時刻整合を満たしたとは判定しない。

今後は長時間の生パケット・PTP/chrony状態の同時記録で逆行要因を切り分け、
独立基準によるUSB RMC遅延評価・適切な校正を行う。根拠なく固定補正値を入れない。
初回失敗を隠すために逆行ゲートを緩和しない。

## 適用した修正

- chronyの起動時ACL資格情報とRuntimeDirectoryModeを修正し、一般ユーザーのSOCK入力を維持。
- systemdのicart-clockサービスがGNSS時刻源を監視し、成立時のみsoftware PTPを配信。
- linuxptpのptp_minor_version=0を明示。RO管理ソケットを書込可能な/run/icart-clockへ配置。
- UFWはenp0s31f6・192.168.1.201からのUDP 319/320だけ許可。
- chrony filter=4で正常にもLastRx=6秒となるため、鮮度上限を8秒へ修正。
- 共通実機起動はGNSS/MID360を先に起動し、同期成立後に走行・位置推定系を起動。
- 運用中の同期条件喪失は共通launchをshutdown。復帰後の走行再開は利用者による再起動。

対象はhardware_configを持つ共通bringup経路。旧個別launchは対象外。
共通launchの実機全体起動や車輪の停止動作は今回は試していない。
自動試験91件成功、対象2パッケージのcolconビルド成功。

## 証跡・参照

同じディレクトリのJSONとchronyテキストに測定結果を保存した。測定用Pythonは当日のNIC/IPに固定した再現資料。
詳細サービス設定と手順は[PTP試験手順](../../../src/rtk_gps_um982/docs/PTP試験手順.md)、
起動連接は[共通起動設計](../../../src/icart_bringup/docs/共通起動設計.md)を参照。

- [LinuxPTP ptp4l公式](https://www.linuxptp.org/documentation/ptp4l/)
- [Livox 時刻同期公式](https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/common/time_sync.html)

## 外部NTP基準との追加比較

ユーザー指定の外部NTP→PC→PTP構成を実機で測定した。[追加評価結果](ntp/README.md)を参照。
現在の適用設定は外部NTP基準。上記のGNSS基準測定とは分けて比較する。
