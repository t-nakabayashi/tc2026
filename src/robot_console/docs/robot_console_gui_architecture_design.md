# robot_console アーキテクチャ詳細設計書

## 構成と責務

正式入口は `ui_qt_main.py` のrobot_console_qtである。
Qt表示・ROS通信・状態集約・HTTP配信を分離し、画面からROSメッセージを直接組み立てない。

| 配置 | 責務 |
| --- | --- |
| ros/ | RobotConsoleNode、購読値のCoreへの変換とコマンド配信 |
| core/console_core.py | 状態・操作・起動管理の統合、Snapshot生成 |
| core/camera_overlay.py | 生画像への信号・道路封鎖の枠重畳、保持期限、判定チップ生成 |
| core/snapshot_model.py | 画面・Web共通の表示データ |
| core/localization_adapter.py、route_adapter.py、drive_mode_adapter.py | 位置・経路・走行状態の表示モデル |
| core/freshness.py、operation_phase.py、event_builder.py | 鮮度・運行フェーズ・イベントの導出 |
| core/launch_manager.py、launch_profile.py、log_manager.py | profile起動・停止・ログ |
| core/bag_recorder.py | bag記録プロセスと保存状態 |
| ui_qt/ | MainWindow、7タブ、各カード |
| web/ | 読取専用HTTP、JSON整形、ブラウザ表示 |

## データの流れ

ROSコールバックは観測と受信時刻をCoreへ渡し、GUIはタイマーでSnapshotを取得する。
Qtのwidget更新はUI側で行う。WebもSnapshotとImageStoreを参照し、独自のROS購読を持たない。
コマンドはUI→ConsoleCore→ROS publisher、起動操作はCore→NodeLaunchManagerへ渡す。

自己位置の地図表示は `/localization/pose_llh`、経路は `/active_route` のLLH、
目標表示は `/route/active_target_llh` を使う。
ENU/LLHの変換はgeo_pose_converterに集約し、画面で独自の原点を設定しない。
GNSS単独品質と融合のyaw・σ・modeを別データとして扱う。

主な購読はroute_state、manager_status、follower_state、active_route、active_target、
pose_enu/pose_llh、cmd_vel、drive_mode_status、車輪odom、RTK/NTRIP診断、融合診断、画像、採取状態。
カメラ入力は `/usb_cam/image_raw`、認識結果は `/perception/*/overlay`。
ConsoleCoreで重畳し、QtとHTMLは同じ画像とperception_decisionsを表示する。
画像・overlayの購読はBEST_EFFORT、active_routeはRELIABLE / TRANSIENT_LOCAL / depth 1。
RTK/NTRIP診断は相対名 `rtk_gps/rtk_status` と `rtk_gps/ntrip_status` で購読し、
実機・模擬・単独起動・共通起動とも `/rtk_gps/` 配下の公開名を使用する。
トピック名はlaunch remapとROS側設定に従う。
active_route等の保持される情報と、速度・センサ等の鮮度が必要な情報を区別する。

## 起動管理

profileの定義は `config/node_launch_profiles.yaml` とCoreの起動モデルで管理する。
選択済みprofile、設定ファイル、launch引数から起動コマンドを構成する。
実機手動のreal_survey、記録ルート自律のrecorded_route／icp_routeはicart_bringupへ委譲する。
実機手動の既定は `localization_mode=icp`、実機（融合）の手動は `gnss` を明示する。
プリセットの引数変更もConsoleCoreへ通知し、表示中の方式と実際の起動引数を一致させる。
同じドライバ・走行スタックの重複起動を拒否する。
環境選択だけでは稼働中プロセスの環境を切り替えない。全停止後に設定して再起動する。

stdout/stderrをログへ集約し、終了状態をSnapshotへ反映する。
プロセス停止はSIGINTから段階的に行う。単なる起動グループのRUN表示は全センサ正常を意味しない。

## 表示と監視

[画面仕様](robot_console_gui_screen_function_design.md)の7タブを使う。
入力途絶はfreshnessとして表示し、未受信値をゼロや正常値に置換しない。
フォロワRUNNING/AVOIDINGを新鮮に受信していれば、開始操作を観測し損ねても運行表示に反映する。
開始指令を推測して再送しない。

R1途絶模擬は `/fusion/gnss_dropout_active` を表示する。
時刻同期警告は `core/clock_sync_status.py` が `/run/icart-clock/status.json` を読み、
同じboot ID・3秒以内・clock_source=ntp・readyの条件を確認する。
後者はROS起動前にも表示でき、実機モードの全タブで1秒周期に更新する。
停止の実行はicart_bringupの同期ゲートが担当する。

## オフライン地図作成

MapCreationTabはicart_mappingで入力を検査し、QProcessで専用ジョブを起動する。
ジョブはbubblewrap隔離内でのみ処理し、ネットワークと実機デバイスに接続しない。
段階状態はstatus.json、詳細はbuild.logへ保存する。Qtは状態とログを非同期に読み、
処理中も中止できる。GUI終了時はジョブを終了する。
完成状態と地図ハッシュを確認できた結果だけ、利用者操作でLaunchSettingsTabへ渡す。
この操作は次回のICP走行・ルート記録の地図引数を更新し、ConsoleCoreの同じoverride更新経路へ通知する。
走行指令・稼働中ノード・現在の経路は変更しない。

## 遠隔観測と記録

HTMLは読取専用で、走行指令・起動停止・bag操作を提供しない。
サーバはloopbackで待ち受け、認証機能を持たない。
別端末への公開は[HTML遠隔観測UIの公開範囲](html_ui_access.md)に従う。
画像参照はImageStore、JSON変換はweb/json_codec.pyで扱う。

[ROSBAG保存](rosbag_ui.md)はPCの記録操作、[経路採取](../../route_survey/README.md)は
共通自己位置（ICP／GNSS融合）と経路属性の保存であり、出力形式と用途が異なる。

## 確認方法

`tools/tests/` で観測の集約・鮮度・操作接続・起動失敗・ログ・JSON・Qt表示を確認する。
Qt試験はpytest-forkedで分離する。ROS結合確認では隔離domainを使い、
未受信・途絶・不正値を含めて確認する。画面表示試験は実機運動の検証を代替しない。
