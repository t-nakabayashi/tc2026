# robot_console GUI改修 画面・機能詳細設計書

## 1. 文書目的・対象範囲

本書は、次期 `robot_console` GUIの正式画面仕様を定義する。対象は、PyQt5ローカル操作UIとHTML遠隔観測UIである。

**実装状態:** 本書が定義するPyQt5の4タブ画面とHTML遠隔観測UIは実装済みであり、正式UIである。未実装の項目は 1.1節「未実装項目」に列挙する。

ROS 2ノード、Core、Snapshot、HTML観測UIとの構造的な役割分担は `robot_console_gui_architecture_design.md` を正とする。本書では、実際の業務フローから各画面の役割、想定操作、レイアウト、表示内容、入力方式を具体化する。

### 1.1 未実装項目

以下は本書に仕様があるが未実装である。実装時は該当節を正とする。

| 項目 | 該当節 | 現状 |
| --- | --- | --- |
| 信号停止・停止線の区別表示（`signal_stop_active` / `line_stop_active`） | 6.7 | `FollowerState` に該当フィールドが無く、`Route` のwaypointフラグ（`signal_stop` / `line_stop`）と follower の `WAITING_STOP` から導出する必要がある。現在はどちらも既定値のまま。 |
| 目標到達判定（`arrival_threshold_m` / `within_arrival_threshold`） | 6.4 | 到達判定しきい値を配信するtopicが無く（`route_follower` のパラメータ）、目標距離は常に「未到達」と表示される。 |
| Node Healthチップのログレベル表示（`last_log_level`） | 6.8 | `LogManager` のWARN/ERROR抽出結果をSnapshotへ載せていない。 |
| 起動予定に含まれない重要profileの警告（`required_but_not_selected`） | 6.8 | 常に `False`。重要基盤profileの定義が未確定。 |
| `/mission_info`・`/rtk_gps/fix`・`/rtk_gps/heading` の購読 | 7章 | 未購読。GPSカードは `/rtk_gps/.../rtk_status` のみで表示している。 |

## 2. 基本方針

次期UIは以下の4タブ構成とする。

```text
1. ダッシュボード
2. 自己位置・センサ情報
3. 起動・設定
4. コンソールログ
```

全タブ共通の上部ステータスバーは設けない。運行状態の詳細確認と走行中操作はダッシュボードに集約する。自己位置・センサ情報タブには運行状態のサマリだけを表示し、起動・設定タブとコンソールログタブには常時ステータス確認領域を置かない。

走行中のローカルGUIは、原則としてダッシュボードタブを表示し続ける。自己位置やセンサの詳細確認はHTML UIを別ブラウザで開く、または一時的に自己位置・センサ情報タブへ切り替えて行う。

### 2.1 画面サイズ・スケーリング方針

各タブの設計は、1280x720に内容を詰め込むことを目的にしない。GUI全体は16:9の論理キャンバスとして設計し、スクロールなしで全体が表示されることを必須とする。ウィンドウサイズが変わった場合は、レイアウト全体を16:9のアスペクト比を保ったまま拡大・縮小する。

実装方針:

- `MainWindow` 内に16:9のルートコンテナを置く。
- ルートコンテナは利用可能領域に対して `min(width / 16, height / 9)` のスケールで拡縮する。
- 余白が出る場合は左右または上下にletterbox余白を置き、UI要素の比率は変えない。
- タブ内に縦スクロールや横スクロールを置かない。情報量が多い場合は、折りたたみ、優先度表示、別タブ遷移、HTML UIへの分担で解決する。
- フォント、アイコン、カード、余白も同じスケール係数で拡縮し、文字だけが先に大きくなってレイアウトを崩さないようにする。
- 16:9を保つ対象はタブ内容領域だけではなく、GUIの主要レイアウト全体である。OSウィンドウ枠やメニューバーを除いたアプリ内コンテンツ領域で16:9を維持する。

基準解像度は実装時に決めてよいが、設計上は1920x1080相当の情報量を想定する。1280x720は縮小表示の確認対象であり、情報量を制限する上限ではない。

## 3. 業務フロー

### 3.1 業務分類

`robot_console` を使う業務は、実行環境と走行モードで4種類に分かれる。

| 業務 | 実行環境 | 走行モード | 主な起動対象 |
| --- | --- | --- | --- |
| 実機手動走行 | 実機 | 手動 | `ypspur_ros2`, `drive_mode_manager`, 必要に応じて `rtk_gps_um982` |
| 実機自律走行 | 実機 | 自律 + 手動介入 | `rtk_gps_um982`, `ypspur_ros2`, route stack, drive stack, obstacle/perception stack |
| シミュレーション手動走行 | Gazebo等 | 手動 | `obstacle_route_sim`, `drive_mode_manager` |
| シミュレーション自律走行 | Gazebo等 | 自律 | `obstacle_route_sim`, route stack, drive stack, obstacle stack |
| 机上確認（手動） | 机上（実センサ・Gazebo無し） | 手動 | `drive_mode_manager`（`joy_input=ps3_joy_sim`）、必要に応じ `robot_navigator`（simulator代替） |
| 机上確認（自律） | 机上（実センサ・Gazebo無し） | 自律 | route stack, drive stack, `robot_navigator`/`obstacle_monitor`/`road_blockage_detector`/`traffic_signal_recognizer`（いずれもsimulator代替使用） |

### 3.2 共通業務フロー

1. `robot_console` を起動する。
2. 起動・設定タブで業務モードを選択する。
3. 起動対象ノードを選択する。
4. 各ノードのconfigファイル、launch引数、上書きパラメータを指定する。
5. 選択したノードを一斉起動、または必要なノードだけ個別起動する。
6. コンソールログタブで起動ログを確認する。
7. ダッシュボードタブでノード状態、GPS、走行モード、route状態、障害物、信号、速度指令が正常であることを確認する。
8. 自律走行の場合、ダッシュボードから `/manual_start` を送信して走行を開始する。
9. 走行中は原則としてダッシュボードタブを維持する。
10. 自己位置やセンサ詳細が必要な場合、HTML UIをブラウザで確認する。ローカルGUIで見る場合は自己位置・センサ情報タブへ一時的に切り替える。
11. 一時停止からの復帰、信号認識代替入力、道路封鎖、障害物hint overrideなどの介入はダッシュボードから行う。
12. 終了時は起動・設定タブまたはダッシュボードの運用終了導線から選択ノードを停止する。

### 3.3 実機手動走行フロー

1. 起動・設定タブで業務モード `実機 / 手動走行` を選択する。
2. `ypspur_ros2` と `drive_mode_manager` を選択する。
3. `ypspur_ros2` の `cmd_vel_topic` は `/cmd_vel` を既定とする。
4. `drive_mode_manager` の `joy_input` は実controller使用時 `joy_node`、開発時 `ps3_joy_sim` から選ぶ。
5. 必要に応じて `rtk_gps_um982` を選択し、GPS受信状態をダッシュボードで確認する。
6. 選択ノードを起動し、コンソールログタブで起動成功を確認する。
7. ダッシュボードでdrive mode、Joy入力、最終 `/cmd_vel`、ypspur odomが正常であることを確認する。
8. 手動走行中はダッシュボードで速度、drive mode、非常系状態を監視する。

### 3.4 実機自律走行フロー

1. 起動・設定タブで業務モード `実機 / 自律走行` を選択する。
2. `rtk_gps_um982`、`ypspur_ros2`、`drive_mode_manager`、route stack、`robot_navigator`、必要なobstacle/perception stackを選択する。
3. GPS config、route config、start/goal/checkpoint、`robot_navigator` の `cmd_vel_topic=/cmd_vel/autonomous`、`odom_topic=/ypspur_ros/odom` を確認する。
4. 選択ノードを一斉起動する。安全上必要な場合は実機基盤を先に起動し、route/perception stackを後から起動する。
5. コンソールログタブで各ノードの起動ログを確認する。
6. ダッシュボードでGPSがRTK_FIXまたは運用許容状態、drive modeが自律準備状態、routeが有効、障害物・信号・道路封鎖状態が正常であることを確認する。
7. `/manual_start` を送信して自律走行を開始する。
8. 走行中はダッシュボードを維持し、一時停止復帰、信号GO/STOP代替入力、road_blocked、obstacle hint overrideを必要時だけ操作する。
9. 自己位置・センサ詳細はHTML UIで確認する。

### 3.5 シミュレーション自律走行フロー

1. 起動・設定タブで業務モード `シミュレーション / 自律走行` を選択する。
2. `obstacle_route_sim`、route stack、`drive_mode_manager`、`robot_navigator`、必要な `obstacle_monitor` を選択する。
3. `obstacle_route_sim` の `road_type`、`road_width`、`enable_pylons`、`pylon_seed`、`start_gazebo_gui` を選択式で指定する。
4. route config、start/goal/checkpoint、`robot_navigator` のtopic設定を確認する。
5. 選択ノードを起動し、コンソールログタブでGazebo bridge、fake localization pose、route stackの起動を確認する。
6. ダッシュボードでシミュレーションpose、route、cmd_vel、obstacle状態を確認する。
7. `/manual_start` を送信して自律走行を開始する。

### 3.6 シミュレーション手動走行フロー

1. 起動・設定タブで業務モード `シミュレーション / 手動走行` を選択する。
2. `obstacle_route_sim` と `drive_mode_manager` を選択する。
3. `obstacle_route_sim` の `road_type`、`road_width`、`start_gazebo_gui` を選択式で指定する。
4. `drive_mode_manager` の `joy_input` は、開発用入力として `ps3_joy_sim` を既定候補にする。
5. 選択ノードを起動し、コンソールログタブでGazebo、bridge、drive mode関連ノードの起動を確認する。
6. ダッシュボードでdrive mode、Joy入力、最終 `/cmd_vel`、シミュレーションodomが正常であることを確認する。
7. 手動走行中はダッシュボードで速度、drive mode、シミュレーションpose、非常系状態を監視する。

### 3.7 机上確認フロー

机上確認は、実機（GPS/ypspur）もGazebo（`obstacle_route_sim`）も使わず、各profileのsimulator代替launchが生成する疑似データ（自己位置、LaserScan、カメラ画像）だけで、route/drive/obstacle/perceptionの各ノードを単体または結合で確認するための業務である。開発時の単体テスト、変更後の素早い動作確認に用いる。

1. 起動・設定タブで業務モード `机上確認 / 手動走行` または `机上確認 / 自律走行` を選択する。
2. プリセット適用により、`ypspur_ros2` / `rtk_gps_um982` / `obstacle_route_sim` を含まない起動予定ノード一覧が生成される。
3. 確認したい対象profile（`robot_navigator`、`obstacle_monitor`、`road_blockage_detector`、`traffic_signal_recognizer` 等）で「Simulator代替を使用」をONにする。
4. `drive_mode_manager` の `joy_input` は `ps3_joy_sim` を選択する。
5. 選択ノードを起動し、コンソールログタブで各simulator代替launchの起動を確認する。
6. ダッシュボードで疑似pose、疑似scan、疑似カメラ画像を入力としたroute/drive/obstacle/perceptionの状態が正常であることを確認する。GPSカードは `GPS N/A`（灰）表示になる。
7. 自律走行の場合は `/manual_start` を送信して確認する。手動走行の場合はJoy入力からのcmd_vel伝播を確認する。

## 4. 起動・設定タブ

### 4.1 役割

起動・設定タブは、業務開始時に最初に使う画面である。起動対象ノードの選択、configファイル選択、launch引数、上書きパラメータ、起動順序、実効設定確認を行う。

このタブは走行中に頻繁に触る画面ではない。走行中の介入操作はダッシュボードへ集約する。

### 4.2 レイアウト

```text
┌────────────────────────────────────────────────────────────────────┐
│ 業務モード選択  プリセット適用  起動予定ノードを一斉起動  起動予定ノードを停止   │
├──────────────────┬──────────────────────┬────────────────────────┤
│ 起動候補ツリー    │ 起動予定ノード一覧         │ ノード設定編集パネル     │
│                  │ 起動順序              │                        │
├──────────────────┴──────────────────────┴────────────────────────┤
│ 起動内容プレビュー（起動予定ノード一覧で選んだ1ノード分のみ）            │
└────────────────────────────────────────────────────────────────────┘
```

左ペインはカテゴリ別の起動候補ツリー、中央ペインは今回起動する予定のノード一覧、右ペインは起動予定ノード一覧で選んだ1ノードの設定編集パネルを表示する。下部には同じ1ノードの起動内容プレビューを表示する。

起動候補ツリーだけでは「これから何を起動するのか」の全体像を把握しづらいため、起動予定ノード一覧を常時表示する。ユーザーは、業務モードからプリセットを適用した後、この一覧を見ながら不要なノードを外し、必要なノードを追加してから一斉起動する。起動候補ツリーは候補カタログ、起動予定ノード一覧は今回起動する確定前リストとして扱う。

実装上は、以下の責務を混同しない。

| 領域 | 責務 | やってはいけないこと |
| --- | --- | --- |
| 起動候補ツリー | 起動可能なprofile候補をカテゴリ別に提示する | ここだけを一斉起動対象の正本にしない |
| 起動予定ノード一覧 | 今回一斉起動するprofile集合と順序を保持する | 詳細な全パラメータ編集UIを詰め込まない |
| ノード設定編集パネル | 一覧で選んだ1profileの設定を編集する | 複数profileの設定を同時に表示しない |
| 起動内容プレビュー | 選んだ1profileの最終起動内容を確認する | 編集可能欄にしない、全profileを並べない |

### 4.3 業務モード選択とプリセット適用

業務モード選択は、手作業で毎回ノードを選ぶ負担を減らすためのプリセット選択である。業務モードを選んだだけではノードを起動しない。ユーザーが `プリセット適用` を押すと、該当業務に必要な候補ノード、既定config、既定launch引数、推奨起動順序が起動予定ノード一覧へ反映される。

業務モードは2段階の選択式とする。

| 項目 | 選択肢 |
| --- | --- |
| 実行環境 | `実機`, `シミュレーション`, `机上確認` |
| 走行モード | `手動走行`, `自律走行` |

`机上確認`は、実機基盤（`ypspur_ros2`）・GPS（`rtk_gps_um982`）・Gazebo（`obstacle_route_sim`）のいずれも使わず、各profileのsimulator代替launchで疑似データ確認を行う実行環境である（3.7節参照）。

プリセット適用後の挙動:

1. 既存の起動予定ノード一覧を、ユーザー確認ダイアログのうえでプリセット内容に置き換える。
2. 起動候補ツリーのチェック状態をプリセット内容に合わせる。
3. 起動予定ノード一覧にprofile、起動順序、config、主要override、状態を表示する。
4. ユーザーはその場の状況に応じて、チェックボックスや追加/除外ボタンで起動対象を取捨選択する。
5. `起動予定ノードを一斉起動` を押した時点で、一覧に残っているノードだけを起動順序に従って起動する。

プリセットは「推奨候補」であり、強制ではない。例えば実機自律走行でも、認識系を使わない試験では `road_blockage_detector` や `traffic_signal_recognizer` を除外できる。シミュレーション自律走行でも、障害物確認をしない場合は `obstacle_monitor` を外せる。

プリセット適用時に既存の起動予定ノード一覧へ未保存の編集がある場合は、上書き確認を出す。確認ダイアログには `現在の起動予定を破棄してプリセットを適用する` ことを明記する。プリセット適用後も、まだ起動は行わない。

### 4.4 起動候補ツリー

カテゴリは以下とする。

| カテゴリ | profile |
| --- | --- |
| 実機基盤 | `ypspur_ros2` |
| GPS/GNSS | `rtk_gps_um982` |
| シミュレーション基盤 | `obstacle_route_sim` |
| 経路計画・管理 | `route_planner`, `route_manager` |
| 経路追従・走行制御 | `route_follower`, `drive_mode_manager`, `robot_navigator` |
| 障害物 | `obstacle_monitor` |
| 認識・監視 | `road_blockage_detector`, `traffic_signal_recognizer` |
| 可視化 | `robot_console_rviz`, `route_markers`, `target_marker` |

各profile行に表示する内容:

- 起動対象チェックボックス
- profile名
- 状態badge `STOPPED / STARTING / RUNNING / ERROR`
- health summary `OK / STALE / LOST`
- 個別起動/停止ボタン

### 4.5 起動予定ノード一覧

起動予定ノード一覧は、現在一斉起動の対象になっているノード全体を把握するための中心UIである。カテゴリツリーでチェックされたprofile、またはプリセット適用で選ばれたprofileを起動順序順に表示する。

表示項目:

| 表示 | 内容 |
| --- | --- |
| 起動順 | 一斉起動時の順序。ドラッグまたは上下ボタンで変更可能にする。 |
| profile名 | `rtk_gps_um982` などのprofile_idと表示名。 |
| カテゴリ | 実機基盤、GPS/GNSS、経路計画・管理など。 |
| config | 選択中configファイル名。詳細pathはtooltipまたは設定カードで表示する。 |
| override概要 | `cmd_vel_topic=/cmd_vel/autonomous` など主要な上書き値だけを短く表示する。 |
| 状態 | STOPPED / STARTING / RUNNING / ERROR。 |
| health | OK / STALE / LOST の要約。 |
| 操作 | 個別起動、個別停止、一覧から除外。 |

起動予定ノード一覧で1行を選択すると、右ペインのノード設定編集パネルと下部の起動内容プレビューがそのノードに切り替わる。一覧では複数ノードの全体像を確認し、詳細編集は選択中の1ノードだけに限定する。

一覧から除外したノードは停止しない。未起動ノードであれば一斉起動対象から外れるだけである。起動済みノードを除外しようとした場合は、`一覧から外す` と `停止して外す` を明確に分ける。

起動予定ノード一覧は、内部状態として `selected_for_launch` と `process_state` を分けて持つ。`selected_for_launch=false` は一斉起動対象外を意味するだけで、起動済みプロセスの停止を意味しない。停止操作は必ず個別停止または起動予定ノード停止の明示操作から実行する。

### 4.6 ノード設定編集パネル共通仕様

ノード設定編集パネルは、起動予定ノード一覧で選ばれている1ノード分だけを表示する。ここでconfigファイル、launch引数、起動時override値を編集する。profile定義から自動生成し、キーボード入力は必要最小限にする。

| 設定種別 | UI部品 | 例 |
| --- | --- | --- |
| configファイル | Combobox + refresh button | `config/default.yaml`, `params/tsukuba.yaml` |
| bool | CheckBox / Switch | `start_gazebo_gui`, `enable_pylons`, `start_gui` |
| enum | ComboBox / RadioButton | `joy_input`, `road_type`, YOLO launch variant |
| topic候補 | ComboBox + advanced free input | `/cmd_vel`, `/cmd_vel/autonomous` |
| label候補 | ComboBox / multi-select | `start_label`, `goal_label`, `checkpoint_labels` |
| numeric | SpinBox / DoubleSpinBox | `road_width`, `pylon_seed` |
| 秘匿値 | 表示しない、またはmasked readonly | NTRIP password |

自由入力欄は、候補リストで表現できないtopic名やlabelを扱う場合だけ `詳細入力` として折りたたむ。

ノード設定編集パネルで入力した値はconfig YAMLへ書き戻さない。入力値は、起動時に渡すlaunch引数、parameter override、またはprofile固有のuser argumentとして扱う。保存機能を追加する場合は、本仕様とは別に保存先、差分確認、秘匿値mask、rollback方法を設計する。

### 4.7 profile別設定

#### ypspur_ros2

| 項目 | UI | 既定 |
| --- | --- | --- |
| `config` | YAML Combobox | `config/default.yaml` |
| `cmd_vel_topic` | ComboBox | `/cmd_vel` |
| `odom_topic` | ComboBox | `/odom` または `/ypspur_ros/odom` |
| `start_coordinator` | CheckBox | `false` |
| `coordinator_device` | ComboBox / 詳細入力 | `<serial_device>` |
| `coordinator_param` | File selector | `<robot_param_file>` |

`start_coordinator=true` の場合は、起動前に実機安全確認ダイアログを表示する。

#### rtk_gps_um982

| 項目 | UI | 既定 |
| --- | --- | --- |
| `config` | YAML Combobox | `config/default.yaml` |
| health topics | readonly | `/rtk_gps/fix`, `/rtk_gps/heading`, `/rtk_gps/rtk_status` |

NTRIP passwordはGUIで編集しない。config内容を表示する場合もmasked表示とする。

#### obstacle_route_sim

| 項目 | UI | 既定 |
| --- | --- | --- |
| `road_type` | ComboBox | `straight` |
| `road_width` | DoubleSpinBox | `5.0` |
| `enable_pylons` | CheckBox | `false` |
| `pylon_seed` | SpinBox | `0` |
| `enable_route_blocker` | CheckBox | `false` |
| `start_gazebo_gui` | CheckBox | `true` |
| `use_sim_time` | CheckBox | `true` |

#### route_planner

| 項目 | UI | 既定 |
| --- | --- | --- |
| param file | YAML Combobox | `params/default.yaml` |

#### route_manager

| 項目 | UI | 内容 |
| --- | --- | --- |
| param file | YAML Combobox | route設定を含むYAML |
| start_label | waypoint ComboBox | routeから候補抽出 |
| goal_label | waypoint ComboBox | routeから候補抽出 |
| checkpoint_labels | multi-select list | 複数選択 |

#### route_follower

| 項目 | UI | 既定 |
| --- | --- | --- |
| param file | YAML Combobox | `params/default.yaml` |

#### drive_mode_manager

| 項目 | UI | 選択肢 |
| --- | --- | --- |
| `joy_input` | RadioButton | `joy_node`, `ps3_joy_sim` |
| `start_gui` | CheckBox | true/false |

#### robot_navigator

| 項目 | UI | 既定 |
| --- | --- | --- |
| param file | YAML Combobox | `params/default.yaml` |
| `cmd_vel_topic` | ComboBox | `/cmd_vel/autonomous` |
| `odom_topic` | ComboBox | `/ypspur_ros/odom` |
| Simulator代替を使用 | CheckBox | `false`（ONで`robot_simulator.launch.py`に切替。GPS/ypspur無しで自己位置・odomを疑似生成する） |

#### obstacle_monitor

| 項目 | UI | 既定 |
| --- | --- | --- |
| param file | YAML Combobox | `params/default.yaml` |
| Simulator代替を使用 | CheckBox | `false`（ONで`laser_scan_simulator.launch.py`に切替。疑似LaserScanを配信する） |

#### road_blockage_detector

| 項目 | UI | 既定 |
| --- | --- | --- |
| `detector_param_file` | YAML Combobox | `params/default.yaml` |
| PyTorch版YOLOを使用 | CheckBox | `false`（ONで`road_blockage_perception_yolo.launch.py`に切替） |
| Simulator代替を使用 | CheckBox | `false`（ONで`yolo_detector/camera_simulator_node.launch.py`に切替。実カメラ無しで疑似画像を配信する） |

#### traffic_signal_recognizer

| 項目 | UI | 既定 |
| --- | --- | --- |
| `recognizer_param_file` | YAML Combobox | `params/default.yaml` |
| Simulator代替を使用 | CheckBox | `false`（ONで`yolo_detector/camera_simulator_node.launch.py`に切替。実カメラ無しで疑似画像を配信する） |

#### robot_console_rviz / route_markers / target_marker

| profile | 項目 | UI | 既定 |
| --- | --- | --- | --- |
| `robot_console_rviz` | rviz config | readonly | `rviz/robot_console_view.rviz` |
| `route_markers` | `active_route_topic` / `marker_topic` | readonly | `/active_route` / `/active_route/markers` |
| `target_marker` | `active_target_topic` / `marker_topic` | readonly | `/active_target` / `/active_target/marker` |

これら3profileは可視化専用であり、業務モードのプリセットには含めない。RViz確認やマーカー表示が必要な場合、起動候補ツリーからユーザーが個別に追加する。

### 4.8 起動内容プレビュー

起動内容プレビューは、起動予定ノード一覧で選ばれている1ノード分だけを表示する。起動予定ノード全体の一覧は中央ペインで確認できるため、プレビューには全ノード分を並べない。

表示内容:

- 選択中profile名
- 一斉起動内での起動順序
- `ros2 launch` コマンド相当の引数
- configファイル
- ノード設定編集パネルで指定した起動時override値
- health topics
- 秘匿値mask状態
- 起動前validation結果

例:

```text
Profile: rtk_gps_um982
Command:
  ros2 launch rtk_gps_um982 rtk_gps_um982.launch.py config:=config/default.yaml
Health:
  /rtk_gps/fix
  /rtk_gps/heading
  /rtk_gps/rtk_status
```

起動内容プレビューでは、configファイルの全内容を展開しない。表示するのは、選択中config、GUIで指定した起動時override、最終的なlaunch引数、validation結果に限定する。YAML内の秘匿値や長大な配列は表示しない。

## 5. コンソールログタブ

### 5.1 役割

コンソールログタブは、起動後の詳細確認、異常調査、開発時のデバッグを目的とする。通常運用では、各ノードの状態はダッシュボードで一元監視し、詳細を見たい場合にコンソールログタブへ移動する。

### 5.2 レイアウト

```text
┌────────────────────────────────────────────────────────────┐
│ profile filter  level filter  search  tail ON/OFF           │
├──────────────────────┬─────────────────────────────────────┤
│ profile一覧           │ 選択中profileのリアルタイムログ        │
├──────────────────────┴─────────────────────────────────────┤
│ WARN/ERROR統合ログ                                          │
└────────────────────────────────────────────────────────────┘
```

### 5.3 表示内容

profile一覧:

- profile名
- RUNNING/STOPPED/ERROR
- PID
- 終了コード
- 最終ログ時刻
- WARN/ERROR件数

ログ表示:

- stdout/stderrを時系列表示
- `[INFO]`, `[WARN]`, `[ERROR]`, `[FATAL]`, `[DEBUG]` の色分け
- ANSI escape sequenceは可能なら解釈し、初期実装では文字列ベース色分けでよい
- tail追従ON/OFF
- 検索語highlight
- ログファイルパス表示

### 5.4 業務フロー上の操作

起動直後は、起動・設定タブからコンソールログタブへ移動し、選択中profileのログを確認する。全ノードが正常ならダッシュボードへ移動する。走行中に異常badgeが出た場合は、ダッシュボードのノード状態サマリから対象profileへジャンプできるようにする。

コンソールログタブは詳細調査用であり、走行状態の正本表示にはしない。ノードの正常/異常の要約はダッシュボードのNode Healthカードへ集約し、ログタブは「なぜ異常か」を見る場所として実装する。

## 6. ダッシュボードタブ

### 6.1 役割

ダッシュボードは、走行前確認と走行中監視の主画面である。自律走行中は基本的にこのタブから変更しない。詳細な走行状態、GPS受信状況、ノードhealth、手動介入操作をこの画面に集約する。

### 6.2 レイアウト

```text
┌────────────────────────────────────────────────────────────┐
│ 運行フェーズ / 走行モード / manual_start / 重要イベント      │
├──────────────────────┬──────────────────────┬──────────────┤
│ Route / Follower      │ Drive / CmdVel        │ GPS / Pose    │
├──────────────────────┼──────────────────────┼──────────────┤
│ Event                 │ Manual Ops            │ Node Health   │
│                       │ (tabbed)              │               │
└────────────────────────────────────────────────────────────┘
```

ダッシュボードも16:9の論理キャンバス内で設計し、スクロールなしで全体を表示する。Event timeline / latest warningsの独立した横長領域は置かない。イベント履歴や警告要約はEventカードまたはNode Healthカードへ統合し、広い操作領域が必要なManual Opsカードへ画面面積を優先配分する。

### 6.3 運行フェーズ領域

表示項目:

- 業務モード `実機/シミュレーション`, `手動/自律`
- 運行フェーズ `未起動`, `起動確認中`, `走行準備完了`, `走行中`, `一時停止`, `異常`, `終了処理中`
- route progress
- current waypoint / next waypoint
- `manual_start` 現在値
- 一時停止理由または復帰待ち理由

操作:

- `manual_start=True` 送信
- 一時停止からの復帰用topic送信
- 必要に応じた `manual_start=False` または停止系操作

復帰用topicの具体名は実装時点のroute/drive仕様に合わせる。画面仕様としては、復帰操作をダッシュボードに置くことを必須とする。

#### 6.3.1 運行フェーズの判定条件

運行フェーズは単独のtopicでは表現されないため、起動管理状態（`LaunchProfileState.status`）、`route_state`、`follower_state`、`drive_mode_status`、`manual_start` の組み合わせから決定する。重い状態を優先し、以下の順に評価して最初に成立したフェーズを採用する。

| 順 | フェーズ | 判定条件 |
| --- | --- | --- |
| 1 | `異常` | いずれかのprofileが `ERROR`、または `follower_state.state == ERROR`、または `route_state.status == STATUS_ERROR` |
| 2 | `終了処理中` | いずれかのprofileが `STOPPING`、または `route_state.status == STATUS_COMPLETED`、または `follower_state.state == FINISHED` |
| 3 | `未起動` | `route_state` / `follower_state` をいずれも未受信で、起動中・起動済みのprofileも無い |
| 4 | `起動確認中` | いずれかのprofileが `STARTING`、または起動済みprofileがあるが `route_state` / `follower_state` を未受信 |
| 5 | `一時停止` | `manual_start=True` かつ停止要因あり（下表） |
| 6 | `走行中` | `manual_start=True` かつ停止要因が無く、`follower_state.state` が `RUNNING`/`AVOIDING`、または `route_state.status == STATUS_RUNNING` |
| 7 | `走行準備完了` | 上記以外（運行系topic受信済みで走行開始待ち） |

`robot_console` がノードを起動しない使い方（HTML遠隔観測UIの単独起動など）では全profileが `STOPPED` のままになるため、起動管理状態だけでは判定せず、運行系topicの受信有無を併用する。受信実績があるtopicが途絶えた場合（`STALE`/`LOST`）は「未受信」とは区別し、`未起動` へは戻さない（途絶自体の提示はEventカードと鮮度表示の責務とする）。

一時停止の理由は、操作者が次に取る判断に近い順で最初に成立したものを1件表示する。

| 順 | 停止要因 | 表示する理由 |
| --- | --- | --- |
| 1 | `follower_state.state == WAITING_STOP` | 停止waypointで待機中（信号/停止線） |
| 2 | `follower_state.state == WAITING_REROUTE` | 再経路待ち（route_managerの応答待ち） |
| 3 | `follower_state.state == STAGNATION_DETECTED` | 滞留検知（回避判断中） |
| 4 | `route_state.status == STATUS_HOLDING` | route_manager holding（`ManagerStatus.last_cause` を併記） |
| 5 | `drive_mode_status.mode == MODE_MANUAL` | 手動介入中 |
| 6 | `drive_mode_status.auto_resume_pending` | 自律復帰待ち |

業務モードのうち実行環境（`実機` / `シミュレーション` / `机上確認`）は起動・設定タブの選択値を用いる（ROS topicからは得られないため、選択が無い場合は `unknown` とする）。走行モード（`手動` / `自律`）は実際に走行制御が選択しているモードを正とし、`drive_mode_status` 未受信の間だけ起動・設定タブの走行モード選択値で代替する。

current waypoint は `route_state.current_label`（空の場合は `follower_state.active_waypoint_label`、いずれも空なら `#index`）、next waypoint は `active_route.waypoints` から次のindexのlabelを引き、labelが無い場合は `#index` 表記とする。次のindexが総waypoint数を超える場合は `goal` と表示する。

ダッシュボードでは、走行中に必要な操作だけを表示する。起動設定、config変更、launch引数変更、ノード候補の追加は起動・設定タブに閉じる。走行中に設定変更が必要になった場合は、ダッシュボードから直接編集させず、停止または安全確認を経て起動・設定タブへ移動する導線にする。

### 6.4 Route / Followerカード

| 表示 | 内容 |
| --- | --- |
| route_manager | state、route version、最終decision、再計画理由 |
| route_follower | state、active waypoint、target waypoint、stagnation reason |
| progress | index、総waypoint数、進捗率 |
| target distance | 目標までの距離、到達判定 |

### 6.5 Drive / CmdVelカード

| 表示 | 内容 |
| --- | --- |
| drive mode | autonomous/manual、output source、auto resume pending |
| `/cmd_vel` | linear x、angular z、freshness |
| `/cmd_vel/autonomous` | 自律指令の有無、mux後との差分 |
| odom | `/odom` または `/ypspur_ros/odom` freshness |

### 6.6 GPS / Poseカード

| 表示 | データ | 表示例 |
| --- | --- | --- |
| RTK | `/rtk_gps/rtk_status.rtk_state` | `RTK_FIX` |
| Satellites | `num_satellites` | `18 sat` |
| HDOP | `hdop` | `0.8` |
| Correction | `correction_age_s` | `1.2 s` |
| RTCM | `rtcm_bytes_received` と増加有無 | `125034 B` |
| Heading | `heading_deg`, `heading_stddev_deg` | `123.4 deg +/- 0.8` |
| Localization source | 現行/将来source | `pose_enu` / `pose_llh` |
| Pose freshness | `/localization/pose_enu` または `pose_llh` | `OK 0.1s` |

色ルール:

| 条件 | 表示 |
| --- | --- |
| `RTK_FIX` かつfreshness OK | 緑 |
| `RTK_FLOAT` | 黄 |
| `DGPS` または `STANDALONE` | 橙 |
| 未受信、`UNKNOWN`、freshness LOST | 赤 |
| シミュレーションでGPS未使用 | 灰、`GPS N/A` |

### 6.7 Eventカード

Eventカードは、現行GUIのsignal stopなどのバナー表示を発展させた領域である。走行中に操作者が即座に気づくべき状態変化を、優先度付きのイベントバナーとして表示する。ObstacleとSignalはここへ統合してよいが、topic送信系の操作はすべてManual Opsカード側に置く。

表示対象:

| イベント | 表示内容 | 表示例 |
| --- | --- | --- |
| signal stop / WAITING_STOP | 停止理由、対象waypoint、待機継続時間 | `SIGNAL STOP A-24 12.4s` |
| signal GO受信 | GO判定、受信時刻、入力元 | `SIGNAL GO external 0.2s ago` |
| front_blocked | `front_clearance_m`, left/right offset, freshness | `FRONT BLOCKED clearance=0.8m L=0.3 R=-0.1` |
| road_blocked | 道路封鎖状態、入力元、最終更新 | `ROAD BLOCKED external` |
| route update / replan | route version、理由、時刻 | `ROUTE UPDATED v5 blocked_segment` |
| topic lost / stale | 対象topic、経過秒 | `GPS STATUS LOST 10.5s` |
| launch/profile error | profile名、ERROR要約 | `robot_navigator ERROR` |

優先順位は、`profile error/topic lost`、`road_blocked`、`front_blocked`、`signal STOP/WAITING_STOP`、`manual_start待ち`、`signal GO`、`route update` の順を基本とする。複数イベントが同時に存在する場合、最上位イベントを大きく表示し、下部に2から3件の小型履歴を表示する。

Eventカードに置かないもの:

- `sig_recog` GO/STOP送信ボタン
- `road_blocked` True/False送信ボタン
- obstacle hint override開始/停止
- manual_start送信

これらはManual Opsカードに置く。Eventカードは状態表示と注意喚起に限定する。

### 6.8 Node Healthカード

Node Healthカードは、起動・設定タブで起動予定または起動済みになったprofile群の状態を一元監視する領域である。現時点の設計上、表示対象profileは以下の14件規模になる。

| 分類 | profile |
| --- | --- |
| 実機基盤 | `ypspur_ros2` |
| GPS/GNSS | `rtk_gps_um982` |
| シミュレーション基盤 | `obstacle_route_sim` |
| 経路計画・管理 | `route_planner`, `route_manager` |
| 経路追従・走行制御 | `route_follower`, `drive_mode_manager`, `robot_navigator` |
| 障害物 | `obstacle_monitor` |
| 認識・監視 | `road_blockage_detector`, `traffic_signal_recognizer` |
| 可視化 | `robot_console_rviz`, `route_markers`, `target_marker` |

14件全ての詳細を常時カード内に表示すると画面を圧迫するため、Node Healthカードは詳細表ではなく、異常優先のサマリビューとして設計する。

表示方式:

- 上段に集計を表示する: `RUNNING 8 / STOPPED 2 / WARN 1 / ERROR 1`。
- 中段にprofileチップをカテゴリ色付きで並べる。各チップはprofile名短縮、状態色、health状態だけを表示する。
- WARN/ERROR/LOSTのprofileは先頭へ自動的に並べ替える。正常profileは小さく圧縮表示する。
- 起動予定に入っていないprofileは原則表示しない。ただし重要基盤profileの未起動警告が必要な場合は、`required but not selected` として別色で表示する。
- チップを押すとコンソールログタブの該当profileへ遷移する。

チップに表示する情報:

| 表示 | 内容 |
| --- | --- |
| profile短縮名 | `gps`, `ypspur`, `manager`, `navigator` など |
| 状態色 | RUNNING=緑、STARTING=青、WARN/STALE=黄、ERROR/LOST=赤、STOPPED=灰 |
| health要約 | health topicの最悪状態 |
| 最新ログlevel | WARN/ERRORがある場合のみ小さく表示 |

詳細なPID、終了コード、ログ本文、health topicごとの行はコンソールログタブに置く。Node Healthカードには、走行中に一目で異常対象を特定するための最小情報だけを表示する。

### 6.9 Manual Opsカード

Manual Opsカードは、走行中に操作者が明示的にtopic送信や手動介入を行うための領域である。手動操作を行う際はGUI上での操作が必須であるため、十分な面積を確保する。Event timeline / latest warnings用の独立領域は設けず、その分をManual Opsカードの操作領域に割り当てる。

Manual Opsカード内はタブ切り替え方式にする。既定表示は `manual_start` タブとし、走行開始または復帰操作へ最短で到達できるようにする。

| タブ | 操作 | 主なUI | 備考 |
| --- | --- | --- | --- |
| `manual_start` | `/manual_start` 送信 | 大型送信ボタン、現在値、最終送信時刻 | 既定タブ。走行開始・復帰で使用 |
| `signal` | `/sig_recog` GO/STOP送信 | GO/STOP segmented button、送信ボタン、現在値 | signal topic送信系はここへ集約 |
| `road_blocked` | `/road_blocked` True/False送信 | True/False選択、確認付き送信 | 入力元と外部上書き状態を表示 |
| `obstacle_hint` | `/obstacle_avoidance_hint` override | `front_blocked`, clearance, left/right offset, start/stop | 数値はSpinBox/DoubleSpinBox |
| `frame_image` | `/frame_image_path` 送信 | path選択、送信 | 通常運用では詳細タブ扱い |

Manual Opsカードに表示する共通情報:

- 対象topic名
- 現在値
- 最終受信時刻
- 最終送信値
- 最終送信時刻
- 入力元 `GUI / external / unknown`
- 送信結果 `sent / waiting_echo / confirmed / timeout`

危険度の高い `road_blocked=True`、obstacle hint override開始、停止系操作は確認ダイアログを出す。Eventカードにはこれらの操作ボタンを置かない。

## 7. 自己位置・センサ情報タブ

### 7.1 役割

自己位置・センサ情報タブは、ローカルGUIで詳細確認が必要な場合に使う補助画面である。走行中の通常監視はダッシュボード、遠隔または並行確認はHTML UIを主とする。

### 7.2 レイアウト

```text
┌────────────────────────────────────────────────────────────┐
│ 運行サマリ  GPSサマリ  localization source                  │
├───────────────────────────────┬────────────────────────────┤
│ 地図 / route overlay           │ 自己位置・GPS詳細           │
├───────────────────────────────┴────────────────────────────┤
│ センサ・画像パネルグリッド                                  │
└────────────────────────────────────────────────────────────┘
```

### 7.3 運行サマリ

ダッシュボードの詳細を再掲せず、以下だけを表示する。

- 運行フェーズ
- route_follower state
- current/target waypoint
- progress
- GPS state
- localization freshness
- sensor freshness summary

### 7.4 地図表示

初期実装は現行互換として `/localization/pose_enu`、`/active_route`、`/active_target` を表示する。将来は `pose_llh` とLLH route/targetを主表示にする。

表示要素:

- 現在自己位置
- GPS fix位置
- active target
- waypoint列
- 走行済み区間
- 未走行区間
- pose_enusourceとLLH sourceの差分
- pose freshness

UI側は座標変換規約を持たず、Coreから地図overlay用Viewを受け取る。

自己位置・センサ情報タブは詳細確認用であり、手動介入や起動停止操作を置かない。地図上のクリックでwaypoint編集、goal変更、manual_start送信などを行わない。地図・画像・鮮度は観測と診断に限定する。

### 7.5 センサ・画像パネル

パネルはSnapshotの `sensor_panels` から生成する。

| panel_id | title | topic |
| --- | --- | --- |
| `route_map` | Route Map | `/active_route` または将来LLH route view |
| `sensor_viewer` | Sensor Viewer | `/sensor_viewer` |
| `road_blockage` | Road Blockage | `/perception/road_blockage/decision_image` |
| `traffic_signal` | Traffic Signal | `/perception/traffic_signal/decision_image` |
| `front_camera` | Front Camera | 将来追加topic |
| `lidar_view` | LiDAR View | 将来追加topic |

各パネルはtitle、topic、freshness、最終更新時刻、画像またはplaceholderを表示する。

## 8. HTML遠隔観測UI

### 8.1 役割

HTML UIは、走行中にローカルGUIをダッシュボードに固定したまま、別ブラウザまたは遠隔端末で自己位置・センサ情報を確認するための観測専用UIである。

### 8.2 表示内容

- 運行サマリ
- GPS/GNSSサマリ
- localization sourceとfreshness
- 地図 / route overlay
- センサ・画像パネル
- topic鮮度一覧
- node health summary

### 8.3 禁止事項

HTML UIには以下を置かない。

- manual_start送信
- sig_recog送信
- road_blocked送信
- obstacle hint override
- launch操作
- param編集
- 全ログ操作

HTML UIは読み取り専用であることを実装上も保証する。HTTP APIはSnapshot、画像、healthのGET系に限定し、topic送信、launch、設定更新に相当するPOST/PUT/DELETE APIを提供しない。

## 9. 画面間導線

画面の切り替えはタブ操作で行い、行き先を指定するだけの遷移ボタンは画面へ置かない。
ダッシュボードは走行中に常時見る画面であり、タブと重複するボタンで表示面積を
消費しないためである（2.1節「スクロールなしで全体表示」）。

画面へ置く導線は、遷移と同時に**対象を引き渡す**ものに限る。

| 起点 | 導線 | 引き渡す対象 | 目的 |
| --- | --- | --- | --- |
| ダッシュボード Node Health | 対象profileログへ移動 | クリックしたprofile | 異常詳細確認 |

タブ操作で足りる以下の遷移は、専用ボタンを設けない。

| 遷移 | 代替 |
| --- | --- |
| ダッシュボード → 自己位置・センサ情報 | 「自己位置・センサ情報」タブ |
| 自己位置・センサ情報 → ダッシュボード | 「ダッシュボード」タブ |
| コンソールログ → ダッシュボード | 「ダッシュボード」タブ |

## 10. 入力設計

不必要なキーボード入力を避ける。

- 既知の選択肢はComboBox、RadioButton、CheckBoxを使う。
- numeric値はSpinBox/DoubleSpinBoxを使う。
- waypoint labelはroute読込後の候補リストから選ぶ。
- 複数checkpointはmulti-select listで選ぶ。
- topic名は既定候補をComboBoxに出し、特殊時のみ詳細入力を許可する。
- configファイルは検出済み候補から選ぶ。
- 秘匿値は編集させない。

## 11. テスト計画・受け入れ条件

実装時の受け入れ条件:

- 共通ステータスバーが存在しない。
- ダッシュボードだけで走行状態、GPS、node health、手動介入が確認・操作できる。
- 自己位置・センサ情報タブには運行サマリがあり、詳細は地図・GPS・画像・鮮度に集中している。
- 起動・設定タブは業務モードから推奨起動セットを選択できる。
- config、enum、bool、numeric、waypoint候補はノード設定編集パネル上の適切なGUI部品で入力できる。
- コンソールログタブでprofile別リアルタイムログを確認できる。
- HTML UIに操作ボタンが存在しない。
- `rtk_gps_um982` のconfig選択とhealth topic表示が起動・設定タブに存在する。
- 現行 `/localization/pose_enu` と将来 `pose_llh` のsource表記が画面上で区別できる。

設計書作成時の確認:

- topic名、profile_id、launch file名を既存パッケージと照合する。
- コード本体を変更しないため、`colcon build` と `pytest` は実行対象外とする。

## 12. 互換性・移行・影響範囲

- 現行 `Dashboard / Console Logs` 中心の構成から、4タブ構成へ拡張する。
- 起動管理は現行のprofile概念を維持しつつ、業務モード、カテゴリ、health topic、起動グループを追加する。
- GPS/GNSS表示のため、実装時には `rtk_gps_um982_msgs/msg/RtkStatus` の購読依存を追加する。
- 自己位置は当面 `/localization/pose_enu` 互換表示を維持し、将来 `localization_fusion/pose_llh` を正sourceにする。
- 起動管理対象の増減はprofile定義追加で扱い、UIコードの個別分岐を避ける。
- 業務分類に机上確認（実センサ・Gazebo無し）を追加し、`robot_navigator`/`obstacle_monitor`/`road_blockage_detector`/`traffic_signal_recognizer`のsimulator代替launchと、可視化専用profile（`route_markers`/`target_marker`）を起動管理対象へ追加する。

## 13. 改版履歴

| 日付 | 版 | 変更概要 |
| --- | --- | --- |
| 2026-08-29 | 0.3 | 業務分類に机上確認を追加。profile別設定（4.7節）に不足していた`route_planner`/`route_follower`/`obstacle_monitor`/`road_blockage_detector`/`traffic_signal_recognizer`/可視化系のカードとsimulator代替トグルを追加。起動候補ツリー・Node Healthカードに`route_markers`/`target_marker`を追加。 |
| 2026-05-28 | 0.2 | 共通ステータスバーを廃止し、実業務フロー起点の正式画面仕様へ改訂。 |
| 2026-05-27 | 0.1 | 次期GUIの画面構成、GPS表示、profile定義駆動の起動管理を初版として作成。 |
