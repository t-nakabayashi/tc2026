# road_blockage_detector 詳細設計書

## 1. 目的・スコープ

本書は `road_blockage_detector` ノードの内部設計を定める。YOLO 推論結果と自己位置を入力と
して、経路封鎖看板の有無と封鎖位置を判定する。判定結果は `robot_navigator` を経由して
ロボットの停止・再開へ反映される。

トピック名、型、起動方法は [README](../README.md) を正とする。本書は**判定の時系列処理、
閾値、多重検知の抑止、認識結果の出力規則、異常時の挙動**を定める。

前提は次のとおりである。

- YOLO 推論結果は `yolo_detector` パッケージが `vision_msgs/Detection2DArray` として
  publish 済みである。
- **本ノードは YOLO モデルのロードや画像推論を行わず、検出結果の意味判定だけを担当する。**
- **`/localization/pose_enu` を唯一の自己位置情報として使用し、TF2 には依存しない。**
- **本ノードは画像を購読・配信しない。** 判定の根拠は検出矩形と判定状態として配信し、
  画像への重畳は表示側（`robot_console`）が行う。
- 過去の経路封鎖位置はノード内メモリで累積保持する。**永続化しない。**

**本パッケージの launch はいずれも `use_sim_time` 引数を提供しない。** 判定は入力メッセージ
の stamp を使うため、検知と自己位置の時刻基準を呼び出し側で一致させる必要がある。launch の
選択だけで入力の時刻不整合が補正されるわけではない。

記載内容はすべて実装済みである。**自動試験を持たない**（9.1 節）。

## 2. 構成とファイル配置

| パス | 用途 |
| --- | --- |
| `road_blockage_detector/road_blockage_detector_node.py` | 判定ノード（約 500 行） |
| `launch/road_blockage_detector.launch.py` | 判定ノード単体の起動 |
| `launch/road_blockage_perception.launch.py` | 認識系一式（NCNN 推論 + 判定）の起動 |
| `launch/road_blockage_perception_yolo.launch.py` | 認識系一式（PyTorch 推論 + 判定）の起動 |

**ROS 非依存のロジックへ切り出していない。** 判定はノードのメソッドとして実装する。

責務は次のとおりである。

| 区分 | 内容 |
| --- | --- |
| 主要役割 | YOLO 検知から経路封鎖看板候補を抽出し、時系列の判定カウントを管理して仮／確定封鎖を判断する |
| 責務1 | 検知パラメータ（クラス ID・スコア・バウンディングボックス閾値）に基づくフィルタリング |
| 責務2 | 判定期間内のカウント履歴から封鎖を評価し、`road_blocked` を publish する |
| 責務3 | 確定封鎖とみなした自己位置を履歴として保持し、再侵入時の多重検知抑止を行う |
| 責務4 | 自己位置が取得できない場合や、検知と自己位置の時刻差が大きい場合に警告を出す |
| 責務5 | 検出矩形、採用可否、判定状態を配信し、表示側が画像へ重畳できるようにする |

## 3. 外部インタフェース仕様

一覧は [README](../README.md) を正とする。本章は QoS の選定理由と、一覧に現れない取り扱いを
定める。

### 3.1 QoS

| 対象 | 設定 | 選定理由 |
| --- | --- | --- |
| 検知結果の購読 | SensorData（BEST_EFFORT） | **配信元が BEST_EFFORT のため、RELIABLE では購読が成立しない** |
| 認識結果の配信 | SensorData | 購読側（`robot_console`）が BEST_EFFORT で購読する |
| 自己位置の購読 | 既定 | — |
| `/road_blocked` の配信 | 既定 | — |

### 3.2 `/road_blocked`（配信）

**仮判定の状態が遷移したときだけ publish する**（false→true、true→false）。周期配信は
しない。

これにより、購読側（`route_follower`）の「最終受信からの経過時間で継続を判定する」実装が
成立する（
[route_follower 詳細設計書](../../route_follower/docs/route_follower_詳細設計書.md)
7.5 節）。**周期的に再送する実装へ変えると、購読側の封鎖継続判定が成立しなくなる。**

### 3.3 自己位置の取り扱い

`/localization/pose_enu` を購読し、**メッセージヘッダーの時刻を含めてキャッシュする。**

| 状況 | 挙動 |
| --- | --- |
| 最新の自己位置が未取得 | 警告を出して**検知処理をスキップする** |
| 検知と自己位置のヘッダー時刻差が閾値以上 | 警告を出すが、**処理は続行する** |

### 3.4 認識結果の配信

`/perception/road_blockage/overlay` に次を載せる。**本ノードは画像を扱わない。**

- **受信した全 bbox。** 有効検知として採用したものは採用フラグを真、閾値未満・対象 class
  外・bbox 条件外で除外したものは偽とする。採用外も残すことで、判定に使われなかった検知を
  表示側で描き分けられる。
- 判定（封鎖なし / 封鎖あり）と判定文（`clear` / `judging` / `confirmed`）。
- 自己位置未取得や多重検知抑止で判定できなかった場合は補足へ `no_pose` / `suppressed` を
  入れる。

header は元画像フレームのものを引き継ぐ。

**描画内容の決定は表示側の責務とする。** `robot_console` は採用した検出を太線、採用外を
細線で描き分け、判定結果は画像へ焼き込まずチップとして表示する。

**配信は運用確認用であり、制御判断の正本は `/road_blocked` とする。**

### 3.5 画像 topic の位置づけ

| topic | 配信元 | 内容 |
| --- | --- | --- |
| `/perception/road_blockage/detection_image` | `yolo_detector` | YOLO 生検出の確認用画像 |
| `/perception/road_blockage/overlay` | 本ノード | 意味判定後の認識結果（画像ではない） |

`robot_console` は `/usb_cam/image_raw` へ本 overlay を重畳して 1 枚のカメラパネルに表示
する。未受信時は判定チップを未受信として扱う。

## 4. 設定の所在

| 名称 | 型 | 既定値 | 説明 |
| --- | --- | --- | --- |
| `detections_topic` | string | `/perception/road_blockage/detections` | YOLO 推論結果入力 topic |
| `pose_enu_topic` | string | `/localization/pose_enu` | 自己位置入力 topic |
| `road_blocked_topic` | string | `/road_blocked` | 経路封鎖判定出力 topic |
| `overlay_topic` | string | `/perception/road_blockage/overlay` | 認識結果の出力 topic |
| `target_class_id` | int | 0 | 経路封鎖看板に対応するクラス ID |
| `score_threshold` | float | 0.5 | 最小スコア。未満の検知は除外 |
| `bbox_width_min` / `bbox_width_max` | float | -1 | 幅閾値 [pixel]。**負値なら判定スキップ** |
| `bbox_height_min` / `bbox_height_max` | float | -1 | 高さ閾値 [pixel]。**負値なら判定スキップ** |
| `bbox_bottom_max` | float | -1 | bbox 下端の最大位置 [pixel]。画像下端からの距離で定義。**負値なら判定スキップ** |
| `decision_duration` | float | 3.0 | 判定期間長 [秒]。この期間のカウント履歴を保持 |
| `decision_frame_ratio` | float | 50.0 | 判定期間内で「カウント>=1」の秒バケット割合 [%]。この値以上で仮判定成立 |
| `confirmation_duration` | float | 10.0 | `road_blocked=true` の継続時間が本値を超えたら封鎖確定 |
| `multi_detection_suppression_range` | float | 10.0 | 過去封鎖位置中心から本距離 [m] 以内に現在位置が入った場合はカウント 0 としてスキップ |

同梱の既定値は `yolo_detector/params/default.yaml` にも置かれている。**2 箇所に同じ
パラメータ群がある**ため、片方だけ変更すると起動経路によって値が変わる。

時刻差の警告閾値はパラメータを持たない。値は 7.4 節に置く。

## 5. データモデル・内部状態

| 名称 | 形式 | 内容 |
| --- | --- | --- |
| カウント履歴 | 秒バケットの両端キュー | 「秒単位バケットの開始時刻」と、その 1 秒間に記録した判定カウントの合計。`decision_duration` 秒を超えた古いバケットは随時削除する。**検知周期に依存せず秒単位のスライディングウィンドウを構成する** |
| 最終検知時刻 | 時刻 | 最後に検知を処理した時刻。間引きや欠損検出に利用 |
| 仮判定カウント | int | 秒バケット割合に応じて加算／リセット |
| 確定封鎖位置 | 姿勢の配列 | 確定封鎖と判断した際の map 座標（自己位置基準）。多重検知抑止に使用。**永続化しない** |
| 最新自己位置 | 姿勢 | 自己位置の最新値キャッシュ |
| 最新自己位置時刻 | 時刻 | 自己位置メッセージヘッダーの時刻。検知との乖離チェックに利用 |
| 封鎖状態の開始時刻 | float | `road_blocked=true` に遷移した時刻 [秒]。経過時間で確定判定 |
| 最新の有効検知数 | int | 最新フレームで閾値条件を満たした検知数。debug ログに利用 |
| 最新の検知割合 | float | カウント履歴から算出した検知割合 [%]。debug ログに利用 |

**確定封鎖位置は上限なく累積する。** 件数の上限や削除の契機を持たない。

## 6. 処理フローと異常時の挙動

### 6.1 走行系との連携

1. YOLO で経路封鎖看板を検出し、`road_blocked` を true で publish した時点で
   `robot_navigator` が走行を一時停止する。**停止中も本ノードは検知を継続する。**
2. 検知開始から `confirmation_duration` 経過前に封鎖検知が途切れた場合、`road_blocked` を
   false で publish し、`robot_navigator` が走行を再開する（**誤検知扱い**）。
3. 検知が `confirmation_duration` を超えて継続した場合は封鎖を確定し、封鎖位置を履歴へ
   追加する。**`road_blocked` は true のまま維持し、ロボットは停止を継続する。**
4. 封鎖確定後、`route_follower` 側で滞留と判定され、`route_manager` へ再計画を要求する。
   `route_manager` は固定区間では現在経路を再配信し、可変区間では planner の更新を試す。
   **経路更新や封鎖の回避が必ず成功するわけではない。**
5. 検知が取り消され `road_blocked=false` を publish した場合、`robot_navigator` は hold
   時間経過後に解除候補として取り込むため、誤検知時に自動で走行再開できる。

### 6.2 起動処理

1. パラメータを宣言・取得する。
2. 購読・配信を生成する。
3. 判定期間から秒単位バケットの上限幅を決定し、キューを初期化する。
4. 起動をログへ出す。

### 6.3 検知メッセージ受信時

1. 最新の自己位置を取得できていない場合は警告を出して処理を終了する。取得済みの場合は
   ヘッダー時刻差を確認し、閾値以上ずれていれば警告を出す（7.4 節）。
2. 確定封鎖位置と比較し、現在位置がいずれかの地点から `multi_detection_suppression_range`
   未満なら、**カウント履歴へ 0 を push して残処理をスキップする。**
3. 各検知について絞り込みを行う（7.1 節）。
4. 条件を満たした検知数を計数し、メッセージ時刻の秒バケットに加算する。新規バケットを
   生成する際は `decision_duration` より古いバケットを削除する。
5. 検知結果と判定状態を認識結果として配信する。
6. 判定ロジック（7.2 節）と封鎖確定処理（7.3 節）を行う。

### 6.4 異常時の挙動

| 事象 | 挙動 |
| --- | --- |
| 検知メッセージに要素が無い | **カウント履歴へ 0 を push し、割合計算を継続する** |
| 自己位置が未取得 | 処理をスキップし、警告を出す。回復後は通常処理へ復帰する |
| 検知と自己位置の時刻差が大きい | **警告のみを出し、処理は継続する** |
| 検知メッセージが連続で欠損 | 警告を出す |

### 6.5 ログ

| レベル | タイミング |
| --- | --- |
| info | ノード起動、`road_blocked` の true/false 遷移、封鎖確定時（位置情報を含む） |
| warn | 自己位置未取得時、検知と自己位置の時刻差が閾値以上、検知メッセージの連続欠損 |
| debug | フィルタ後の検知数、割合計算結果、履歴長などの内部状態 |

`error` と `fatal` は使わない。

`DiagnosticReporter` へ生存報告を登録する。監視側の扱いは
[ノード健全性監視設計](../../../docs/ノード健全性監視設計.md)を参照する。

## 7. 判定ロジックと閾値

### 7.1 検知の絞り込み

各検知について次を順に行う。

1. 仮説を走査し、最大スコアのクラスを決定する。
2. スコアが `score_threshold` 未満なら除外する。
3. クラス ID が `target_class_id` と一致しなければ除外する。
4. bbox の幅・高さ・下端位置を算出し、指定閾値の範囲に入らなければ除外する。
   **負値指定の閾値は判定をスキップする。**

### 7.2 仮判定

カウント履歴の秒バケット数を分母とし、**値が 1 以上のバケットの割合**を算出する。

| 条件 | 動作 |
| --- | --- |
| 割合が `decision_frame_ratio` 以上 | 仮判定カウントを +1。直前まで 0 だった場合は `road_blocked=true` を publish し、封鎖状態の開始時刻を現在時刻でセットして確定計測を開始する |
| 割合が閾値未満 | 仮判定カウントを 0 へリセット。直前まで 1 以上だった場合は `road_blocked=false` を publish し、封鎖計測時間をログへ出して開始時刻を未設定へ戻す（**誤検知として走行再開**） |

**秒単位のバケットを使うのは、検知周期に依存せず割合を求めるためである。** 推論周期が
変わっても判定の意味が変わらない。

### 7.3 封鎖確定

仮判定カウントが 1 以上かつ封鎖状態の開始時刻が設定済みの場合に経過時間を確認する。

経過時間が `confirmation_duration` 以上になったら、最新の自己位置から取得した姿勢を確定
封鎖位置へ追加し、仮判定カウントを 0 へクリアし、開始時刻を未設定へ戻す。

**`road_blocked` は true のまま維持する。** 以降は `route_follower` の滞留判定と
`route_manager` の再計画により走行再開が行われる。

### 7.4 時刻差の警告閾値

検知のヘッダー時刻と最新の自己位置のヘッダー時刻の差が **3 秒以上**の場合に警告を出す。
パラメータを持たない。処理は続行するため、**時刻がずれた状態でも判定は行われる。**

## 8. 並行性・周期・起動・依存関係

### 8.1 周期と並行性

**タイマーを持たない。** 処理はすべて検知メッセージのコールバック内で完結する。判定の頻度は
`yolo_detector` の推論周期に従う。

callback group を指定しないため、既定の group で単一スレッドの executor が直列に処理する。
ロックを持たない。

### 8.2 起動

| launch | 起動内容 |
| --- | --- |
| `road_blockage_detector.launch.py` | 判定ノード単体 |
| `road_blockage_perception.launch.py` | NCNN 推論ノード + 判定ノード |
| `road_blockage_perception_yolo.launch.py` | PyTorch 推論ノード + 判定ノード |

認識系一式の launch は `model_path` を引数として推論ノードへ渡す。

**`use_sim_time` 引数を提供しない**（1 章）。

### 8.3 依存関係

`ament_python` でビルドする。依存は `rclpy`、`tc_diagnostics`、`geometry_msgs`、
`std_msgs`、`tc_perception_msgs`、`vision_msgs` である。

**画像を扱わないため `cv_bridge` と OpenCV に依存しない。**

## 9. 確認方法と未確認範囲

### 9.1 自動確認

**自動試験を持たない。** `tests/` ディレクトリが無い。

試験を書く場合、判定部分（絞り込み、秒バケットの割合計算、仮判定の遷移、確定判定、多重
検知抑止）は ROS メッセージを模した入力で検証できる。ただし現在はノードのメソッドとして
実装されているため、ROS 非依存モジュールへの切り出しが前提になる。

### 9.2 手動で確認する範囲

- `camera_simulator_node` または実カメラで看板を入力し、`road_blocked` が true へ遷移
  すること
- `confirmation_duration` 経過前に検知が途切れた場合に false へ戻ること（誤検知扱い）
- `confirmation_duration` 経過後に封鎖が確定し、`road_blocked` が true のまま維持される
  こと
- 確定後に同じ地点へ再侵入したとき、多重検知が抑止されること
- 自己位置未取得で処理がスキップされ、警告が出ること
- 時刻差が大きい場合に警告が出るが処理は続くこと
- 認識結果の採用／採用外が `robot_console` で描き分けられること
- `/road_blocked` が状態遷移時のみ publish されること（3.2 節）
- 封鎖確定後に `route_follower` の滞留判定と `route_manager` の再計画まで通ること

### 9.3 未確認範囲

- **実機カメラでの検出精度と誤検知率を確認していない。** モデルの性能は
  [yolo_detector 詳細設計書](../../yolo_detector/docs/yolo_detector_詳細設計書.md)の
  範囲外でもある。
- `decision_duration` 3 秒・`decision_frame_ratio` 50 %・`confirmation_duration` 10 秒の
  組み合わせが実際の看板通過時間に対して適切かを確認していない。
- 時刻差 3 秒の警告閾値（7.4 節）が実機の時刻同期精度に対して適切かを確認していない。
- 確定封鎖位置が上限なく累積する点（5 章）の長時間走行での影響を確認していない。
- 自己位置の誤差が多重検知抑止の判定へ与える影響を確認していない。

## 10. 影響範囲・未決事項

### 10.1 影響範囲

`/road_blocked` の消費者は `route_follower` と `robot_navigator` である。**状態遷移時のみ
publish する規則**（3.2 節）は `route_follower` の封鎖継続判定が前提にする。

`tc_perception_msgs/PerceptionOverlay` のフィールドを変更する場合、`robot_console` の
重畳処理と `traffic_signal_recognizer` を併せて更新する。

`vision_msgs/Detection2DArray` の構成（class id を文字列で渡す規則）は `yolo_detector` と
対になる。

パラメータの既定値が `yolo_detector/params/default.yaml` にもある（4 章）。

### 10.2 `traffic_signal_recognizer` との対応関係

YOLO 推論層の後段で意味判定を行う**対になるパッケージ**として扱う。

| 項目 | road_blockage_detector | traffic_signal_recognizer |
| --- | --- | --- |
| YOLO 入力 | `/perception/road_blockage/detections` | `/perception/traffic_signal/detections` |
| raw 画像入力 | なし | なし |
| 制御出力 | `/road_blocked` | `/sig_recog` |
| 認識結果 | `/perception/road_blockage/overlay` | `/perception/traffic_signal/overlay` |
| YOLO 生検出画像 | `/perception/road_blockage/detection_image` | `/perception/traffic_signal/detection_image` |

`/perception/*/detection_image` は YOLO 推論結果の確認用である。後段判定結果の確認は
`/perception/*/overlay` を `robot_console` が生画像へ重畳した表示で行う。

### 10.3 未決事項

| 件 | 選択肢 | blocker |
| --- | --- | --- |
| 9.1 節: 自動試験が無い | 判定部分を ROS 非依存モジュールへ切り出して試験する／手動確認に留める（現状） | ではない |
| 5 章: 確定封鎖位置が上限なく累積する | 件数上限または距離による削除を設ける／現状を維持する | ではない |
| 7.4 節: 時刻差の警告閾値がパラメータを持たない | パラメータへ出す／固定のまま文書で明示する（現状） | ではない |
| 4 章: パラメータの既定値が 2 箇所にある | 片方を正として参照に統一する／現状を維持する | ではない。起動経路によって値が変わり得る |
| 1 章: `use_sim_time` 引数を提供しない | 引数を設ける／呼び出し側で時刻基準を揃える前提を明示する（現状） | ではない |
