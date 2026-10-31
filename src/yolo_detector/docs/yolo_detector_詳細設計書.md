# yolo_detector 詳細設計書

## 1. 目的・スコープ

本書は `yolo_detector` パッケージの内部設計を定める。対象ノードは 3 つである。

| ノード | 用途 |
| --- | --- |
| `yolo_node` | PyTorch の `.pt` モデルで推論する |
| `yolo_ncnn_node` | NCNN 形式モデルで推論する（CPU 推論を軽量化したい用途） |
| `camera_simulator_node` | 静止画を画像 topic として配信する検証補助 |

トピック名、型、パラメータの一覧と起動方法は [README](../README.md) を正とする。
本書はそれを再掲せず、**モデルの解決順序、推論の有効化と多重実行の抑止、出力の構成規則、
2 ノードの差分、制約**を定める。

カメラ画像を YOLO モデルへ入力し、検出結果を `vision_msgs/Detection2DArray` と重畳画像
として配信する。**本パッケージの責務は画像推論層に限る。** 経路封鎖看板であるか、信号が
GO / STOP のどちらかといった意味判定は下流の `road_blockage_detector` と
`traffic_signal_recognizer` が担う。

非スコープは、意味判定、停止の指示、カメラの較正、モデルの学習である。モデルファイルの
変換は同梱スクリプトで行うが、ノードからは呼ばない。

記載内容はすべて実装済みである。**自動試験を持たない**（9.1 節）。

## 2. 構成とファイル配置

| パス | 用途 |
| --- | --- |
| `yolo_detector/yolo_node.py` | PyTorch 版（約 350 行） |
| `yolo_detector/yolo_ncnn_node.py` | NCNN 版（約 345 行） |
| `yolo_detector/camera_simulator_node.py` | 静止画配信（約 150 行） |
| `launch/yolo_node.launch.py` | PyTorch 版の単体起動 |
| `launch/yolo_ncnn_node.launch.py` | NCNN 版の単体起動 |
| `models/best.pt`、`models/traffic_signal_best.pt` | 学習済み PyTorch モデル |
| `models/best_ncnn_model/` | NCNN 形式へ変換したモデルのディレクトリ |
| `models/README.md` | モデルの由来 |
| `scripts/convert_to_ncnn.py` | `.pt` から NCNN 形式への変換（オフラインで手動実行） |

**2 つの推論ノードは共通の基底クラスを持たず、構造の同じ別実装である。** パラメータ名、
出力の構成、ログの形式は揃えてあるが、コードは共有していない。差分は 7.4 節に示す。

**ROS 非依存のロジックへ切り出していない。** 推論と出力生成はノードのメソッドとして
実装する。

`models/` のモデルファイルは管理対象である。変換後の NCNN ディレクトリも含める。

## 3. 外部インタフェース仕様

一覧は [README](../README.md) を正とする。本章は一覧に現れない取り扱いと、QoS の選定
理由を定める。

### 3.1 QoS

| 対象 | 設定 | 選定理由 |
| --- | --- | --- |
| 入力画像の購読 | 既定プロファイル / depth 10 | — |
| 有効化フラグの購読 | 既定プロファイル / depth 10 | — |
| 検出結果の配信 | 既定プロファイル / depth 10 | — |
| 重畳画像の配信 | 既定プロファイル / depth 10 | — |

**すべて既定の RELIABLE である。** BEST_EFFORT を使わない。画像の購読側が BEST_EFFORT で
配信するカメラと組み合わせる場合、QoS 非互換となり購読が成立しない。`usb_cam` は
RELIABLE で配信するため現構成では成立する。

### 3.2 検出結果の構成

`Detection2DArray` の `header` は**入力画像の header を引き継ぐ。** 推論時刻ではなく
撮影時刻を保つため、下流が画像と検出の対応を取れる。

各検出は 1 つの仮説のみを持つ。

| フィールド | 値 |
| --- | --- |
| `results[0].hypothesis.class_id` | class id を**文字列化**した値 |
| `results[0].hypothesis.score` | confidence |
| `bbox.center.position` | 矩形の中心（画素座標） |
| `bbox.center.theta` | 0.0（回転を扱わない） |
| `bbox.size_x`、`bbox.size_y` | 矩形の幅と高さ（画素） |

**class name は検出結果に入らない。** 重畳画像の描画とログにのみ使う。下流は class id
の文字列で判定する。

`Detection2D.header` にも同じ header を入れる。

### 3.3 重畳画像

モデルが返した元画像を複製し、検出矩形とクラス名・スコアを描いて配信する。header は
入力画像のものを引き継ぐ。

**検出が 0 件でも配信する。** 元画像のまま出る。推論結果が得られなかった場合（結果が
空）は空の配列から画像を作るため、**不正なサイズの画像を配信し得る。**

### 3.4 推論の有効化

`enabled_topic` が空文字の場合は常時推論する。空でない場合はその topic を購読し、
**受信値が `enabled_value` と一致する間だけ推論する。** 一致しない値を受けると無効に
なる。

起動直後の状態は `start_enabled` で決める。**有効化 topic を購読する構成でも、最初の
受信までは `start_enabled` の値が有効である。**

信号認識のインスタンスは `/recog_flag` で制御し、信号停止地点でのみ推論する
（`route_follower` が 1 を出す区間）。

### 3.5 画面・可視化

重畳画像を topic として配信する。ノード自身はウィンドウを開かない。`robot_console` は
この重畳画像を使わず、**検出結果から自前で重畳する**（
[robot_console アーキテクチャ詳細設計書](../../robot_console/docs/robot_console_gui_architecture_design.md)
7.1 節）。本パッケージの重畳画像は `rqt_image_view` などでの確認用である。

## 4. 設定の所在

一覧と既定値は [README](../README.md) を正とする。本章はどの設定がどの処理に効くかと、
ノード別の範囲を定める。閾値の値と理由は 7 章に置く。

| パラメータ | 効く処理 |
| --- | --- |
| `model_path` | モデルの解決（7.1 節） |
| `image_topic` | 入力画像の購読 |
| `detection_interval` | 推論タイマーの周期（8.1 節） |
| `confidence_threshold` | 推論時の confidence 下限（7.2 節） |
| `detections_topic`、`annotated_image_topic` | 出力先 |
| `enabled_topic`、`enabled_value`、`start_enabled` | 推論の有効化（3.4 節） |

ノード別のパラメータは次のとおりである。

| パラメータ | 対象 | 効く処理 |
| --- | --- | --- |
| `image_size` | `yolo_node` | 推論時の入力画像サイズ（7.4 節） |
| `class_names` | `yolo_ncnn_node` | class name の代替（7.3 節） |

`camera_simulator_node` は `frame_image_path`、`frame_width`、`frame_height`、
`frame_ratio` を持つ。静止画の読み込みとリサイズに使う。

**同梱の `params/default.yaml` は本パッケージのノード向けではない。** 中身は
`road_blockage_detector` のパラメータであり、推論ノードの値は launch 引数または上位の
launch から与える。

## 5. データモデル・内部状態

| 状態 | 初期値 | 更新契機 | クリア条件 |
| --- | --- | --- | --- |
| モデル | 起動時に読み込む | なし | なし |
| 最新画像 | なし | 画像受信ごとに置き換え | なし |
| 最新 header | なし | 同上 | なし |
| 推論の有効状態 | `start_enabled` | 有効化 topic の受信 | なし |

**保持するのは最新の 1 フレームのみである。** 推論周期より速く画像が届く場合、間のフレーム
は破棄される。キューを持たない。

**入力の鮮度を判定しない。** 画像の受信が途絶しても、最後に受けたフレームに対して推論を
続け、同じ検出結果を配信し続ける。**下流は検出結果の header（撮影時刻）で古さを判定する
必要がある。**

## 6. 処理フローと異常時の挙動

### 6.1 画像受信時

1. `cv_bridge` で `bgr8` へ変換する。
2. header を複製する。
3. 画像ロックを取り、最新画像と header を置き換える。

### 6.2 推論タイマー

1. 推論が無効なら即座に戻る。**最新画像の保持は続く。**
2. タイマーロックを**非ブロッキングで取得**する。取得できなければ戻る（8.2 節）。
3. 最新画像と header を複製する。未受信なら戻る。
4. 推論を実行し、所要時間を測る。
5. 検出結果と重畳画像を生成する。
6. 検出結果を配信する。
7. 重畳画像を `bgr8` へ変換し、header を入力のものに置き換えて配信する。
8. 検出内容と推論時間をログへ出す。
9. ロックを解放する（`finally` で必ず行う）。

### 6.3 異常時の挙動

| 事象 | 挙動 |
| --- | --- |
| モデル読み込みの失敗 | **error を出して例外を再送出する。** ノードは起動しない |
| `model_path` が空 | warn を出してノード内部の代替へ落とす。`yolo_node` の代替は存在しないファイルを指すため、続く読み込みが失敗する（7.1 節） |
| 画像変換の失敗 | error を出し、**そのフレームの処理を飛ばす。** 保持中の画像は更新しない |
| 画像が未受信 | 推論を行わず戻る。何も配信しない |
| 推論中に次のタイマーが来る | 非ブロッキングロックで**その回を飛ばす** |
| 推論結果が空 | 空配列から重畳画像を作る。不正なサイズの画像を配信し得る（3.3 節） |
| class name が取得できない | 代替名を使う（7.3 節） |

**推論自体の例外は捕捉していない。** モデルが実行時に例外を投げた場合、タイマー
コールバックの外へ伝播する。ロックは `finally` で解放されるが、以降の周期でも同じ例外が
起きる。

### 6.4 終了処理

ノードを破棄して ROS をシャットダウンする。**終了時に安全値を配信しない。** 最後の検出
結果が下流に残る。

`DiagnosticReporter` を使わない。**健全性監視の対象に入っていない。**

### 6.5 ログ

起動時に `info` で入力 topic、推論周期、confidence 閾値、class 名、出力 topic を出す。
有効化制御を使う場合はその設定も出す。

**推論ごとに検出内容と推論時間を `info` で出す。** 間引きを行わないため、推論周期と同じ
頻度でログが出る。

モデル読み込みの失敗と画像変換の失敗は `error`、`model_path` 未指定は `warn` を使う。
`debug` と `fatal` は使わない。

## 7. 判定ロジックと閾値

### 7.1 モデルの解決順序

モデルの既定値は **launch とノード内部の 2 箇所にあり、指す先が違う。**

| 与え方 | `yolo_node` | `yolo_ncnn_node` |
| --- | --- | --- |
| launch の引数既定 | share の `models/best.pt` | share の `models/best_ncnn_model` |
| ノード内部の代替（`model_path` が空のとき） | share の `models/yolo11n.pt` → 無ければソースツリーの `models/yolo11n.pt` | share の `models/best_ncnn_model` |

launch は必ず `model_path` を渡すため、`ros2 launch` 経由ではノード内部の代替に到達
しない。上位の `road_blockage_perception*.launch.py` と
`traffic_signal_perception.launch.py` も `model_path` を引数として渡す。

**`ros2 run` でパラメータを与えない場合、`yolo_node` は存在しないファイルを指す。**
`models/` に `yolo11n.pt` は置いていない。この場合 warn を出した後にモデル読み込みが
失敗し、**ノードは起動しない**（6.3 節）。`yolo_ncnn_node` の代替は同梱の NCNN モデルを
指すため起動する。

既定モデルは用途別に学習したものと同じではない場合がある。意図したモデルで動いているかを
起動ログで確認する必要がある。

### 7.2 confidence 閾値

推論の呼び出し時に `confidence_threshold`（既定 0.5）を渡す。**閾値による絞り込みは
モデル側で行い、ノードは受け取った検出をすべて出力する。** ノード側で再度フィルタしない。

### 7.3 class name の解決

モデルが返す名前表から class id で引く。表が辞書なら `get` で引き、見つからなければ
`class_<id>` を返す。表が配列なら添字で引く。

`yolo_ncnn_node` は、取得できない場合に `class_names` パラメータ（既定 `['item']`）の
対応する要素へ落とす。**NCNN 形式のモデルは名前表を持たないことがあるため**である。

### 7.4 2 ノードの差分

| 項目 | `yolo_node` | `yolo_ncnn_node` |
| --- | --- | --- |
| モデル形式 | PyTorch `.pt` | NCNN ディレクトリ |
| 推論デバイス | **`cpu` を明示指定** | 指定しない（モデル側の既定） |
| 入力サイズ | `image_size`（既定 320）を明示指定 | 指定しない（モデル側の既定） |
| class name の代替 | 名前表のみ | `class_names` パラメータへ落とす |
| 既定モデルの解決 | 2 段（share → ソース） | 1 段（share のみ） |

**`yolo_node` は GPU を使わない。** `cpu` を固定で指定するため、GPU が利用できる環境でも
CPU で推論する。

その他の処理（有効化、ロック、出力構成、重畳、ログ）は同じである。

### 7.5 静止画配信のリサイズ

`camera_simulator_node` は静止画を読み込み、`frame_width` / `frame_height` が指定されて
いればその大きさへ合わせる。**縦横比を保つレターボックス方式**で余白を足す。指定が無い
場合は元の大きさで配信する。`frame_ratio` で配信周期を決める。

配信する画像のパスは、起動時のパラメータと、実行中の文字列 topic の両方で指定できる。
`robot_console` の静止画像入力がこの topic を使う。

## 8. 並行性・周期・起動・依存関係

### 8.1 周期

| 項目 | 値 |
| --- | --- |
| 推論タイマー | `detection_interval`（既定 0.5 秒） |
| 静止画の配信 | `frame_ratio`（既定 10.0 の逆数） |
| ログ出力 | 推論ごと（間引きなし） |

### 8.2 並行性

**`ReentrantCallbackGroup` と `MultiThreadedExecutor` を使う。** 推論が周期より長くかかる
場合でも画像の購読を続けるためである。

ロックを 2 つ持つ。

| ロック | 保護対象 | 取得方法 |
| --- | --- | --- |
| 画像ロック | 最新画像と header | 通常の取得（短い区間） |
| タイマーロック | 推論処理全体 | **非ブロッキング。取得できなければその回を飛ばす** |

タイマーロックを非ブロッキングにするのは、推論が周期より遅い場合に**タイマーコール
バックが積み上がることを防ぐ**ためである。結果として、実際の推論頻度は
`detection_interval` と推論時間の遅い側で決まる。

画像は推論の直前に**ディープコピー**する。推論中に新しいフレームが届いても、処理中の
画像が書き換わらないようにするためである。

### 8.3 起動

entry point は 3 ノード分である。launch の引数は [README](../README.md) を正とする。

`yolo_node.launch.py` と `yolo_ncnn_node.launch.py` は単体起動用である。実際の認識系は
用途別に複数インスタンスを起動する（7.4 節の差分に従って選ぶ）。

| 用途 | 使うノード | 出力 topic | 下流 |
| --- | --- | --- | --- |
| 経路封鎖看板 | `yolo_ncnn_node` | `/perception/road_blockage/detections` | `road_blockage_detector` |
| 信号認識 | `yolo_node` | `/perception/traffic_signal/detections` | `traffic_signal_recognizer` |

**単一ノードで class を混在処理しない。** モデル、推論頻度、起動条件、対象クラス、下流
判定が用途ごとに異なるためである。

認識系全体は下流パッケージの launch から起動する。経路封鎖検知は
`road_blockage_detector` の `road_blockage_perception.launch.py` または
`road_blockage_perception_yolo.launch.py`、信号認識は `traffic_signal_recognizer` の
`traffic_signal_perception.launch.py` である。

起動時にモデルを読み込む。**読み込みに失敗すると起動しない**（6.3 節）。

### 8.4 依存関係

`ament_python` でビルドする。依存は `rclpy`、`sensor_msgs`、`std_msgs`、`vision_msgs`、
`cv_bridge` である。

Python パッケージとして `ultralytics`、`numpy`、`opencv-python` を使う。
**`ultralytics` は pip で導入し、`requirements.txt` で管理する。** NCNN 形式の推論も
`ultralytics` 経由で行い、NCNN のライブラリを直接呼ばない。

モデルファイルは `setup.py` で share ディレクトリへインストールする。**`.pt` と NCNN
ディレクトリの容量がビルド成果物に乗る。**

## 9. 確認方法と未確認範囲

### 9.1 自動確認

**自動試験を持たない。** `tests/` ディレクトリが無い。

試験を書く場合、推論を含まない部分（出力の構成、class name の解決、有効化の判定、
レターボックスのリサイズ）は `ultralytics` を差し替えれば ROS 非依存で検証できる構造で
ある。推論そのものはモデルファイルを要するため、自動確認の対象にしにくい。

### 9.2 手動で確認する範囲

- `camera_simulator_node` で静止画を配信し、検出矩形とスコアが重畳されること
- `model_path` を空にして `ros2 run` したときの挙動（7.1 節）。`yolo_node` は起動に失敗する
- `confidence_threshold` を変えたときの検出数の変化
- 有効化 topic による推論の開始・停止（`/recog_flag` 連動を含む）
- 推論周期より速い画像入力でフレームが破棄されること
- 推論が周期より遅い場合にタイマーが積み上がらないこと（8.2 節）
- 検出結果の header が入力画像の header と一致すること
- 下流（`road_blockage_detector`、`traffic_signal_recognizer`）での判定まで通ること

### 9.3 未確認範囲

- **実機カメラでの検出精度を確認していない。** モデルの性能は本書の範囲外である。
- 実機 CPU での推論時間を測定していない。`detection_interval` が実際に守られるかは
  未確認である（8.2 節）。
- 画像の受信途絶時に古い検出を配信し続ける点（5 章）の下流への影響を確認していない。
- 推論結果が空のときに不正なサイズの画像を配信し得る点（3.3 節）を再現確認していない。
- 推論中の例外が伝播する挙動（6.3 節）を確認していない。
- NCNN 形式と PyTorch 形式で同じ画像に対する検出が一致するかを確認していない。
- `models/` の各モデルの学習条件と対象クラスは `models/README.md` の範囲であり、本書では
  扱わない。

## 10. 影響範囲・未決事項

### 10.1 影響範囲

`vision_msgs/Detection2DArray` の構成（3.2 節）は `road_blockage_detector` と
`traffic_signal_recognizer` が前提にする。**class name を入れず class id の文字列で渡す
規則**を変える場合は両パッケージを同時に更新する。

header を入力画像から引き継ぐ規則は、下流が画像と検出の時刻差を判定する根拠である
（`robot_console` の重畳で時刻差の大きい枠を破線にする処理が依存する）。

`models/` のモデルを差し替える場合、class id とクラスの対応が変わるため下流の判定を
併せて確認する。

`ultralytics` のバージョンは `requirements.txt` で管理する。推論 API が変わると 2 つの
ノードを同時に修正する必要がある。

### 10.2 未決事項

| 件 | 選択肢 | blocker |
| --- | --- | --- |
| 9.1 節: 自動試験が無い | 推論を差し替えて出力構成・class name 解決・有効化判定を試験する／推論を含むため手動確認に留める（現状） | ではない |
| 2 章: 2 ノードが別実装である | 共通部分を基底クラスまたは ROS 非依存モジュールへ切り出す／差分が小さいため現状を維持する | ではない。片方だけ修正すると挙動が分かれる |
| 3.3 節: 結果が空のとき不正な画像を配信し得る | 空の場合は配信しない／入力画像をそのまま配信する | ではない |
| 6.3 節: 推論の例外を捕捉していない | 捕捉してそのフレームを飛ばす／現状どおり伝播させる | ではない |
| 7.4 節: `yolo_node` が CPU 固定である | デバイスをパラメータへ出す／CPU 固定を前提として明示する（現状） | ではない |
| 7.1 節: `yolo_node` のノード内部の代替が存在しないファイルを指す | 代替を同梱モデルへ向ける／`model_path` 未指定を起動時に拒否する／代替を削って launch 既定のみとする | ではない。launch 経由では到達しない |
| 6.5 節: ログを間引いていない | 推論ごとの `info` を間引く／`debug` へ落とす | ではない |
