# yolo_detector パッケージ README

## 概要
`yolo_detector` は、カメラ画像を YOLO モデルへ入力し、検出結果を
`vision_msgs/msg/Detection2DArray` と重畳画像として配信する ROS 2 パッケージです。
tc2026 では本パッケージを YOLO 推論層に限定し、経路封鎖や信号 GO/STOP などの
意味判定は下流の `road_blockage_detector` と `traffic_signal_recognizer` が担当します。

## 主な機能
- `/usb_cam/image_raw` などの `sensor_msgs/msg/Image` を購読し、YOLO 推論を実行。
- PyTorch `.pt` モデル用の `yolo_node` と、NCNN モデル用の `yolo_ncnn_node` を提供。
- `detections_topic` と `annotated_image_topic` を launch 引数で切り替え可能。
- `enabled_topic` による推論有効化制御に対応。信号認識では `/recog_flag==1` の間だけ推論できます。
- 静止画を `/usb_cam/image_raw` へ publish する検証補助ノード `camera_simulator_node` を提供。

## ノード構成
| 実行ファイル | 用途 |
| --- | --- |
| `yolo_node` | PyTorch `.pt` モデルを使用する YOLO 推論ノード。 |
| `yolo_ncnn_node` | NCNN 形式モデルを使用する YOLO 推論ノード。CPU 推論の軽量化用途を想定します。 |
| `camera_simulator_node` | 指定した静止画を `/usb_cam/image_raw` へ周期 publish する補助ノード。 |

## 起動方法
### PyTorch 版 YOLO
```bash
ros2 launch yolo_detector yolo_node.launch.py \
  image_topic:=/usb_cam/image_raw \
  detection_interval:=0.5 \
  image_size:=320 \
  confidence_threshold:=0.5
```

### NCNN 版 YOLO
```bash
ros2 launch yolo_detector yolo_ncnn_node.launch.py \
  image_topic:=/usb_cam/image_raw \
  detection_interval:=0.5 \
  confidence_threshold:=0.5
```

### 静止画配信ノード
`camera_simulator_node` は `frame_image_path` の画像を読み込み、`/usb_cam/image_raw` へ publish します。
出力 topic は固定です。

```bash
ros2 launch yolo_detector camera_simulator_node.launch.py \
  frame_image_path:=<image_path> \
  frame_width:=640 \
  frame_height:=480 \
  frame_ratio:=10.0
```

## yolo_detector 単体での動作確認
`yolo_detector` 付属の launch だけを使い、YOLO 推論結果の publish を確認します。
既定の付属モデルを使う場合は `model_path` の指定を省略できます。
別モデルで確認する場合だけ `model_path` を指定してください。

### カメラ映像の topic がすでに流れている場合
1. 入力画像 topic が publish されていることを確認します。

```bash
ros2 topic hz /usb_cam/image_raw
```

2. YOLO ノードを起動します。NCNN モデルを使う場合は以下を実行します。

```bash
ros2 launch yolo_detector yolo_ncnn_node.launch.py \
  image_topic:=/usb_cam/image_raw
```

PyTorch モデルを使う場合は以下を実行します。

```bash
ros2 launch yolo_detector yolo_node.launch.py \
  image_topic:=/usb_cam/image_raw
```

3. 出力 topic を確認します。

```bash
ros2 topic echo /yolo_detector/detections --once
ros2 topic hz /yolo_detector/detection_image
```

4. 画像を確認する場合は `rqt_image_view` などで `/yolo_detector/detection_image` を表示します。

### 自分で静止画を使って publish する場合
1. 静止画配信ノードを起動します。

```bash
ros2 launch yolo_detector camera_simulator_node.launch.py \
  frame_image_path:=<image_path> \
  frame_width:=640 \
  frame_height:=480 \
  frame_ratio:=10.0
```

2. 別ターミナルで YOLO ノードを起動します。

```bash
ros2 launch yolo_detector yolo_ncnn_node.launch.py \
  image_topic:=/usb_cam/image_raw
```

PyTorch モデルを使う場合は `yolo_node.launch.py` と `.pt` モデルを指定します。

```bash
ros2 launch yolo_detector yolo_node.launch.py \
  image_topic:=/usb_cam/image_raw
```

3. `/yolo_detector/detections` と `/yolo_detector/detection_image` が publish されることを確認します。

```bash
ros2 topic echo /yolo_detector/detections --once
ros2 topic hz /yolo_detector/detection_image
```

## 認識系パッケージとの組み合わせ
経路封鎖と信号認識では、モデル、出力 topic、推論有効化条件、下流判定が異なります。
運用時は下流パッケージ側の統合 launch から用途別 `yolo_detector` インスタンスを起動します。

```bash
# 経路封鎖検知: NCNN版 YOLO + road_blockage_detector
ros2 launch road_blockage_detector road_blockage_perception.launch.py

# 経路封鎖検知: PyTorch版 YOLO + road_blockage_detector
ros2 launch road_blockage_detector road_blockage_perception_yolo.launch.py

# 信号認識: PyTorch版 YOLO + traffic_signal_recognizer
ros2 launch traffic_signal_recognizer traffic_signal_perception.launch.py
```

## ROS インタフェース
### Subscribe
| パラメータ | 既定値 | 型 | 説明 |
| --- | --- | --- | --- |
| `image_topic` | `/usb_cam/image_raw` | `sensor_msgs/msg/Image` | 推論対象の入力画像。 |
| `enabled_topic` | 空文字 | `std_msgs/msg/Int32` | 指定時、`enabled_value` と一致する間だけ推論を有効化。 |

### Publish
| パラメータ | 既定値 | 型 | 説明 |
| --- | --- | --- | --- |
| `detections_topic` | `yolo_detector/detections` | `vision_msgs/msg/Detection2DArray` | YOLO 検出結果。 |
| `annotated_image_topic` | `yolo_detector/detection_image` | `sensor_msgs/msg/Image` | 検出矩形とスコアを重畳した画像。 |

## パラメータ
| 名称 | 既定値 | 対象 | 説明 |
| --- | --- | --- | --- |
| `model_path` | launch ごとに設定 | 共通 | PyTorch `.pt` ファイル、または NCNN モデルディレクトリ。 |
| `image_topic` | `/usb_cam/image_raw` | 共通 | 購読する画像 topic。 |
| `detection_interval` | `0.5` | 共通 | 推論タイマー周期 [秒]。 |
| `confidence_threshold` | `0.5` | 共通 | 検出 confidence の下限。 |
| `detections_topic` | `yolo_detector/detections` | 共通 | `Detection2DArray` の出力 topic。 |
| `annotated_image_topic` | `yolo_detector/detection_image` | 共通 | 検出重畳画像の出力 topic。 |
| `enabled_topic` | 空文字 | 共通 | 推論有効化フラグの入力 topic。空文字なら常時有効。 |
| `enabled_value` | `1` | 共通 | 推論を有効化する `Int32` 値。 |
| `start_enabled` | `true` | 共通 | 起動直後に推論を有効にするか。 |
| `image_size` | `320` | PyTorch | 推論時の入力画像サイズ。 |
| `class_names` | `['item']` | NCNN | NCNN モデルの class id と対応する class name。 |

## モデル配置
モデルは `models/` 配下に配置します。`setup.py` は `models/` を
`install/yolo_detector/share/yolo_detector/models/` 配下へインストールします。
付属 launch の既定値は `FindPackageShare('yolo_detector')` から install/share 配下の
モデルパスを組み立てるため、既定モデル名を使う場合は `model_path` を指定する必要はありません。

```bash
cp <model_file>.pt <ros2_ws>/src/yolo_detector/models/
```

launch 引数で `model_path` を上書きする場合、指定した文字列はそのままノードへ渡されます。
相対パスは「実行時のカレントディレクトリ」基準で解釈されるため、パッケージ相対パスとしては
扱われません。付属モデル以外を指定する場合は、実行場所に依存しないパスを指定してください。

`ros2 run` で `model_path` を与えずに起動した場合は launch の既定値が効かず、ノード内部の
代替パスが使われます。`yolo_node` の代替は `models/yolo11n.pt` を指しますが同梱して
いないため、モデル読み込みに失敗して起動しません。`ros2 run` で使う場合は `model_path` を
明示してください。

NCNN 形式へ変換する場合は、`scripts/convert_to_ncnn.py` を使用します。

```bash
cd <ros2_ws>/src/yolo_detector
python3 scripts/convert_to_ncnn.py models/best.pt
```

## 開発・ビルド
```bash
cd <ros2_ws>
colcon build --symlink-install --packages-select yolo_detector
colcon build --packages-select yolo_detector
```

## トラブルシューティング
- モデル読み込みに失敗する場合は、`model_path` が実在する `.pt` ファイルまたは NCNN モデルディレクトリを指しているか確認してください。
- 静止画入力で何も publish されない場合は、`frame_image_path` の画像を `cv2.imread()` で読み込めるか確認してください。
- 信号認識用の統合 launch では `/recog_flag==1` の間だけ YOLO 推論が有効です。単体確認では `start_enabled:=true`、統合確認では `/recog_flag` の publish 状態を確認してください。
