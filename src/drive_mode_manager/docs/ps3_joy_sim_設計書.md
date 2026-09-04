# PS3 Joy Simulator 設計書

作成日: 2026-05-21

## 1. 文書目的・対象範囲

本書は、`drive_mode_manager` に追加する開発用 `ps3_joy_sim_node` の設計を定義する。対象は PS3 controller 相当の `sensor_msgs/msg/Joy` をキーボード入力と PyQt5 GUI から模擬する機能である。実機 controller、Bluetooth 接続、`joy_node` driver 差異の検証は対象外とする。

本書は、実装前入力資料 `tmp/ps3_joy_sim_design_input.md` の内容を正式文書へ反映した正本である。tmp 文書は長期参照しない。

## 2. 背景・要求・スコープ

`manual_teleop_node` と `drive_cmd_mux_node` は `/joy` を入力として、L1 デッドマン、L1 + PS 長押しによる手動遷移、L1 release による自律復帰を判定する。PS3 controller が手元にない開発環境でもこの経路を確認できるよう、`joy` topic の代替 publisher を用意する。

本ノードの責務は以下である。

- 相対 topic `joy` に `sensor_msgs/msg/Joy` を周期 publish する。
- `l`, `p`, `w`, `s`, `a`, `d`, `space` のキーボード入力を Joy 配列へ変換する。
- GUI 上に publish topic、publish rate、L1/PS、左 stick、axes/buttons、publish count を表示する。
- `/cmd_vel` や `/cmd_vel/manual` は publish せず、速度指令の決定は既存ノードに委ねる。

## 3. 全体構成・アーキテクチャ

代表的な接続は以下である。

```text
ps3_joy_sim_node
  -> joy

manual_teleop_node
  <- joy
  -> cmd_vel/manual

drive_cmd_mux_node
  <- joy
  <- cmd_vel/manual
  <- cmd_vel/autonomous
  -> cmd_vel
  -> drive_mode_status
```

`ps3_joy_sim_node` は実 controller の `joy_node` と同じ topic へ publish する代替入力源である。両者を同時に同じ `joy` topic へ publish しない。

## 4. パッケージ構成・ファイル配置

| ファイル | 役割 |
| --- | --- |
| `drive_mode_manager/ps3_joy_sim_core.py` | キー集合から Joy 配列を生成する ROS 非依存ロジック |
| `drive_mode_manager/ps3_joy_sim_node.py` | PyQt5 GUI と `rclpy` publisher を持つ ROS 2 ノード |
| `launch/ps3_joy_sim.launch.py` | simulator 単独起動用 launch |
| `tests/test_ps3_joy_sim_core.py` | index、符号、同時押し、reset 相当の単体テスト |

通常の `drive_mode_manager.launch.py` には含めない。開発者が明示的に `ps3_joy_sim.launch.py` または `ros2 run drive_mode_manager ps3_joy_sim_node` で起動する。

## 5. 外部インタフェース仕様

| 方向 | Topic | Type | QoS | 内容 |
| --- | --- | --- | --- | --- |
| Publish | `joy` | `sensor_msgs/msg/Joy` | Reliable / Volatile / depth 10 | PS3 controller 相当の入力状態 |

Subscribe、Service、Action は持たない。

## 6. パラメータ・設定仕様

| パラメータ | 型 | 既定値 | 内容 |
| --- | --- | --- | --- |
| `joy_topic` | string | `joy` | Joy publish topic |
| `publish_rate_hz` | double | `20.0` | Joy publish 周期 |
| `num_axes` | int | `6` | `Joy.axes` 配列長 |
| `num_buttons` | int | `17` | `Joy.buttons` 配列長 |
| `left_stick_x_axis` | int | `0` | 左 stick 横軸 index |
| `left_stick_y_axis` | int | `1` | 左 stick 縦軸 index |
| `invert_left_stick_x` | bool | `true` | キーボード左右入力から publish する左 stick 横軸符号を反転する。tc2025 実機 Joy 互換を既定とする |
| `invert_left_stick_y` | bool | `false` | キーボード前後入力から publish する左 stick 縦軸符号を反転する |
| `stick_step` | double | `0.1` | `w`/`s`/`a`/`d` 1 回押下あたりの stick 加算量 |
| `l1_button_index` | int | `4` | L1 button index |
| `ps_button_index` | int | `16` | PS button index |
| `key_l1` | string | `l` | L1 模擬キー |
| `key_ps` | string | `p` | PS 模擬キー |
| `key_stick_forward` | string | `w` | 左 stick 前 |
| `key_stick_backward` | string | `s` | 左 stick 後 |
| `key_stick_left` | string | `a` | 左 stick 左 |
| `key_stick_right` | string | `d` | 左 stick 右 |
| `key_reset` | string | `space` | 全入力 reset |
| `normalize_diagonal_stick` | bool | `true` | stick 保持値をノルム 1.0 以内へ正規化する |
| `cmd_vel_linear_scale` | double | `1.2` | GUI に表示する予測 `cmd_vel` 並進速度 scale |
| `cmd_vel_angular_scale` | double | `1.5` | GUI に表示する予測 `cmd_vel` 角速度 scale |
| `cmd_vel_deadzone` | double | `0.05` | GUI に表示する予測 `cmd_vel` の deadzone |
| `cmd_vel_linear_axis_invert` | bool | `false` | GUI 予測並進速度の符号反転 |
| `cmd_vel_angular_axis_invert` | bool | `false` | GUI 予測角速度の符号反転。通常は `invert_left_stick_x` で Joy 軸を合わせるため変更しない |

既定 index は現行 `manual_teleop_node` と `drive_cmd_mux_node` の設定に合わせる。

## 7. データモデル・内部状態

`ps3_joy_sim_core.py` は `Ps3JoySimConfig` と `Ps3JoyState` を持つ。node は `pressed_keys: set[str]`、`stick_x`、`stick_y`、最新 `Ps3JoyState`、`publish_count` を lock 付きで保持する。GUI thread は key event で L1/PS の押下集合と stick 保持値を更新し、ROS executor thread は timer で最新状態を publish する。

## 8. 処理フロー・状態遷移

起動時にパラメータを読み込み、`joy_topic` へ publisher を作成する。以後、timer callback で押下中キーと stick 保持値から Joy 配列を再計算し、`header.stamp` に node clock を設定して publish する。

| キー | Joy 反映 |
| --- | --- |
| `l` | `buttons[l1_button_index] = 1` |
| `p` | `buttons[ps_button_index] = 1` |
| `w` | `stick_y` へ `stick_step` を加算 |
| `s` | `stick_y` から `stick_step` を減算 |
| `a` | `stick_x` から `stick_step` を減算 |
| `d` | `stick_x` へ `stick_step` を加算 |
| `space` | 押下集合を clear し、`stick_x=0.0`、`stick_y=0.0` に戻す |

L1 + PS 長押しの時間判定は `drive_cmd_mux_node` が行う。本ノードは押下状態と stick 保持値の publish のみ行う。stick 保持値から現在の `manual_teleop_node` 既定値に基づく予測 `cmd_vel` を算出し、GUI に表示するが、`cmd_vel` topic は publish しない。

## 9. QoS・並行性・タイミング設計

`joy` は 20Hz、Reliable / Volatile / depth 10 で publish する。`manual_teleop_node` と `drive_cmd_mux_node` が Joy timeout を持つため、入力変化時だけではなく常に周期 publish する。

PyQt5 の GUI thread と `rclpy` executor thread は分離する。ROS callback から Qt widget は直接更新せず、GUI は `QTimer` で `snapshot()` を読み取って表示する。

## 10. 起動・終了・launch 設計

単独起動用に `ps3_joy_sim.launch.py` を提供する。

```bash
ros2 launch drive_mode_manager ps3_joy_sim.launch.py
```

`drive_mode_manager.launch.py joy_input:=ps3_joy_sim` では `joy_node` の代替入力源として同時起動する。`ps3_joy_sim.launch.py` は simulator 単独確認用として提供する。本物の `joy_node` と同じ `joy` topic へ同時 publish しない。

終了時は追加の停止指令を publish しない。下流の `manual_teleop_node` と `drive_cmd_mux_node` は Joy timeout により安全側へ倒れる。

## 11. エラー処理・ログ・診断

起動時に publish topic と rate を `info` で出す。index が配列長外の場合は該当配列を書き換えず、GUI 表示上の L1/PS 状態は保持するが、publish される buttons には反映されない。

PyQt5 が利用できない環境では GUI node 起動に失敗する。その場合は headless の core pytest と、別手段の Joy publisher による結合確認に切り替える。

## 12. UI・可視化仕様

GUI は Publish Topic、Publish Rate、L1 / PS / L1 + PS、Left Stick X / Y、予測 `cmd_vel` の `v` / `w`、Left Stick View、`axes[]`、`buttons[]`、Publish Count、Focus、Pressed Keys を表示する。

キーボード入力は GUI window に focus がある場合のみ取得される。`w`/`s`/`a`/`d` は押すたびに stick 保持値へ加算または減算され、離しても値を保持する。`space` で stick を neutral に戻す。Focus 表示が OFF の場合は window を選択してから操作する。

## 13. 依存関係・ビルド設定

既存 `drive_mode_manager` の依存 `sensor_msgs` と `python3-pyqt5` を利用する。新しい外部依存は追加しない。`setup.py` の `console_scripts` に `ps3_joy_sim_node` を追加する。

## 14. テスト計画・受け入れ条件

単体テストでは L1/PS index、`w/s/a/d` の累積軸更新、Y 軸反転、斜め入力正規化、予測 `cmd_vel`、reset 相当、index 範囲外時の配列長維持を確認する。

確認コマンドは以下である。

```bash
pytest src/drive_mode_manager/tests
colcon build --symlink-install --packages-select drive_mode_manager
colcon build --packages-select drive_mode_manager
```

ROS 2 実行確認では、`ros2-local-run` スキルに従い、`ypspur_ros2` や実機 driver は起動しない。GUI 目視確認は DISPLAY が利用できるローカル環境でのみ行う。

## 15. 互換性・移行・影響範囲

既存 topic、msg、service、`drive_mode_manager.launch.py` の起動構成は変更しない。`ps3_joy_sim_node` は代替 Joy 入力源として追加されるだけであり、通常運用へは影響しない。

## 16. 未決事項・今後の拡張

- 実 controller の L1 / PS index と stick 符号は実機で別途確認する。
- PS ボタンが実機 `/joy` で安定取得できない場合は、mux 側の手動遷移 trigger を別操作へ変更する。
- Start / Select の模擬は現行運用で使わないため追加しない。
- GUI から `/drive_mode_status` を補助表示する拡張は、必要になった時点で検討する。

## 17. 改版履歴

| 版 | 日付 | 変更概要 |
| --- | --- | --- |
| 0.2 | 2026-05-22 | stick 累積入力、reset neutral、GUI の予測 `cmd_vel` 表示、斜め正規化既定有効を反映した |
| 0.1 | 2026-05-21 | 初版。tmp 設計インプットを正式設計へ反映し、実装仕様を定義した |
