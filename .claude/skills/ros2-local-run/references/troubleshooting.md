# よくあるつまずきと対処

このワークスペースで実際に ROS 2 ノードを起動して確認する際に繰り返し発生した詰まりどころと
対処を整理する。手順そのものは SKILL.md および `package-checks.md` /
`integration-check.md` を参照し、本ファイルは「うまくいかないときにどう切り分けるか」に絞る。

## ビルド直後の `executable not found`

`colcon build` が成功していても、`--symlink-install` なしビルドの直後や、対象パッケージだけを
ビルドし忘れた場合、`ros2 launch` / `ros2 run` が `executable '<name>' not found` のように
失敗することがある。

- `ros2 launch` の前に、対象パッケージを再ビルドしたか、`install/<package_name>/` 配下に
  対象の実行ファイルが存在するかを確認する。
- 解消しない場合は `build-and-test` スキルのクリーン手順（パッケージ単位 → ワークスペース
  全体の順）に従う。

## GUI 評価が使えるかどうかの判定

`robot_console` や `gui_route_stack_eval.py` のような tkinter/PyQt5 GUI を伴う確認は、
`DISPLAY` が使える環境かどうかで手順が変わる。判定してから GUI あり/なしを選ぶ。

```bash
printf 'DISPLAY=%s WAYLAND_DISPLAY=%s XDG_SESSION_TYPE=%s QT_QPA_PLATFORM=%s\n' \
  "$DISPLAY" "$WAYLAND_DISPLAY" "$XDG_SESSION_TYPE" "$QT_QPA_PLATFORM"
```

- `DISPLAY` （または `WAYLAND_DISPLAY`）が空でなければ、ローカルデスクトップまたは X11
  転送が有効な可能性が高い。この場合は GUI あり評価（`gui_route_stack_eval.py` 等）を
  優先する。実際の運用でも GUI あり評価の方が圧倒的に多く使われていた。
- 空の場合、または SSH 接続で X11 転送を有効にしていない場合は headless 評価
  (`headless_route_stack_eval.py`) に切り替える。

## Gazebo / GPU 描画確認（LiDAR・カメラ）

`obstacle_route_sim` で LiDAR やカメラのセンサ描画・認識結果を確認する際、GPU レンダリングが
必要になり、環境によっては素の起動では描画が正しく出ないことがある。

`src/obstacle_route_sim/README.md` の正式な対処は「`render`/`video` group 追加後にログアウト・
ログインしてから確認する」である。まずこちらを優先する。ログインし直せないセッション
（このエージェントの実行環境など）で一時的に group 権限だけ得たい場合は、以下のように
`sg` でコマンドを包む方法が実際に有効だった。

```bash
sg render -c "sg video -c 'bash -lc \"cd <workspace> && source install/setup.bash && \
  timeout 18s ros2 launch obstacle_route_sim sim_obstacle_route.launch.py \
  road_type:=crank enable_pylons:=true start_gazebo_gui:=false\"'"
```

`/dev/dri/renderD128` などのレンダーノードに対する read/write 権限があるかも合わせて
確認する（`ls -l /dev/dri/`）。

上記でも描画が掴めない場合、以下の環境変数を 1 つずつ切り替えて切り分けた実績がある。

```bash
export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
export EGL_PLATFORM=surfaceless
# 特定 GPU を明示したい場合
export DRI_PRIME=1
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/50_mesa.json
```

`LIBGL_ALWAYS_SOFTWARE=1`（ソフトウェアレンダリングへの強制フォールバック）は
`src/obstacle_route_sim/README.md` が明示的に「Gazebo/Ogre2 の安定性を落とす場合があるため
既定では使用しない」としている。上記をすべて試しても描画が掴めない場合の最終手段としてのみ
検討し、使った場合はその旨を作業報告に明記する。

どのパターンでも描画が確認できない場合は、Gazebo GUI なし (`start_gazebo_gui:=false`)
でノード間の topic 疎通だけを確認し、描画自体の確認は未確認事項として報告する。

## 停止処理でプロセスが残る場合のエスカレーション

`ros2 launch` や Gazebo 関連プロセスは `SIGINT` だけでは終了しないことがある。以下の順で
エスカレーションする。

1. 通常の停止（`Ctrl-C` 相当、または評価ツールの `request_stop_all()` 相当）を行う。
2. 数秒待ってから残存確認する。

   ```bash
   pgrep -af "gz sim|ros2 launch|parameter_bridge|robot_console|route_planner|route_manager|route_follower|robot_navigator|robot_simulator"
   ```

3. まだ残っている場合は該当プロセスに `SIGINT` を送る。

   ```bash
   pkill -INT -f 'gz sim server'
   sleep 1
   ```

4. それでも残る場合のみ、対象 PID を明示して `kill -9` にエスカレーションする。broad な
   `pkill -9` ではなく、`pgrep -af` で確認した具体的な PID を指定する。

   ```bash
   kill -9 <pid>
   ```

5. 最後に `ros2 node list` / `ros2 topic list` / `pgrep -af` で本当に何も残っていないことを
   確認し、エスカレーションした事実を作業報告に明記する。
