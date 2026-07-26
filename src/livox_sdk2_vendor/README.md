# livox_sdk2_vendor

Livox-SDK2 を colcon の install 空間へ配置する vendor パッケージ。
`livox_ros_driver2` が SDK を解決できるようにすることだけを目的とし、ノードは持たない。

## 解決している問題

### 1. Livox-SDK2 が GCC 13 でビルドできない

upstream の Livox-SDK2 は `sdk_core/comm/define.h` と
`sdk_core/logger_handler/file_manager.h` が `<cstdint>` を include せずに
`std::uint8_t` / `std::uint16_t` を使用している。推移的 include が厳格化された
GCC 13 (Ubuntu 24.04 / ROS 2 Jazzy) では以下で停止する。

```
sdk_core/comm/define.h:248:8: error:
  'uint8_t' in namespace 'std' does not name a type
```

`third_party/patches/0001-add-cstdint-include-for-gcc13.patch` で解消する。
出典は upstream PR #111 で、2026-07-26 時点で未マージである。同種の PR に
#83 #85 #91 #99 #108 #121 #126 #131 があるがいずれも未マージであり、
upstream 側での解決は期待できない。

### 2. SDK の導入に sudo が必要

upstream の手順は `/usr/local` へ `sudo make install` する前提で、
`livox_ros_driver2` 側も `find_library(... /usr/local/lib REQUIRED)` と
探索先を固定している。

本パッケージは SDK を colcon の install 空間 (`install/livox_sdk2_vendor/`) へ
配置する。`livox_ros_driver2` が本パッケージに依存することで
`CMAKE_PREFIX_PATH` に prefix が載り、`find_library` が解決される。
結果として sudo が不要になる。

依存関係の宣言はワークスペース直下の `colcon.meta` で行う。submodule である
`livox_ros_driver2` の `package.xml` は書き換えない。

## 構成

```text
src/livox_sdk2_vendor/
├── CMakeLists.txt
├── package.xml
├── README.md
└── third_party/
    ├── Livox-SDK2/          submodule (Livox-SDK/Livox-SDK2)
    └── patches/
        └── 0001-add-cstdint-include-for-gcc13.patch
```

## パッチ適用の方針

`src/ypspur_ros2` が yp-spur に対して行っている方式に揃えている。

- submodule の作業ツリーは変更しない。
- ビルドディレクトリへコピーしたうえでパッチを適用する。
- 適用は冪等であり、`git apply --check` と `--reverse --check` で
  未適用・適用済みを判定する。どちらでもない場合は不整合として停止する。

サンプル (`samples/`) は実機運用に不要なため、`sdk_core` のみをビルドする。

## install される成果物

| パス | 内容 |
| --- | --- |
| `lib/liblivox_lidar_sdk_shared.so` | 共有ライブラリ。`livox_ros_driver2` がリンクする |
| `lib/liblivox_lidar_sdk_static.a` | 静的ライブラリ |
| `include/livox_lidar_api.h` ほか 3 点 | 公開ヘッダ |

## 更新手順

Livox-SDK2 を更新する場合は、パッチが当たるかを確認する。

```bash
git -C src/livox_sdk2_vendor/third_party/Livox-SDK2 fetch origin
git -C src/livox_sdk2_vendor/third_party/Livox-SDK2 checkout <tag>
git -C src/livox_sdk2_vendor/third_party/Livox-SDK2 apply --check \
  ../patches/0001-add-cstdint-include-for-gcc13.patch
```

upstream が `<cstdint>` を取り込んだ場合はパッチが不要になる。その場合は
パッチファイルと `CMakeLists.txt` のパッチ適用ブロックを削除する。
