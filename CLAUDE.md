# bbs3d_ros2 — Claude 向けプロジェクト概要

## このリポジトリの位置づけ
[KOKIAOKI/3d_bbs](https://github.com/KOKIAOKI/3d_bbs) の GPU 全探索グローバル位置推定を、単一の ament パッケージとしてラップする ROS 2 (humble) ノード。`package.xml` 1 つ、実行ファイル 1 つ (`bbs3d_ros2_node`)、launch ファイル 1 つの構成。

## アーキテクチャ(1 段落)
上流 `3d_bbs` は `./3d_bbs/` に git submodule として配置し、`COLCON_IGNORE` を置いて colcon が走査しないようにしている。ユーザは初回のみ `cd 3d_bbs && cmake .. && make && sudo make install` を実行し、`libgpu_bbs3d.so` と `gpu_bbs3d` / `pointcloud_iof` / `discrete_transformation` のヘッダを `/usr/local` 以下に配置する。本パッケージの `cmake/Findgpu_bbs3d.cmake` がそれを解決して `find_package(gpu_bbs3d)` が成立する。本ラッパーは **CUDA を再コンパイルしない** — `libgpu_bbs3d.so` をリンクし、ヘッダを include するだけ。

## どこから読むか
**まず [`docs/IMPLEMENTATION_PLAN.md`](docs/IMPLEMENTATION_PLAN.md) を読むこと。** Step 0〜7 の計画(checkbox 付き)、設計判断、移植元の上流ファイルへのポインタを含む唯一の作業台帳。最初の未完了 step から再開する。

## 規約
- ユーザとの会話は **日本語** で行う。
- **`3d_bbs/` 以下は触らない** — これは上流 submodule。ラッパーコードはリポジトリルート (`include/`、`src/`、`launch/`、`config/`、`rviz/`) に置く。
- `package.xml` の name は `bbs3d_ros2`(REP 144 で先頭数字が禁止されているため)。GitHub リポジトリ名は別でも可。
- 装飾的なコメントや先回りした抽象化は書かない。基本は上流挙動を維持し、計画に明記された差分のみを入れる。

## 上流の主な参照(パスは `3d_bbs/` からの相対)
- `bbs3d/include/gpu_bbs3d/bbs3d.cuh` — 公開 API。
- `ros2_test/rviz2/include/ros2_test_rviz2.hpp` — `include/bbs3d_ros2/wrapper_node.hpp` への移植元。
- `ros2_test/rviz2/src/gpu_bbs3d_rviz2/gpu_ros2_test_rviz2.cpp` — `src/wrapper_node.cpp` への移植元。
- `ros2_test/config/ros2_test.yaml` — config スキーマ。
- `ros2_test/rviz2/launch/gpu_ros2_test_rviz2_launch.py` と `rviz2_config/rviz2.rviz` — 参照 launch / rviz 設定。

## ビルド
```bash
# 初回のみ: 上流 gpu_bbs3d のインストール
cd ~/colcon_ws/src/bbs3d_ros2/3d_bbs
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j && sudo make install

# ラッパーのビルド
cd ~/colcon_ws
colcon build --packages-select bbs3d_ros2  # Release is the default in CMakeLists.txt
source install/setup.bash
ros2 launch bbs3d_ros2 bbs3d_rviz2.launch.py
```

`3d_bbs/` submodule を更新したら `sudo make install` を必ず再実行する。
