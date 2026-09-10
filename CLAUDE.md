# bbs3d_ros2 — Claude 向けプロジェクト概要

## このリポジトリの位置づけ
[KOKIAOKI/3d_bbs](https://github.com/KOKIAOKI/3d_bbs) の全探索グローバル位置推定を、**上流 `ros2_test/rviz2` パッケージを移植 (port) して `colcon build` 可能な単一 ament パッケージにする** ROS 2 (humble) ノード。`package.xml` 1 つ、実行ファイル 1 つ (`bbs3d_ros2_node`)、launch ファイル 1 つの構成。フェーズ A で上流互換移植、フェーズ B で使いにくさを改善する拡張(Service トリガ、トピック名 config 化、target 点群の topic 入力、エラーハンドリング強化)を追加し、Step 11 で GPU 非搭載マシン向けに CPU 実装対応を入れた。

## アーキテクチャ(1 段落)
上流 `3d_bbs` は `./3d_bbs/` に git submodule として配置し、`COLCON_IGNORE` を置いて colcon が走査しないようにしている。ユーザは初回のみ手動で `cd 3d_bbs && cmake .. && make && sudo make install && sudo ldconfig` を実行し(GPU 非搭載機では `cmake .. -DBUILD_CUDA=OFF`)、`libcpu_bbs3d.so`(常に)/ `libgpu_bbs3d.so`(CUDA 有効時のみ)と `cpu_bbs3d` / `gpu_bbs3d` / `pointcloud_iof` / `discrete_transformation` のヘッダを `/usr/local` 以下に配置する(**本パッケージはこのインストールを支援しない**)。本パッケージの `cmake/Findcpu_bbs3d.cmake` / `cmake/Findgpu_bbs3d.cmake` がそれを解決する。本パッケージは **CUDA を再コンパイルしない** — ライブラリをリンクし、ヘッダを include するだけ。GPU 実装は CUDA と `libgpu_bbs3d.so` が見つかったときだけコンパイルされ(`BBS3D_HAS_GPU`)、実行時にどちらを使うかは yaml の `backend`(`auto` / `gpu` / `cpu`)で決まる。ノードは `include/bbs3d_ros2/bbs3d_backend.hpp` の仮想インタフェース越しにしか BBS3D を触らない。

## どこから読むか
1. **[`docs/IMPLEMENTATION_PLAN.md`](docs/IMPLEMENTATION_PLAN.md)** — Step 0〜11 の計画(checkbox 付き)、設計判断、移植元の上流ファイルへのポインタを含む唯一の作業台帳。フェーズ A(Step 3〜6 = 上流互換移植)とフェーズ B(Step 7〜11 = 拡張)に分かれている。最初の未完了 step から再開する。
2. **[`docs/DEVELOPMENT_WORKFLOW.md`](docs/DEVELOPMENT_WORKFLOW.md)** — ブランチ運用 / PR / レビュー / マージのルール。**コード変更を始める前に必ず一読**。

## 規約
- ユーザとの会話は **日本語** で行う。
- **すべての変更は PR 経由で main にマージする**(直接 push 禁止)。1 Step = 1 ブランチ = 1 PR を原則とする。詳細は [`docs/DEVELOPMENT_WORKFLOW.md`](docs/DEVELOPMENT_WORKFLOW.md) を参照。
- **コード/CMake/テストの実装前に必ず plan mode で計画する**:何を・どう作るか(TDD 適用時は RED テスト設計を含む)を提示し、ユーザの明示的承認を得てから着手する。承認なしに実装を開始しない。詳細は [`docs/DEVELOPMENT_WORKFLOW.md`](docs/DEVELOPMENT_WORKFLOW.md)。**ドキュメントのみの変更は対象外**。
- **ROS 2 ノードの機能追加は TDD で進める**:plan(上記)→ ユーザ承認 → RED(失敗テスト)→ ユーザ承認 → GREEN(最小実装)→ ユーザ承認 → PR の順。**ビルドエラーは RED として認められない**(テスト実行に到達した上で意図通り FAIL すること)。
- **`3d_bbs/` 以下は触らない** — これは上流 submodule。構造改変・インストール支援スクリプトも置かない。本パッケージのコードはリポジトリルート (`include/`、`src/`、`launch/`、`config/`、`rviz/`) に置く。
- `package.xml` の name は `bbs3d_ros2`(REP 144 で先頭数字が禁止されているため)。GitHub リポジトリ名は別でも可。
- 装飾的なコメントや先回りした抽象化は書かない。フェーズ A は上流挙動を厳密に維持。フェーズ B は計画に明記された差分のみを入れる。

## 上流の主な参照(パスは `3d_bbs/` からの相対)
- `bbs3d/include/gpu_bbs3d/bbs3d.cuh` — GPU 版の公開 API(float)。
- `bbs3d/include/cpu_bbs3d/bbs3d.hpp` — CPU 版の公開 API(double)。GPU 版とほぼ同一。
- `test/src/cpu_test.cpp` / `test/cmake/Findcpu_bbs3d.cmake` — CPU 版の呼び出し順序と Find モジュールの移植元。
- `ros2_test/rviz2/include/ros2_test_rviz2.hpp` — `include/bbs3d_ros2/bbs3d_node.hpp` への移植元。
- `ros2_test/rviz2/src/gpu_bbs3d_rviz2/gpu_ros2_test_rviz2.cpp` — `src/bbs3d_node.cpp` への移植元。
- `ros2_test/config/ros2_test.yaml` — config スキーマ。
- `ros2_test/rviz2/launch/gpu_ros2_test_rviz2_launch.py` と `rviz2_config/rviz2.rviz` — 参照 launch / rviz 設定。
- `ros2_test/click_loc/` — 上流の Tk トリガ。本パッケージでは移植せず、`ros2 topic pub /click_loc std_msgs/msg/Bool "{data: true}"` で代替する。

## ビルド
```bash
# 初回のみ: 上流 3d_bbs のインストール(ユーザが手動で行う)
# GPU 非搭載機では cmake に -DBUILD_CUDA=OFF を追加する
cd ~/colcon_ws/src/bbs3d_ros2/3d_bbs
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j && sudo make install && sudo ldconfig

# 本パッケージのビルド
cd ~/colcon_ws
colcon build --packages-select bbs3d_ros2  # Release is the default in CMakeLists.txt
source install/setup.bash
ros2 launch bbs3d_ros2 bbs3d_rviz2.launch.py
```

`3d_bbs/` submodule を更新したら `sudo make install && sudo ldconfig` を必ず再実行する。
