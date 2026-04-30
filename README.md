# bbs3d_ros2

[KOKIAOKI/3d_bbs](https://github.com/KOKIAOKI/3d_bbs) の **ROS 2 ラッパーノード** です。3D-BBS 本体のアルゴリズム・性能・パラメータの詳細は本家のリポジトリを参照してください。

> 詳細ドキュメント・論文・テストデータ・ベンチマーク等はすべて本家にあります → **https://github.com/KOKIAOKI/3d_bbs**

> **Status:** WIP。ロードマップと進捗は [`docs/IMPLEMENTATION_PLAN.md`](docs/IMPLEMENTATION_PLAN.md) を参照してください。

## 動機
本家の `ros2_test_rviz2` は 3D-BBS のデモ実装で、屋外ロボットへ組み込むには手数がかかります。本リポジトリは **屋外ロボットでの実運用** を目的に、

- 入出力のトピック名や Service など **ROS 2 インタフェース部を改変しやすい構造** にする、
- launch / config / rviz 設定を ROS 2 標準の `share/<pkg>/` 配置に揃えて **アクセスしやすく** する、
- 1 つの ament パッケージとして clone → `colcon build` で完結する **配布しやすい形** にする、

ことを狙っています。3D-BBS 本体には手を入れず、本家を git submodule として同梱しています。

## 前提
- Ubuntu 22.04
- ROS 2 humble
- CUDA 12.0+
- Eigen 3.4+(submodule 経由で取得)

## インストール

### 1. **必ず `--recursive` を付けて** clone
本リポジトリは [KOKIAOKI/3d_bbs](https://github.com/KOKIAOKI/3d_bbs) を git submodule として同梱しています。`--recursive` を忘れると上流ソースがダウンロードされず、後続のビルドが必ず失敗します。

```bash
cd ~/colcon_ws/src
git clone --recursive git@github.com:abudori3939/bbs3d_ros2.git
```

`--recursive` を付け忘れた場合は、後から以下で submodule を初期化できます:

```bash
cd ~/colcon_ws/src/bbs3d_ros2
git submodule update --init --recursive
```

### 2. 本家 3d_bbs をインストール(初回のみ)
本ラッパーは本家の `libgpu_bbs3d.so` をリンクする方式です。**先に本家を `sudo make install` してください。** 手順は本家の README に従います。

```bash
cd ~/colcon_ws/src/bbs3d_ros2/3d_bbs
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
sudo make install
```

> **重要:** `git submodule update` で `3d_bbs/` を更新したときは、必ずこのステップを再実行してください。古いライブラリと新しいヘッダの不整合で実行時エラーになることがあります。

### 3. ラッパーノードを `colcon build`
ここで初めて本リポジトリ自体をビルドします。

```bash
cd ~/colcon_ws
colcon build --packages-select bbs3d_ros2
source install/setup.bash
```

> ビルドタイプは `CMakeLists.txt` で Release をデフォルトにしています。デバッグ目的で切り替えたい場合は `--cmake-args -DCMAKE_BUILD_TYPE=Debug`(または `RelWithDebInfo`)を付けてください。

## 実行

```bash
ros2 launch bbs3d_ros2 bbs3d_rviz2.launch.py config_file:=/path/to/your/config.yaml
```

(詳細な実行手順とトピック / Service の I/F 一覧は WIP — Step 5 で更新します。)

## ライセンス
MIT。本家 [KOKIAOKI/3d_bbs](https://github.com/KOKIAOKI/3d_bbs)(MIT)に基づきます。研究利用の際は元論文を引用してください:

```
@inproceedings{aoki20243dbbs,
  title={3D-BBS: Global Localization for 3D Point Cloud Scan Matching Using Branch-and-Bound Algorithm},
  author={Koki Aoki and Kenji Koide and Shuji Oishi and Masashi Yokozuka and Atsuhiko Banno and Junichi Meguro},
  booktitle={IEEE International Conference on Robotics and Automation},
  year={2024},
  organization={IEEE}
}
```
