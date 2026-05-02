# bbs3d_ros2

[KOKIAOKI/3d_bbs](https://github.com/KOKIAOKI/3d_bbs) の GPU 全探索グローバル位置推定を、上流 `ros2_test/rviz2` パッケージから移植して colcon build 可能な単一 ament パッケージにした ROS 2 (humble) ノードです。3D-BBS 本体のアルゴリズム・性能・パラメータの詳細は本家のリポジトリを参照してください。

> 詳細ドキュメント・論文・テストデータ・ベンチマーク等はすべて本家にあります → **https://github.com/KOKIAOKI/3d_bbs**

> **Status:** WIP。フェーズ A(上流互換移植)が進行中で、フェーズ B(Service トリガ・トピック名 config 化・target 動的入力・エラーハンドリング強化)は未着手です。ロードマップと進捗は [`docs/IMPLEMENTATION_PLAN.md`](docs/IMPLEMENTATION_PLAN.md) を参照してください。

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
- NVIDIA GPU(3D-BBS の voxelmap 構築・探索に必要)

## インストール

導入は 4 ステップ:**(1) clone → (2) 3d_bbs を sudo make install → (3) テストデータ配置(初回デモ用)→ (4) colcon build**。

### 1. `--recursive` で clone
本リポジトリは [KOKIAOKI/3d_bbs](https://github.com/KOKIAOKI/3d_bbs) を git submodule として同梱しています。`--recursive` を忘れると上流ソースが取得されず、後続のビルドが必ず失敗します。

```bash
cd ~/colcon_ws/src
git clone --recursive git@github.com:abudori3939/bbs3d_ros2.git
```

`--recursive` を付け忘れた場合は、後から以下で submodule を初期化できます:

```bash
cd ~/colcon_ws/src/bbs3d_ros2
git submodule update --init --recursive
```

### 2. 本家 3d_bbs を `sudo make install`(初回のみ、ユーザ手動)
**本パッケージは 3d_bbs のインストールを支援しません。** ユーザが手動で本家を `sudo make install` し、`/usr/local/lib/libgpu_bbs3d.so` とヘッダを配置してください。本パッケージはそれを `cmake/Findgpu_bbs3d.cmake` 経由で発見してリンクします。

```bash
cd ~/colcon_ws/src/bbs3d_ros2/3d_bbs
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
sudo make install
```

> **重要:** `git submodule update` で `3d_bbs/` を更新したときは、**必ずこのステップを再実行**してください。古いライブラリと新しいヘッダの不整合で実行時エラーになることがあります。

### 3. テストデータの配置(動作デモを試す場合のみ)
本家のデモを再現するには、本家配布のテストデータ(target PCD + rosbag)が必要です。

1. 本家の [`3d_bbs/ros2_test/ros2_test_code.md`](3d_bbs/ros2_test/ros2_test_code.md) に記載の Google Drive リンクから **target データ** と **ros2_test_data**(rosbag)をダウンロード
2. 本リポジトリの `data/` 配下に配置:

```
bbs3d_ros2/
├── data/
│   ├── target/target.pcd        # ← target データ
│   └── ros2_test_data/          # ← rosbag(後段で使用)
│       ├── metadata.yaml
│       └── ros2_test_data.db3
```

`/data/` は `.gitignore` 済みのため、git にトラックされません。

### 4. 本パッケージを `colcon build`

```bash
cd ~/colcon_ws
colcon build --packages-select bbs3d_ros2
source install/setup.bash
```

> ビルドタイプは `CMakeLists.txt` で Release をデフォルトにしています。デバッグ目的で切り替えたい場合は `--cmake-args -DCMAKE_BUILD_TYPE=Debug`(または `RelWithDebInfo`)を付けてください。

## 動作デモ

`config/bbs3d_ros2.yaml` の `target_clouds` を **絶対パス** に書き換えてから実行します(相対パスは cwd 依存で挙動が変わります)。

### Step A: `config/bbs3d_ros2.yaml` の編集
```yaml
target_clouds: "/home/yourname/colcon_ws/src/bbs3d_ros2/data/target"  # 絶対パス
```

または編集せず、別 yaml を作って launch 引数で渡すこともできます:

```bash
cp ~/colcon_ws/src/bbs3d_ros2/config/bbs3d_ros2.yaml /tmp/my_bbs3d.yaml
# /tmp/my_bbs3d.yaml の target_clouds を編集
ros2 launch bbs3d_ros2 bbs3d_rviz2.launch.py config_file:=/tmp/my_bbs3d.yaml
```

### Step B: 起動
```bash
source ~/colcon_ws/install/setup.bash
ros2 launch bbs3d_ros2 bbs3d_rviz2.launch.py
```

期待される表示:
- RViz2 ウィンドウ(target_points / global_pose の Display 構成)
- ノードログに `[ROS2] 3D-BBS initialized`

### Step C: rosbag を再生(別シェル)
```bash
source ~/colcon_ws/install/setup.bash
ros2 bag play ~/colcon_ws/src/bbs3d_ros2/data/ros2_test_data
```

これで `/livox/points`(LiDAR)と `/livox/imu`(IMU)が流れ始めます。

### Step D: グローバル位置推定をトリガ(別シェル)
```bash
source ~/colcon_ws/install/setup.bash
ros2 topic pub --once /click_loc std_msgs/msg/Bool "{data: true}"
```

期待される結果:
- ノードログに `[Localize] start` → `[Localize] Execution time: ...[msec]` → `[Localize] score: ...`
- `/global_pose` `/score` `/time` がそれぞれ 1 回 publish される
- RViz 上で `global_pose`(赤い点群)が target に重なる位置に表示される

### 自分の環境で実行する場合
- `target_clouds` を自前の地図 PCD ディレクトリの絶対パスに書き換える
- `lidar_topic_name` / `imu_topic_name` を自分の sensor のトピック名に書き換える
- 探索範囲(`min_rpy` / `max_rpy`)・解像度(`min_level_res`)・スコア閾値などをチューニング(本家 README 参照)

## ROS 2 インタフェース

### Subscriptions
| Topic | Type | 用途 |
|---|---|---|
| `/click_loc` | `std_msgs/msg/Bool` | `data: true` でグローバル位置推定をトリガ |
| `<lidar_topic_name>`(default `/livox/points`) | `sensor_msgs/msg/PointCloud2` | source 点群 |
| `<imu_topic_name>`(default `/livox/imu`) | `sensor_msgs/msg/Imu` | 重力方向アライメント用 |

### Publications
| Topic | Type | 用途 |
|---|---|---|
| `/tar_points` | `sensor_msgs/msg/PointCloud2` | 起動時に target 点群を 1 度 publish(RViz 表示用) |
| `/src_points_on_global_pose` | `sensor_msgs/msg/PointCloud2` | 推定された global pose に変換した source 点群 |
| `/global_pose` | `geometry_msgs/msg/PoseStamped` | 推定された 6DoF pose |
| `/score` | `std_msgs/msg/Int32` | 推定の best score |
| `/time` | `std_msgs/msg/Float32` | 推定の実行時間 [msec] |

### Services
- 現在なし。フェーズ B(Step 7)で `~/localize`(`std_srvs/srv/Trigger`)を追加予定。

### TF
- `map → viewer`(target 点群の重心位置に viewer フレームを broadcast、RViz の TopDown 視点用)

## 設定 (`config/bbs3d_ros2.yaml`)

スキーマは上流 [`3d_bbs/ros2_test/config/ros2_test.yaml`](3d_bbs/ros2_test/config/ros2_test.yaml) と同じです。主な項目:

| キー | 説明 |
|---|---|
| `target_clouds` | **絶対パス**で target PCD ディレクトリを指定 |
| `lidar_topic_name` / `imu_topic_name` | 入力トピック名 |
| `min_level_res` / `max_level` | voxelmap の最小解像度 / 階層数 |
| `min_rpy` / `max_rpy` | 角度探索範囲 [rad](`6.28` は内部で `2π` に置換) |
| `score_threshold_percentage` | スコア閾値(`floor(src_size * pct)` を下回ると失敗扱い) |
| `tar_leaf_size` / `src_leaf_size` | ダウンサンプル(0.0 で off) |
| `min_scan_range` / `max_scan_range` | source 点群のクロップ [m] |
| `timeout_msec` | 探索タイムアウト(0 で off) |

## トラブルシューティング

| 症状 | 原因 / 対処 |
|---|---|
| `[ERROR] Can not open folder` の直後に segfault | `target_clouds` がプレースホルダ(`/path/to/target`)のまま、または存在しないパス。**絶対パス**を指定すること。フェーズ B(Step 8)で graceful error に改善予定 |
| ビルドが `find_package(gpu_bbs3d) failed` で失敗 | Step 2(本家 `sudo make install`)が未実行。`/usr/local/lib/libgpu_bbs3d.so` を確認 |
| `submodule update` 後に動作不安定 | 上流ヘッダだけ新しくなりライブラリが古いまま。Step 2 を再実行 |
| `/click_loc` を pub しても何も起きない | rosbag 再生中(`/livox/points` `/livox/imu` が流れている)か確認。ノードログに `point cloud msg is not received` / `imu msg is not received` が出ていれば未受信 |

## 開発

開発フロー(ブランチ運用 / TDD / plan mode)は [`docs/DEVELOPMENT_WORKFLOW.md`](docs/DEVELOPMENT_WORKFLOW.md)、進捗台帳は [`docs/IMPLEMENTATION_PLAN.md`](docs/IMPLEMENTATION_PLAN.md) を参照してください。

## ライセンス
MIT。詳細は [`LICENSE`](LICENSE) を参照。本家 [KOKIAOKI/3d_bbs](https://github.com/KOKIAOKI/3d_bbs) も MIT ライセンスで、本リポジトリはそれに基づきます。

## 謝辞 / 引用
本パッケージは [KOKIAOKI/3d_bbs](https://github.com/KOKIAOKI/3d_bbs)(著者: Koki Aoki 氏ほか)の成果物を移植・拡張したものです。3D-BBS のアルゴリズム・性能評価・テストデータはすべて本家にあります。研究利用の際は元論文を引用してください:

```bibtex
@inproceedings{aoki20243dbbs,
  title={3D-BBS: Global Localization for 3D Point Cloud Scan Matching Using Branch-and-Bound Algorithm},
  author={Koki Aoki and Kenji Koide and Shuji Oishi and Masashi Yokozuka and Atsuhiko Banno and Junichi Meguro},
  booktitle={IEEE International Conference on Robotics and Automation},
  year={2024},
  organization={IEEE}
}
```
