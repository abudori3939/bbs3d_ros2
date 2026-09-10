# bbs3d_ros2
https://github.com/user-attachments/assets/fc4b50d1-b303-477b-a6aa-9d409b80988f

このROS 2ノードは、[KOKIAOKI/3d_bbs](https://github.com/KOKIAOKI/3d_bbs) の Global Localization を、他のROS 2ノードと同じように `colcon build` できるように整理したラッパーノードです。
本家のテストプログラムの `ros2_test/rviz2` の移植＋ターゲット点群のトピック入力に対応します。

エレベータによる階層移動や広大な地図を分割して切り替える際に初期位置を与えるノードとして使うことを想定しています。
3D-BBS 本体のアルゴリズム・性能・パラメータの詳細は本家のリポジトリを参照してください。

> [!NOTE]
> 本家はこちら -> **https://github.com/KOKIAOKI/3d_bbs**

> **Status:** 未完成。作業中です。

## 対応環境
- Ubuntu 22.04
- ROS 2 humble
- Eigen 3.4+(submodule 経由で取得)
- NVIDIA GPU + CUDA 12.0+ — **任意**。GPU がある環境では GPU 実装を、無い環境では CPU 実装を使います

本家 3D-BBS は GPU 実装 (`gpu_bbs3d`) と CPU 実装 (`cpu_bbs3d`) の両方を提供しています。
本パッケージはビルド時に CUDA と `libgpu_bbs3d.so` の有無を自動判定し、見つかれば GPU 実装込みで、
見つからなければ CPU 実装のみでビルドします。**使い方(コマンド・launch・yaml)は同じです**。
実行時にどちらを使うかは yaml の `backend`(既定 `"auto"`)で切り替えられ、起動ログに
`3D-BBS backend: GPU` / `3D-BBS backend: CPU` として出ます。

> [!NOTE]
> CPU 実装は GPU 実装より大幅に遅く、点群サイズや探索範囲によっては 1 回の推定に数秒〜数十秒かかります。
> `src_leaf_size` を大きくする / `max_scan_range` を絞る / `timeout_msec` を設定する、などで調整してください。

## インストール
### 1. このリポジトリを `--recursive` で clone
本リポジトリは [KOKIAOKI/3d_bbs](https://github.com/KOKIAOKI/3d_bbs) を git submodule として同梱しています。
`--recursive` を忘れると上流ソースが取得されず、後続のビルドが必ず失敗します。

```bash
cd ~/colcon_ws/src
git clone --recursive https://github.com/abudori3939/bbs3d_ros2.git
```

`--recursive` を付け忘れた場合は、後から以下で submodule を初期化できます:

```bash
cd ~/colcon_ws/src/bbs3d_ros2
git submodule update --init --recursive
```

### 2. 本家 3d_bbs を `sudo make install`(初回のみ、ユーザ手動)
`3d_bbs` をビルドしインストールします。`3d_bbs` 内で `build` ディレクトリを作りビルドします。 `sudo make install` しインストールします。

```bash
cd ~/colcon_ws/src/bbs3d_ros2/3d_bbs
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
sudo make install
sudo ldconfig   # /usr/local/lib のライブラリを ld キャッシュに登録
```

**GPU / CUDA が無いマシンでは `-DBUILD_CUDA=OFF` を付けます**(本家の CMake は既定で CUDA を必須とするため、
付けないと configure が失敗します)。CPU 実装 `libcpu_bbs3d.so` はこのオプションに関係なくインストールされます。

```bash
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_CUDA=OFF
make -j
sudo make install
sudo ldconfig
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
デフォルトでは、`data/target/target.pcd` にしています。`data/target` ディレクトリを作成し、この中にpcdファイルを配置してください。指定したディレクトリ内のpcdをすべて読み込む構造をしているため、ファイル名が `target.pcd` である必要はありません。

`/data/` は `.gitignore` 済みのため、git にトラックされません。

### 4. 本パッケージを `colcon build`

```bash
cd ~/colcon_ws
colcon build --packages-select bbs3d_ros2
source install/setup.bash
```

> ビルドタイプは `CMakeLists.txt` で Release をデフォルトにしています。デバッグ目的で切り替えたい場合は `--cmake-args -DCMAKE_BUILD_TYPE=Debug`(または `RelWithDebInfo`)を付けてください。

GPU 実装を含めたかどうかは configure ログに出ます:

```
-- bbs3d_ros2: GPU backend = ON    # CUDA と libgpu_bbs3d.so が見つかった
-- bbs3d_ros2: GPU backend = OFF   # 見つからないので CPU 実装のみ
```

自動判定を上書きしたい場合は `--cmake-args -DBBS3D_ENABLE_GPU=OFF`(GPU 機で CPU のみビルド)/ `=ON` を指定します。

## 動作デモ

`config/bbs3d_ros2.yaml` の `target_clouds` を **絶対パス** に書き換えてから実行します(相対パスは cwd 依存で挙動が変わります)。

### Step 1: `config/bbs3d_ros2.yaml` の編集
```yaml
target_clouds: "/home/yourname/colcon_ws/src/bbs3d_ros2/data/target"  # 絶対パス
```

または編集せず、別 yaml を作って launch 引数で渡すこともできます:

```bash
cp ~/colcon_ws/src/bbs3d_ros2/config/bbs3d_ros2.yaml /tmp/my_bbs3d.yaml
# /tmp/my_bbs3d.yaml の target_clouds を編集
ros2 launch bbs3d_ros2 bbs3d_rviz2.launch.py config_file:=/tmp/my_bbs3d.yaml
```

### Step 2: 起動
```bash
ros2 launch bbs3d_ros2 bbs3d_rviz2.launch.py
```

期待される表示:
- RViz2 ウィンドウ(target_points / global_pose の Display 構成)
- ノードログに `[ROS2] 3D-BBS initialized`

### Step 3: rosbag を再生(別シェル)
```bash
ros2 bag play ~/colcon_ws/src/bbs3d_ros2/data/ros2_test_data
```

これで `/livox/points`(LiDAR)と `/livox/imu`(IMU)が流れ始めます。

### Step 4: グローバル位置推定をトリガ(別シェル)
Topic と Service のどちらでもトリガできます(同名 `~/localize` を共存):
```bash
# Topic で叩く(シンプル版)
ros2 topic pub --once /bbs3d_ros2_node/localize std_msgs/msg/Bool "{data: true}"

# Service で叩く(失敗 reason がレスポンスに乗る、自律システム向け)
ros2 service call /bbs3d_ros2_node/localize std_srvs/srv/Trigger {}
```

期待される結果:
- ノードログに `[Localize] start` → `[Localize] Execution time: ...[msec]` → `[Localize] score: ...`
- `/global_pose` 推定したpose(PoseStamped) が1 回 publish される
- RViz 上で `/src_points_on_global_pose`(赤い点群)が target に重なる位置に表示される

### 自分の環境で実行する場合
- `target_clouds` を自前の地図 PCD ディレクトリの絶対パスに書き換える
- `lidar_topic_name` / `imu_topic_name` を自分の sensor のトピック名に書き換える
- 探索範囲(`min_rpy` / `max_rpy`)・解像度(`min_level_res`)・スコア閾値などをチューニング(本家 README 参照)

### 地図ホットスワップ(topic モード)
階層移動や広大地図の分割切替で、**ノード再起動なしに target を切り替えたい** 場合は `target_source_mode: "topic"` を使います。

```yaml
# config/bbs3d_ros2.yaml
target_source_mode: "topic"
target_cloud_topic_name: "/target_cloud"   # 任意の topic 名に変更可
```

`target_clouds` 行は不要(無視されます)。起動後はノードログに `topic mode: waiting for target on /target_cloud` が出てトリガ待ち状態となり、target topic に PointCloud2 が来るたびに voxelmap を再構築します。送信側 QoS の既定は [REP-2003 Maps 推奨](https://ros.org/reps/rep-2003.html) の `transient_local`+`reliable`(`ros2 bag play` の単発送信や latched publisher で OK)。

`pcl_ros` 等の REP-2003 非準拠 publisher(`volatile`+`reliable` で送信、設定変更不可)と接続する場合は、subscriber 側 QoS を yaml で切替えます:

```yaml
target_cloud_qos_reliability: "best_effort"   # default: "reliable"
target_cloud_qos_durability:  "volatile"      # default: "transient_local"
```

DDS の QoS マッチング規則(`pub.durability >= sub.durability`)で接続不可になっている場合は `ros2 topic info <target_cloud_topic_name> --verbose` で publisher 側 QoS を確認してください。

- target 未受信状態で `~/localize` を叩くと `response.message == "target map not loaded"`
- 再構築中(数秒)に `~/localize` を叩くと **再構築完了まで blocking で待たされた後** に通常 localize 結果が返る(クライアント側のタイムアウトは future の `wait_for` 等で制御してください)
- 再構築完了後の localize は通常通り動作

### CPU / GPU の切り替え
GPU の無いマシンでも**同じ手順のまま**使えます(コマンド・launch・yaml は共通)。どちらの実装で動いているかは起動ログで確認できます:

```
[INFO] [bbs3d_ros2_node]: 3D-BBS backend: CPU
```

実装の選択は 2 段階です(どちらもユーザが意識せず既定のままで動きます)。

**1. ビルド時**(どの実装をコンパイルするか) — CMake が自動判定します。CUDA と `/usr/local/lib/libgpu_bbs3d.so` が両方見つかれば GPU 実装込み、見つからなければ CPU 実装のみになります。判定結果は `colcon build` のログに出ます:

```
-- bbs3d_ros2: GPU backend = ON    # GPU + CPU 両方コンパイル
-- bbs3d_ros2: GPU backend = OFF   # CPU のみ
```

GPU 機であえて CPU のみビルドしたい場合は `colcon build --packages-select bbs3d_ros2 --cmake-args -DBBS3D_ENABLE_GPU=OFF` を使います。

**2. 実行時**(どちらを使うか) — yaml の `backend` で指定します。既定は `"auto"` で、**上の「1. ビルド時」の判定結果**に従います(GPU 込みでビルドされていれば GPU、CPU のみのビルドなら CPU)。既存の yaml をそのまま使う場合は何も追記する必要はありません。

```yaml
# config/bbs3d_ros2.yaml
backend: "auto"   # "auto"(既定) | "gpu" | "cpu"
```

| 値 | 挙動 |
|---|---|
| `"auto"` | GPU 実装を含むビルドなら GPU、CPU のみのビルドなら CPU |
| `"gpu"` | 常に GPU。CPU のみのビルドでは起動時に ERROR を出して exit 1 |
| `"cpu"` | 常に CPU(GPU 機で CPU の速度・精度を比較したいときなど)|

不正な値を書いた場合は起動時に `backend must be 'auto', 'gpu' or 'cpu', got '...'` を出して終了します(黙って別の実装で動くことはありません)。

> [!NOTE]
> `"auto"` の判定は **ビルド時**に決まります。GPU 機でビルドしたバイナリを GPU の見えない環境(`--gpus` 無しのコンテナ、ドライバ不整合など)で動かすと GPU 実装のまま起動して CUDA 側で失敗します。その場合は `backend: "cpu"` を明示するか、その環境でビルドし直してください。

CPU 実装は GPU 実装より大幅に遅いため、実用速度が必要なら以下を調整してください:

- `src_leaf_size` を大きくして source 点群を減らす(探索コストは点数にほぼ比例)
- `max_scan_range` を絞る
- `min_level_res` を大きくして最下層の voxel 解像度を粗くする(精度と引き換え)
- `timeout_msec` を設定して、時間内に見つからなければ失敗として返す

> [!WARNING]
> `max_level` を **小さく** すると探索開始階層が細かくなり、初期変換集合が増えて **逆に遅くなります**。高速化目的で下げないでください。

## ROS 2 インタフェース

> 下表の Topic / Service 名はすべて [`config/bbs3d_ros2.yaml`](config/bbs3d_ros2.yaml) で変更可能です。yaml キーと既定値は [`## 設定`](#設定-configbbs3d_ros2yaml) を参照。

### Subscriptions
| Topic | Type | 用途 |
|---|---|---|
| `~/localize`(完全修飾 `/bbs3d_ros2_node/localize`) | `std_msgs/msg/Bool` | `data: true` でグローバル位置推定をトリガ |
| `<lidar_topic_name>`(サンプル `/livox/points`) | `sensor_msgs/msg/PointCloud2` | source 点群 |
| `<imu_topic_name>`(サンプル `/livox/imu`) | `sensor_msgs/msg/Imu` | 重力方向アライメント用 |
| `<target_cloud_topic_name>`(既定 `/target_cloud`、`target_source_mode: "topic"` 時のみ) | `sensor_msgs/msg/PointCloud2` | target 点群を topic で受け取り voxelmap を動的に再構築(地図ホットスワップ)。QoS は既定で REP-2003 Maps 推奨(`transient_local`+`reliable`)、yaml で変更可 |

### Publications
| Topic | Type | 用途 |
|---|---|---|
| `/tar_points` | `sensor_msgs/msg/PointCloud2` | 起動時に target 点群を 1 度 publish(RViz 表示用) |
| `/src_points_on_global_pose` | `sensor_msgs/msg/PointCloud2` | 推定された global pose に変換した source 点群 |
| `/global_pose` | `geometry_msgs/msg/PoseStamped` | 推定された 6DoF pose |
| `/score` | `std_msgs/msg/Int32` | 推定の best score |
| `/time` | `std_msgs/msg/Float32` | 推定の実行時間 [msec] |

### Services
| Service | Type | 用途 |
|---|---|---|
| `~/localize`(完全修飾 `/bbs3d_ros2_node/localize`) | `std_srvs/srv/Trigger` | グローバル位置推定をトリガ。失敗時は `response.message` に reason(`"point cloud not received"`、`"imu not received"`、`"localization timed out"`、`"score below threshold"`、`"target map not loaded"`)を返す。topic モードで target 再構築中の呼出は再構築完了まで blocking で待つ |

> Topic と Service は ROS 2 で別名前空間に属するため、同じ完全修飾名で共存できます。`ros2 topic pub` か `ros2 service call` かで型に応じた呼び出しになります。

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
| `backend` | 使用する 3D-BBS 実装(既定 `"auto"`、`"gpu"` / `"cpu"` で固定)|
| `tar_points_topic_name` | target 点群 publisher 名(既定 `/tar_points`) |
| `src_points_on_global_pose_topic_name` | source 点群 publisher 名(既定 `/src_points_on_global_pose`) |
| `global_pose_topic_name` | 推定 pose publisher 名(既定 `/global_pose`) |
| `score_topic_name` | best score publisher 名(既定 `/score`) |
| `time_topic_name` | 実行時間 publisher 名(既定 `/time`) |
| `localize_topic_name` | トリガ Bool topic + Trigger service の共通名(既定 `~/localize`) |
| `target_source_mode` | target 点群の入手元(既定 `pcd`、または `topic` で動的受信) |
| `target_cloud_topic_name` | topic モードで subscribe する target トピック名(既定 `/target_cloud`)|
| `target_cloud_qos_reliability` | target sub の reliability(既定 `"reliable"`、または `"best_effort"`)|
| `target_cloud_qos_durability` | target sub の durability(既定 `"transient_local"` = REP-2003 Maps 推奨、または `"volatile"` = `pcl_ros` 等と接続用)|

> Topic / Service 名は既定で `config/bbs3d_ros2.yaml` 内ではコメントアウトされています(=既定値で動く)。変更したい行の `#` を外して値を書き換えてください。

## トラブルシューティング

| 症状 | 原因 / 対処 |
|---|---|
| `[ERROR] Can not open folder` の直後に segfault | `target_clouds` がプレースホルダ(`/path/to/target`)のまま、または存在しないパス。**絶対パス**を指定すること。フェーズ B(Step 8)で graceful error に改善予定 |
| ビルドが `find_package(cpu_bbs3d) failed` で失敗 | Step 2(本家 `sudo make install`)が未実行。`/usr/local/lib/libcpu_bbs3d.so` を確認 |
| 起動時に `error while loading shared libraries: libcpu_bbs3d.so` | `sudo ldconfig` が未実行(`/usr/local/lib` が ld キャッシュに入っていない)|
| GPU 機なのに `GPU backend = OFF` になる | 本家を `-DBUILD_CUDA=OFF` でビルドした、または CUDA が見つからない。`/usr/local/lib/libgpu_bbs3d.so` と `nvcc` を確認 |
| 起動時に `backend: 'gpu' was requested but this build has no GPU support` | CPU のみでビルドしたパッケージに `backend: "gpu"` を指定している。`"auto"` / `"cpu"` にするか、GPU 環境でビルドし直す |
| CPU で 1 回の推定が非常に遅い | CPU 実装は GPU の数十倍遅い。`src_leaf_size` を大きくする / `max_scan_range` を絞る / `timeout_msec` を設定する |
| `submodule update` 後に動作不安定 | 上流ヘッダだけ新しくなりライブラリが古いまま。Step 2 を再実行 |
| `~/localize` を pub / call しても何も起きない | rosbag 再生中(`/livox/points` `/livox/imu` が流れている)か確認。Service なら `response.message` に `"point cloud not received"` / `"imu not received"` 等が乗る。Topic 経由のときはノードログに同じメッセージが出るので、トピック名やセンサデータの流れを確認 |

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
