# bbs3d_ros2 実装計画

このドキュメントは `bbs3d_ros2` の構築作業の唯一の真実 (source of truth) です。前提知識のないセッション(別の Claude セッションを含む)が、最初の未完了項目から作業を再開できることを意図して書かれています。

> **開発フローのルールは [`DEVELOPMENT_WORKFLOW.md`](DEVELOPMENT_WORKFLOW.md) を参照してください。** すべての変更は PR 経由で main にマージし、各 Step の checkbox 更新もその PR 内で行います。

## 目的
[KOKIAOKI/3d_bbs](https://github.com/KOKIAOKI/3d_bbs) のグローバル位置推定を単一の ament パッケージとしてラップする ROS 2 ノード。配布モデル:

1. ユーザはこのリポジトリを `~/colcon_ws/src/` に再帰 clone する。
2. ユーザは同梱の `3d_bbs/` submodule で `cmake && make && sudo make install` を 1 回だけ実行する(`libgpu_bbs3d.so` とヘッダを `/usr/local` 以下に配置)。
3. ユーザは `colcon build --packages-select bbs3d_ros2` でビルドし、launch を起動する。

## 命名
- パッケージ名(`package.xml` 内): **`bbs3d_ros2`** — REP 144 により先頭は英字必須。上流の識別子(`gpu_bbs3d`、namespace `gpu`、`bbs3d/include/`)と整合する。
- GitHub リポジトリ名: `bbs3d_ros2` でも `3d_bbs_ros2` でも可。パッケージ名は固定。

## レイアウト
```
bbs3d_ros2/
├── package.xml
├── CMakeLists.txt
├── cmake/Findgpu_bbs3d.cmake     # /usr/local 配下の gpu_bbs3d を探す
├── include/bbs3d_ros2/wrapper_node.hpp
├── src/wrapper_node.cpp
├── src/wrapper_main.cpp
├── launch/bbs3d_rviz2.launch.py
├── config/bbs3d_ros2.yaml
├── rviz/bbs3d.rviz
├── docs/IMPLEMENTATION_PLAN.md   # 本ファイル
├── CLAUDE.md                     # 次セッションの Claude 向けブリーフィング
├── README.md
├── scripts/setup.sh              # 任意の補助スクリプト(Step 6)
└── 3d_bbs/                       # git submodule。COLCON_IGNORE を含む
```

## 設計判断
1. **上流は submodule + COLCON_IGNORE**。 ament パッケージは 1 つに保ち、上流のネストされた ament パッケージを colcon が走査しないようにする。
2. **`gpu_bbs3d` は `/usr/local` インストール経由で発見**。 ラッパーは CUDA を再コンパイルしない。`find_package(gpu_bbs3d)` は本リポジトリ `cmake/Findgpu_bbs3d.cmake`(上流からコピー)で解決する。
3. **本 CMake で CUDA 言語は有効化しない**。 `libgpu_bbs3d.so` をリンクし CUDA ヘッダ(`<cuda_runtime.h>`、thrust)を include するだけなので、`find_package(CUDA REQUIRED)`(legacy module)で十分。
4. **Tk ベースの `click_loc` GUI は廃止**するが、トリガは `/click_loc` Bool トピック(上流互換、トピックしか触らないユーザ向け)と `std_srvs/srv/Trigger` Service(`~/localize`、自律システム向け)の **両方を常用** する。Bool は今後も廃止しない。
5. **上流挙動を素直に維持する**。 mutex 不在による潜在的レース、IMU 時刻差バグ(`gpu_ros2_test_rviz2.cpp:190-193` の sec/nanosec 符号混在)など上流の課題は本ラッパーでは触らず、必要に応じて上流に issue を投げる(下記「上流に issue を投げる候補」参照)。
6. **設定は yaml 一本**。 上流の `ros2_test.yaml` スキーマを踏襲し、ROS 2 パラメータでの上書きは入れない。launch ファイルは rviz2 など別プロセスを束ねる目的のみに使う。

## ステップと進捗

作業が進んだら checkbox を更新する。各 step の最終行は完了条件 (DoD)。

### Step 0 — リポジトリ初期化  ✅ 完了
- [x] `mkdir bbs3d_ros2/{include/bbs3d_ros2,src,launch,config,rviz,cmake,docs,scripts}`
- [x] `git init -b main`
- [x] `.gitignore`
- DoD: 空のリポジトリと骨格ディレクトリが揃っている。

### Step 1 — Submodule 追加 + COLCON_IGNORE  ✅ 完了
- [x] `git submodule add https://github.com/KOKIAOKI/3d_bbs.git 3d_bbs`
- [x] `3d_bbs/COLCON_IGNORE`(空ファイル)
- [ ](任意・推奨)`git submodule update --init --recursive` で `3d_bbs/thirdparty/Eigen` も取得する。上流 CMake はシステム Eigen3 にフォールバックするため必須ではない。
- DoD: `~/colcon_ws` から `colcon list` を実行したとき、`bbs3d_ros2` のみが表示され、上流のネストパッケージは出ない。

### Step 2 — ament パッケージ骨格  ✅ 完了
- [x] `package.xml`(rclcpp、tf2_ros、sensor_msgs、std_msgs、std_srvs、geometry_msgs、pcl_conversions、libpcl-all-dev、eigen、yaml-cpp、libboost-filesystem-dev、exec で rviz2 / launch_ros)。
- [x] `CMakeLists.txt`(executable はまだ追加せず、`ament_auto_package` のみ)。
- [x] `cmake/Findgpu_bbs3d.cmake`(上流 `ros2_test/rviz2/cmake/Findgpu_bbs3d.cmake` のコピー)。
- DoD: `colcon build --packages-select bbs3d_ros2` が executable なしで成功する(configure 通過)。

### Step 3 — ラッパーノード移植  🟡 未着手
- [ ] `include/bbs3d_ros2/wrapper_node.hpp`
  - 上流 `3d_bbs/ros2_test/rviz2/include/ros2_test_rviz2.hpp` をベースに作る。
  - クラス名を `Bbs3dRos2Wrapper`、`namespace bbs3d_ros2` で囲う。
  - `rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr localize_srv_` を追加。
- [ ] `src/wrapper_node.cpp`
  - 上流 `3d_bbs/ros2_test/rviz2/src/gpu_bbs3d_rviz2/gpu_ros2_test_rviz2.cpp` を素直に移植(挙動互換)。クラス名 / namespace のみ書き換える。
  - `~/localize` Service ハンドラを実装し、`click_callback` と同じ処理を呼ぶ(共通処理を `private` メソッドに切り出すのが自然)。
  - mutex 保護や IMU バグ修正、ROS 2 パラメータ上書きなど **上流に存在しない振る舞い変更は入れない**。
- [ ] `src/wrapper_main.cpp` — 薄い `rclcpp::init` + `spin(std::make_shared<Bbs3dRos2Wrapper>())`。
- [ ] `CMakeLists.txt` — executable target を追加:
  ```cmake
  find_package(CUDA REQUIRED)
  find_package(gpu_bbs3d REQUIRED)
  find_package(PCL REQUIRED)
  find_package(Eigen3 REQUIRED)
  find_package(yaml-cpp REQUIRED)

  ament_auto_add_executable(bbs3d_ros2_node
    src/wrapper_main.cpp
    src/wrapper_node.cpp
  )
  target_include_directories(bbs3d_ros2_node PUBLIC
    ${PROJECT_SOURCE_DIR}/include
    ${gpu_bbs3d_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${CUDA_INCLUDE_DIRS}
  )
  target_link_libraries(bbs3d_ros2_node
    ${gpu_bbs3d_LIBRARY}
    ${PCL_LIBRARIES}
    yaml-cpp
  )
  ```
- DoD: `colcon build` が `bbs3d_ros2_node` を生成し、`ros2 run bbs3d_ros2 bbs3d_ros2_node --ros-args -p config:=...` で起動して、有効な yaml に対し `[ROS2] 3D-BBS initialized` を表示する。

### Step 4 — launch / config / rviz  🟡 未着手
- [ ] `launch/bbs3d_rviz2.launch.py`
  - `get_package_share_directory('bbs3d_ros2')` で解決する。
  - `DeclareLaunchArgument('config_file', default_value=<share>/config/bbs3d_ros2.yaml)`。
  - 2 ノード: `rviz2`(`-d <share>/rviz/bbs3d.rviz`)と `bbs3d_ros2_node`。
- [ ] `config/bbs3d_ros2.yaml` — 上流 `ros2_test/config/ros2_test.yaml` のコピー。`target_clouds: ""` にし、上書き手段(launch 引数 / `ros2 param set`)をコメントで案内。
- [ ] `rviz/bbs3d.rviz` — 上流 `ros2_test/rviz2/rviz2_config/rviz2.rviz` のコピー。`~/localize` Service Call パネルを追加(または `ros2 service call` 手順を README に書く)。
- DoD: `ros2 launch bbs3d_ros2 bbs3d_rviz2.launch.py` で上流デモと同じ Display 構成の RViz が起動する。

### Step 5 — README + LICENSE + ユーザドキュメント  🟡 未着手
- [ ] `README.md` を拡充:
  - 前提条件(Ubuntu 22.04、ROS 2 humble、CUDA 12.0+、Eigen 3.4+)。
  - 4 ステップの導入・ビルド・実行手順。
  - **太字注意:** `git submodule update` 後は必ず `sudo make install` を再実行する旨。
  - test data リンク(上流 README より)。
  - トピック / Service の I/F 一覧表。
- [ ] `LICENSE` — MIT。著作権表示はメンテナ名で。
- [ ] README で上流 `KOKIAOKI/3d_bbs` への謝辞を明記。
- DoD: 初見のユーザが README だけで動作デモまで到達できる。

### Step 6 — 補助スクリプト(任意)  🟡 未着手
- [ ] `scripts/setup.sh`
  - `git submodule update --init --recursive`
  - `[ ! -f 3d_bbs/COLCON_IGNORE ] && touch 3d_bbs/COLCON_IGNORE`
  - `cd 3d_bbs && mkdir -p build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc) && sudo make install`
  - 冪等性を確保し、再実行しても安全にする。
- [ ](任意)`.github/workflows/ci.yml` で `nvidia/cuda:12.4.0-devel-ubuntu22.04` ベースに setup.sh + colcon build を回す。
- DoD: クリーンな Ubuntu 22.04 上で `bash scripts/setup.sh && colcon build --packages-select bbs3d_ros2` が通る。

### Step 7 — エンドツーエンド動作確認  🟡 未着手
- [ ] `colcon build --packages-select bbs3d_ros2` をクリーンに通す(`CMakeLists.txt` で Release がデフォルト)。
- [ ] 上流 test data を使って `ros2 launch bbs3d_ros2 bbs3d_rviz2.launch.py` を起動し、`[ROS2] 3D-BBS initialized` を確認する。
- [ ] `ros2 service call /bbs3d_ros2_node/localize std_srvs/srv/Trigger {}` で `/global_pose`、`/src_points_on_global_pose`、`/score`、`/time` が上流挙動と一致して出力されることを確認する。
- [ ] `/click_loc` Bool 後方互換経路でも同じ結果が得られることを確認する。
- [ ] 上流との挙動差分があれば README に明記する。
- DoD: 上流 `gpu_ros2_test_rviz2_launch.py` のデモと同等の動作 + 新 Service トリガが動作する。

## 上流の主な参照
| パス(上流 `3d_bbs/` からの相対) | 役割 |
|---|---|
| `CMakeLists.txt` | トップレベルの非 ament CMake。`sudo make install` で `/usr/local` に配置。 |
| `bbs3d/include/gpu_bbs3d/bbs3d.cuh` | ラッパーが使う公開 API。 |
| `bbs3d/include/pointcloud_iof/{pcl_eigen_converter,pcd_loader,gravity_alignment}.hpp` | ラッパーが使うユーティリティ。 |
| `ros2_test/rviz2/include/ros2_test_rviz2.hpp` | `wrapper_node.hpp` の移植元。 |
| `ros2_test/rviz2/src/gpu_bbs3d_rviz2/gpu_ros2_test_rviz2.cpp` | `wrapper_node.cpp` の移植元。 |
| `ros2_test/rviz2/launch/gpu_ros2_test_rviz2_launch.py` | 参照 launch。 |
| `ros2_test/rviz2/rviz2_config/rviz2.rviz` | 参照 rviz 設定。 |
| `ros2_test/config/ros2_test.yaml` | 参照 config スキーマ。 |
| `ros2_test/rviz2/cmake/Findgpu_bbs3d.cmake` | 既に本パッケージ `cmake/` にコピー済。 |
| `ros2_test/click_loc/` | `/click_loc` Bool を発行する Tk ボタン — 本ラッパーでは Service に置き換え。 |

## 上流に issue を投げる候補(本リポジトリでは触らない)
- **IMU 時刻差計算のバグ**: `3d_bbs/ros2_test/rviz2/src/gpu_bbs3d_rviz2/gpu_ros2_test_rviz2.cpp:190-193` で `imu_t - cloud_t` を計算しているつもりが sec と nanosec の符号が混ざっている。`std::abs(imu_t - cloud_t)` の意図に対し、実装は概ね `std::abs((imu_sec - cloud_sec) + (imu_nano + cloud_nano)*1e-9)` 相当。
- **コールバック共有状態の mutex 不在**: `source_cloud_msg_` と `imu_buffer` がロック無しで参照される。MultiThreadedExecutor で起動された場合に競合する可能性。

## オープンクエスチョン
- ターゲットマシン(Jetson Orin? ワークステーション?)— bbs3d を将来内製ビルドする場合の CUDA arch フラグに影響する。
- `bbs3d_ros2` を rosdep に登録するかどうか — v0.1 の対象外。

---

# 提案: 運用拡張仕様 (要レビュー)

> **このセクションはレビュー待ちの提案です。** 上記 Step 0〜7 (最小ラッパー方針) は変更しません。本提案は **Step 8 以降の追加拡張** として位置付け、既存方針 (上流挙動を素直に維持) と整合させた形で書いています。承認されたら下に「Step 8〜11」を正式採用、Step 7 完了後に着手する想定です。

## 動機

実機運用 (Hesai JT128 + 60m 屋外地図) を進める中で、最小ラッパーでは以下のギャップが顕在化した:

1. **target は PCD フォルダ固定**。SLAM 出力点群を topic 経由で投入する経路がない。エレベーター跨ぎなど地図切替ユースケースに対応できない。
2. **`/tar_points` の VOLATILE QoS と動的 viewer TF**。Rviz2 を後から起動すると地図と TF が表示されない (本セッションで実機検証済み、`gpu_ros2_test_rviz2.cpp:38, :91-118`)。
3. **trigger 結果通知が分散**。service は同期返信できるが、topic 経路 (`/click_loc` Bool) は fire-and-forget で結果を取れない。
4. **エラーで std::cout して握りつぶす**。`pcd_path` 不在・LiDAR 未受信時にノードが silent に動かなくなる。観測手段がない。
5. **トピック名のハードコード**。`/tar_points` `/global_pose` `/score` `/time` は固定。複数台運用や namespace 整理ができない。
6. **Tk GUI 必須**。CI / headless / 自動化テストで邪魔。

これらは upstream `ros2_test_rviz2` の限界であり、「サンプル動作確認」では問題ない。**運用ノードとしては別物のスコープ**。Step 0〜7 で最小ラッパーを完成させてから、これらを順次解決する。

## 既存方針との整合

既存 `CLAUDE.md` の規約「上流挙動を素直に維持する」「`3d_bbs/` 以下は触らない」「装飾的な抽象化は書かない」は **本提案でも厳守する**:

- 上流 `gpu_bbs3d` ライブラリ (`/usr/local`) の API は変更しない、利用するだけ
- 上流 `ros2_test_rviz2` のソースは引き続き **読まない・コピーしない**。本ラッパーは独立に書く (Step 3 で既に行っている移植コピーは尊重し、追加機能は wrapper_node に **足す**形)
- 抽象化は段階的にのみ導入。Step 8 で必要になった分だけ
- mutex 導入や IMU 時刻差バグ修正は引き続き「上流に issue 候補」のまま放置 (本ラッパー内では触らない)

ただし以下は **本ラッパー固有の振る舞い**として導入する (上流挙動の維持と矛盾しない):

- 新規追加する機能 (status loop、target topic mode、result topic) は `~/click_loc` 互換経路を壊さず、追加経路として共存
- 設定ファイル形式は **上流 `ros2_test.yaml` の上位互換**。既存キーはそのまま、新キーを追加するのみ
- エラー時の挙動変更 (fatal → status へ反映) は本ラッパー独自。上流は変えない

## 仕様サマリ (本セッションで合意済み)

1. **target source 切替** — `target.source: "pcd" | "topic"` を yaml で選択
2. **target topic mode の動的更新** — メッセージ受信ごとに voxelmap 再構築。`target.lock_first_message: true` で 1 メッセージ固定もサポート
3. **dual trigger** — service `~/trigger_localize` (`std_srvs/srv/Trigger`、同期返信) と topic (`std_msgs/Bool`、fire-and-forget、結果は `~/trigger_result` topic に publish) の両対応。enable フラグで個別 ON/OFF
4. **status state machine** — `~/status` (`std_msgs/String`、key=value 形式) を 1 Hz 常時 publish。state ∈ `{INITIALIZING, TARGET_LOADING, TARGET_ERROR, WAITING_INPUTS, READY, LOCALIZING, LOCALIZE_FAIL_TIMEOUT, LOCALIZE_FAIL_THRESHOLD, CONFIG_ERROR}`
5. **fatal terminate 廃止** — PCD 不在、YAML 不正、CUDA 例外、いずれもノードは spin 継続。状態は status に反映
6. **トピック・サービス・frame の yaml リネーム** — 全入出力を yaml で指定可能 (`topics.lidar`, `topics.imu`, `topics.target_in`, `topics.pose_out`, ..., `services.trigger`, `frames.map`, `frames.base`, `frames.lidar`)
7. **TRANSIENT_LOCAL `~/target_points`** — Rviz2 後発接続でも地図表示 (本ラッパー内で完結。上流は変えない)
8. **launch に rviz2 同梱**、`use_rviz` 引数で headless 切替

詳細スキーマと実装の核は本提案 PR には書かず、Step 8 以降の各 PR で TDD サイクル経由で固める。

## 提案する追加 Step

> 着手は **Step 7 完了後**。各 Step は `DEVELOPMENT_WORKFLOW.md` の TDD フローに従い、1 Step = 1 ブランチ = 1 PR。

### Step 8 — 設定 yaml の拡張 + 全トピック・frame リネーム  🟡 提案
- [ ] `config/bbs3d_ros2.yaml` に `topics.*`, `services.*`, `frames.*` セクションを追加 (既存キーは互換維持)
- [ ] `wrapper_node.cpp` の publisher / subscriber 生成箇所を yaml キー参照に置き換え
- [ ] launch_testing でリネーム後の topic から localize 結果が取れることを TDD
- DoD: `topics.lidar: "foo"` に変えると wrapper が `/foo` を subscribe し、既存の挙動と一致する

### Step 9 — `/tar_points` を TRANSIENT_LOCAL + Rviz2 後発接続テスト  🟡 提案
- [ ] `tar_points_pub_` を `rclcpp::QoS(1).transient_local()` に変更
- [ ] launch_testing で「ノード起動 → 数秒待ち → 別 subscriber を起動 → /tar_points を 1 件受信できる」を TDD
- [ ] 同様に viewer TF を `tf2_ros::StaticTransformBroadcaster` へ置換
- DoD: Rviz2 を後から起動しても target_points 表示・viewer TF 解決ができる

### Step 10 — Status loop + エラー耐性  🟡 提案
- [ ] `Status` enum と key=value 形式の文字列化ユーティリティ追加 (`include/bbs3d_ros2/status.hpp`)
- [ ] `~/status` を 1 Hz publish するタイマ追加
- [ ] PCD load 失敗時に **fatal せず** state=`TARGET_ERROR` を立てて spin 継続
- [ ] LiDAR/IMU の最終受信時刻を保持し、freshness を status に反映
- [ ] localize の例外を try/catch で受け、state を反映 (LOCALIZE_FAIL_*)
- [ ] launch_testing で「PCD 不在で起動 → state=TARGET_ERROR を 1 Hz で publish し続ける」を TDD
- DoD: あらゆる起動失敗・実行時失敗で**ノード自体は落ちない**。state が外部から購読できる

### Step 11 — Dual trigger + result topic  🟡 提案
- [ ] `enable_trigger_topic` / `enable_trigger_service` の yaml フラグ追加
- [ ] `~/trigger_localize` topic (`std_msgs/Bool`) と service (`std_srvs/srv/Trigger`) を共通の `do_localize()` 関数で統一実装 (既存 `click_callback` から共通処理を抽出)
- [ ] `~/trigger_result` topic (`std_msgs/String`) を追加し、topic 経路の結果通知を提供
- [ ] launch_testing で「topic publish → 結果が `~/trigger_result` に出る」「service call → response が同等内容」の両方を TDD
- DoD: 自律ノードからは service で同期、軽量スクリプトからは topic で fire-and-forget。`/click_loc` 後方互換も維持

### Step 12 — Target topic mode (PCD 以外の経路)  🟡 提案
- [ ] `target.source: "topic"` で `topics.target_in` を subscribe
- [ ] `target.lock_first_message: true` で 1 メッセージ固定、`false` (既定) で受信ごとに voxelmap 再構築
- [ ] 再構築は別スレッドで実行し、構築中は state=`TARGET_LOADING`、localize 受付は **古い target で継続**可能 (atomic swap 設計)
- [ ] GPU メモリの並行使用量を確認し、本番地図サイズ (60m / max_level=6) で問題ないことを確認
- [ ] launch_testing で「topic から target を投入 → localize 成功」「2 種類の target を順に投入 → 結果が切り替わる」を TDD
- DoD: SLAM 出力やエレベーター跨ぎユースケースで PCD ファイル無しに localize できる

## リスク・考慮事項

- **GPU メモリの並行使用** (Step 12): voxelmap atomic swap 中は新旧 2 インスタンスを同時保持。60m 地図なら問題ないはずだが、Step 12 着手時に実測してから設計を確定する
- **MultiThreadedExecutor の必要性** (Step 11): service コールバック内で localize (~200ms) が走るため、SingleThreadedExecutor だと spin がブロックされて status loop が止まる。Step 10 着手時に切替を併せて検討
- **CUDA context のスレッド安全性** (Step 12): voxelmap 再構築を別スレッドで行う場合、CUDA context の取り扱いに注意。`gpu_bbs3d::BBS3D` のデストラクタが安全に動くかを Step 12 着手前に最小サンプルで確認
- **TDD の launch_testing 重さ**: launch_testing で `rclpy.init` が立ち上がるたびに数秒かかる。テスト数が増えると CI が長くなる。1 Step あたりの新テストは 1〜3 ケースに絞る
- **既存 RED テスト (`test_localize_service.py`) との関係**: Step 3 で進行中の `~/localize` Trigger Service 実装は本提案の `~/trigger_localize` service と重複する。Step 11 着手時に **同じエンドポイント**として整理 (リネームか別 service として両立か) を判断

## 採用しなかった選択肢

- **完全 fork + リネーム**: 上流追従できなくなる、上流リスペクト方針に反する
- **3d_bbs/ros2_test 配下を直接改造**: `CLAUDE.md` の「`3d_bbs/` 以下は触らない」原則に反する
- **独自 srv/msg パッケージ (`bbs3d_msgs`) を切る**: パッケージ数が増えビルド・依存が複雑化。`std_srvs/Trigger` + `std_msgs/String` (key=value) で十分

## 進め方の確認事項 (本 PR でレビューしてもらいたい点)

1. Step 8〜12 の **粒度と順序**は妥当か (細かすぎ・粗すぎ)
2. Step 11 で `~/trigger_localize` を新設するとき、Step 3 の `~/localize` をどう整理するか (リネーム / 別経路として両立)
3. Step 12 の **target topic mode** を本リポジトリのスコープに入れるか、別パッケージ (`bbs3d_ros2_advanced` 等) に分離するか
4. status state machine の状態名・粒度は妥当か (簡素化要望があれば)
5. これら全てを完了させたあと **README をどこまで書き直すか** (上流デモ手順 → 運用手順の比重変更)
