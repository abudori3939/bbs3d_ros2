# bbs3d_ros2 実装計画

このドキュメントは `bbs3d_ros2` の構築作業の唯一の真実 (source of truth) です。前提知識のないセッション(別の Claude セッションを含む)が、最初の未完了項目から作業を再開できることを意図して書かれています。

> **開発フローのルールは [`DEVELOPMENT_WORKFLOW.md`](DEVELOPMENT_WORKFLOW.md) を参照してください。** すべての変更は PR 経由で main にマージし、各 Step の checkbox 更新もその PR 内で行います。

## 目的
[KOKIAOKI/3d_bbs](https://github.com/KOKIAOKI/3d_bbs) の GPU 全探索グローバル位置推定を、**`ros2_test/rviz2` パッケージを移植 (port) して `colcon build` 可能な ament パッケージにする** ROS 2 (humble) ノード。動機:

- 上流の `ros2_test/rviz2` はそのままでは `colcon_ws` でビルドできない(CMake 構成・パッケージ名・依存解決などが colcon と非互換)。
- 本パッケージはそのソースを本リポジトリ `src/` 以下に移植し、一般的な ROS 2 ノードと同じ手順で `colcon build --packages-select bbs3d_ros2` でビルド・起動できるようにする。
- 移植後は使いにくい点(トリガが Tk GUI 固定、トピック名がハードコード、target が PCD 固定、エラーが出にくい)を段階的に改善する。

配布モデル:

1. ユーザはこのリポジトリを `~/colcon_ws/src/` に **再帰 clone** する(`git clone --recursive ...`)。
2. ユーザは同梱の `3d_bbs/` submodule で `cmake && make && sudo make install` を **手動で 1 回だけ** 実行する(`libgpu_bbs3d.so` とヘッダを `/usr/local` 以下に配置)。
3. ユーザは `colcon build --packages-select bbs3d_ros2` でビルドし、launch を起動する。

> **本パッケージは 3d_bbs のインストールを支援しない**(setup スクリプトなどは置かない)。
> **上流 `3d_bbs/` の構造改変もしない**(ファイル編集・パッチ当ては禁止。必要があれば上流に issue / PR)。

## 命名
- パッケージ名(`package.xml` 内): **`bbs3d_ros2`** — REP 144 により先頭は英字必須。上流の識別子(`gpu_bbs3d`、namespace `gpu`、`bbs3d/include/`)と整合する。
- C++ クラス名: **`bbs3d_ros2::Bbs3dNode`**(`include/bbs3d_ros2/bbs3d_node.hpp`)。
- 実行ファイル名: **`bbs3d_ros2_node`**。
- GitHub リポジトリ名: `bbs3d_ros2` でも `3d_bbs_ros2` でも可。パッケージ名は固定。

## レイアウト
```
bbs3d_ros2/
├── package.xml
├── CMakeLists.txt
├── cmake/Findgpu_bbs3d.cmake     # /usr/local 配下の gpu_bbs3d を探す(任意)
├── cmake/Findcpu_bbs3d.cmake     # 同 cpu_bbs3d(必須)
├── include/bbs3d_ros2/bbs3d_node.hpp
├── include/bbs3d_ros2/bbs3d_backend.hpp  # GPU/CPU 実装の仮想インタフェース
├── src/bbs3d_node.cpp
├── src/bbs3d_node_main.cpp
├── src/bbs3d_backend.cpp         # factory(#ifdef BBS3D_HAS_GPU はここだけ)
├── src/bbs3d_backend_cpu.cpp
├── src/bbs3d_backend_gpu.cpp     # GPU 入りビルドのみコンパイル
├── launch/bbs3d_rviz2.launch.py
├── config/bbs3d_ros2.yaml
├── rviz/bbs3d.rviz
├── docs/IMPLEMENTATION_PLAN.md   # 本ファイル
├── docs/DEVELOPMENT_WORKFLOW.md
├── CLAUDE.md                     # 次セッションの Claude 向けブリーフィング
├── README.md
└── 3d_bbs/                       # git submodule。COLCON_IGNORE を含む
```

## 設計判断

### 共通方針(フェーズ A / B 共通)
1. **上流は submodule + COLCON_IGNORE**。 ament パッケージは 1 つに保ち、上流のネストされた ament パッケージを colcon が走査しないようにする。
2. **`gpu_bbs3d` / `cpu_bbs3d` は `/usr/local` インストール経由で発見**。 本パッケージは CUDA を再コンパイルしない。`find_package(gpu_bbs3d)` / `find_package(cpu_bbs3d)` は本リポジトリ `cmake/Find*.cmake`(上流からコピー)で解決する。Step 11 以降、`cpu_bbs3d` は必須、`gpu_bbs3d` は任意(見つかったときだけ GPU 実装をコンパイル)。
3. **本 CMake で CUDA 言語は有効化しない**。 `libgpu_bbs3d.so` をリンクし CUDA ヘッダ(`<cuda_runtime.h>`、thrust)を include するだけなので、`find_package(CUDA)`(legacy module)で十分。
4. **`3d_bbs/` 配下は触らない**。 構造改変・インストール支援スクリプトも含めて行わない。
5. **設定は yaml 一本**。 上流の `ros2_test.yaml` スキーマを踏襲し、ROS 2 パラメータでの上書きは入れない。フェーズ B で yaml 項目は増やすが、入口は yaml に固定。

### フェーズ A — 上流互換 port
6. **上流挙動を厳密に維持**。 クラス名 / namespace / 実行ファイル名 / ファイル配置のみ変える。mutex 不在による潜在的レース、IMU 時刻差バグ(`gpu_ros2_test_rviz2.cpp:190-193` の sec/nanosec 符号混在)など上流の課題は本フェーズでは触らず、必要に応じて上流に issue を投げる(下記「上流に issue を投げる候補」参照)。
7. **トリガは `/click_loc` Bool トピックのみ**。 上流の Tk GUI クライアント(`ros2_test/click_loc/`)は移植しない。ユーザは `ros2 topic pub /click_loc std_msgs/msg/Bool "{data: true}"` でトリガする。

### フェーズ B — 拡張(意図的に上流から逸脱)
8. **トリガに Service 版 (`~/localize`, `std_srvs/srv/Trigger`) を追加**。 Bool トピックは上流互換のため引き続きサポート。Service 版は確実な通信が必要な自律システム向け。
9. **全トピック名(出力含む)+ トリガトピック名を yaml で変更可能にする**。 デフォルト値は上流互換。
10. **target 点群は PCD / topic の 2 モードを yaml で切替**。 `target_source_mode: "pcd" | "topic"`。topic モードは latched (`transient_local` QoS) で受信し、地図ホットスワップ(動作中の地図切替)を可能にする。
11. **エラーハンドリングを構造化**。 `std::cout` を `RCLCPP_INFO/WARN/ERROR` に置換、起動時に PCD パス・読み込み件数を診断、ランタイムは `/scan` `/imu` 未受信を周期 WARN、localize 結果の成否理由を構造化ログに出す。
12. **GPU / CPU 実装を 2 層で切替**(Step 11)。 GPU 実装を含めるかは CMake がビルド時に自動判定(CUDA + `libgpu_bbs3d.so` の有無)し、どちらを使うかは yaml `backend` で実行時に決める。ユーザ手順(`colcon build` / launch / yaml)は GPU 機・非 GPU 機で同一に保つ。ノードは仮想インタフェース越しにしか BBS3D を触らず、スカラは double に統一する(float 変換は GPU 実装側に閉じ込め)。

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

---

## フェーズ A — 上流互換 port

> 目標: `ros2_test/rviz2` と同等の動作を、`colcon build` 可能な ament パッケージとして再現する。

### Step 3 — ノード移植  ✅ 完了
**TDD 適用**(機能追加に相当するため)。ただし上流互換が DoD のため、テスト粒度は「上流挙動と等価な I/O が出ること」を検証する integration test 1〜数件で足りる(`launch_testing` 推奨)。

- [x] `include/bbs3d_ros2/bbs3d_node.hpp`
  - 上流 `3d_bbs/ros2_test/rviz2/include/ros2_test_rviz2.hpp` をベースに作る。
  - クラス名 `Bbs3dNode`、`namespace bbs3d_ros2` で囲う。
  - **Service ハンドラはまだ追加しない**(Step 7 で追加)。
- [x] `src/bbs3d_node.cpp`
  - 上流 `3d_bbs/ros2_test/rviz2/src/gpu_bbs3d_rviz2/gpu_ros2_test_rviz2.cpp` を素直に移植(挙動互換)。クラス名 / namespace のみ書き換える。
  - mutex 保護や IMU バグ修正、ROS 2 パラメータ上書きなど **上流に存在しない振る舞い変更は入れない**。
- [x] `src/bbs3d_node_main.cpp` — 薄い `rclcpp::init` + `spin(std::make_shared<Bbs3dNode>())`。
- [x] `CMakeLists.txt` — executable target を追加:
  ```cmake
  find_package(CUDA REQUIRED)
  find_package(gpu_bbs3d REQUIRED)
  find_package(PCL REQUIRED)
  find_package(Eigen3 REQUIRED)
  find_package(yaml-cpp REQUIRED)

  ament_auto_add_executable(bbs3d_ros2_node
    src/bbs3d_node_main.cpp
    src/bbs3d_node.cpp
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

### Step 4 — launch / config / rviz  ✅ 完了
launch / rviz / config 設定のみの変更。TDD 対象外。

- [x] `launch/bbs3d_rviz2.launch.py`
  - `get_package_share_directory('bbs3d_ros2')` で解決する。
  - `DeclareLaunchArgument('config_file', default_value=<share>/config/bbs3d_ros2.yaml)`。
  - 2 ノード: `rviz2`(`-d <share>/rviz/bbs3d.rviz`)と `bbs3d_ros2_node`。
  - **Tk `click_loc` ノードは含めない**。
- [x] `config/bbs3d_ros2.yaml` — 上流 `ros2_test/config/ros2_test.yaml` のコピー。`target_clouds: "your_path/target"` のまま、上書き手段(launch 引数 `config_file:=`)をコメントで案内。
- [x] `rviz/bbs3d.rviz` — 上流 `ros2_test/rviz2/rviz2_config/rviz2.rviz` のコピー。
- DoD: `ros2 launch bbs3d_ros2 bbs3d_rviz2.launch.py` で上流デモと同じ Display 構成の RViz が起動する。

### Step 5 — README + LICENSE  ✅ 完了
ドキュメントのみ。TDD 対象外。

- [x] `README.md` を拡充:
  - 前提条件(Ubuntu 22.04、ROS 2 humble、CUDA 12.0+、Eigen 3.4+、NVIDIA GPU)。
  - 4 ステップの導入・ビルド・実行手順(clone → 3d_bbs sudo make install → test data 配置 → colcon build)。
  - **太字注意:** 本パッケージは 3d_bbs のインストールを支援しない。ユーザが手動で `cd 3d_bbs && cmake .. && make && sudo make install` を行う。
  - **太字注意:** `git submodule update` 後は必ず `sudo make install` を再実行。
  - test data 配置の案内(`bbs3d_ros2/data/target/` `bbs3d_ros2/data/ros2_test_data/`、上流 `3d_bbs/ros2_test/ros2_test_code.md` の Google Drive リンクを参照)。
  - トリガ手順: `ros2 topic pub --once /click_loc std_msgs/msg/Bool "{data: true}"`。
  - 動作デモのフルフロー(config 編集 → ros2 launch → rosbag 再生 → /click_loc トリガ → 結果確認)。
  - トピック / TF の I/F 一覧表(Service 行は **フェーズ B 完了後に追記**)。
  - 設定スキーマ表、トラブルシューティング表。
- [x] `LICENSE` — MIT。Copyright (c) 2026 abudori3939。
- [x] README で上流 `KOKIAOKI/3d_bbs` への謝辞 + 元論文の引用を明記。
- DoD: 初見のユーザが README だけで動作デモまで到達できる。

### Step 6 — エンドツーエンド動作確認(フェーズ A 完了)  ✅ 完了
動作確認 + 既存スモークテストへの env var gating 追加(コード変更なので plan mode 対象、ただし TDD は対象外)。

- [x] `colcon build --packages-select bbs3d_ros2` をクリーンに通す(`CMakeLists.txt` で Release がデフォルト)。
- [x] 上流 test data を使って `ros2 launch bbs3d_ros2 bbs3d_rviz2.launch.py` を起動し、`[ROS2] 3D-BBS initialized` を確認する。**手元実行ログを PR 本文に貼付**。
- [x] `ros2 topic pub --once /click_loc std_msgs/msg/Bool "{data: true}"` で `/global_pose`、`/src_points_on_global_pose`、`/score`、`/time` が上流挙動と一致して出力されることを確認する。**手元実行ログを PR 本文に貼付**。
- [x] 上流との挙動差分があれば README に明記する。
- [x] **`BBS3D_REQUIRE_TEST_DATA=1` env var gating を追加**(PR #4 review 反応の commit):
  - default(unset): data 未配置で smoke を skip(現状維持、ローカル開発を妨げない)
  - `=1` セット: data 未配置で smoke を fail(CI / 厳格モード)
  - 3 シナリオ(data あり + env var、data 無し + env var、data 無し + env var なし)で動作確認
- DoD: 上流 `gpu_ros2_test_rviz2_launch.py` のデモと同等の動作が再現できる + ローカル実行ログの貼付 + env var gating の動作確認(フェーズ A 完了)。

---

## フェーズ B — 拡張(使いにくさの改善)

> ここから機能追加。各 Step は **TDD フロー**(`DEVELOPMENT_WORKFLOW.md` 参照)に厳格に従う。
> RED → ユーザ承認 → GREEN → ユーザ承認 → PR の順。ビルドエラーは RED として認められない。

### Step 7 — Service トリガ `~/localize` を追加 + Bool topic リネーム  ✅ 完了
**TDD 適用**。

- [x] `Bbs3dNode` に `rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr localize_srv_` を追加。Service 名は `~/localize`(完全修飾は `/bbs3d_ros2_node/localize`)。
- [x] **Bool トリガ topic を `/click_loc` → `~/localize` にリネーム**(Tk クリック GUI の名残整理)。Topic と Service は ROS 2 で別名前空間のため同名で共存。
- [x] `click_callback` を `localize_topic_callback` にリネーム、`click_sub_` を `localize_sub_` にリネーム。`click_callback` の本体を `run_localization()` に切り出し、`localize_topic_callback` と `localize_srv_callback` の両方から呼ぶ。
- [x] Service レスポンス: 成功時 `success=true, message=""`。失敗時は `success=false, message=<reason>`(`"point cloud not received"`、`"imu not received"`、`"localization timed out"`、`"score below threshold"`)。
- [x] テスト: `launch_testing` + `rclpy` で `~/localize` を入力なしで呼び、`response.message` に `"not received"` を含むことを検証(`test/test_localize_service.py`)。
- [x] 副作用として `localize_topic_callback` の stdout 出力も正規化文言に変更(上流 `"point cloud msg is not received"` 等から `"point cloud not received"` 等へ)。フェーズ B 逸脱として許容。
- DoD: `ros2 service call /bbs3d_ros2_node/localize std_srvs/srv/Trigger {}` で Bool トリガと同じ結果が得られる。

### Step 8 — エラーハンドリング・ロギング強化  ✅ 完了
**TDD 適用**。

- [x] `std::cout` / `std::cerr` を全て `RCLCPP_INFO/WARN/ERROR` に置換(全 23 箇所)。装飾的な `*=*=*` 囲み枠も削除。
- [x] 起動時診断:
  - PCD パスを `std::filesystem::weakly_canonical` で絶対パスに展開してログ出力。
  - 件数(`tar_cloud_ptr->size()`)+ 所要時間を INFO で出力(例: `"Target clouds loaded: N points in X ms"`)。
  - 失敗時は `RCLCPP_ERROR` + `throw std::runtime_error` → `main` で catch → `[Fatal]` を stderr に出して `exit(1)`。これまでの bad PCD → segfault を解消。
- [x] ランタイム警告:
  - `lidar_last_received_` / `imu_last_received_` を `cloud_callback` / `imu_callback` で `state_mutex_` 配下に更新。
  - `run_localization` 冒頭で snapshot を取り、最終受信が 3s 以上前(または未受信、`nanoseconds() == 0`)なら `RCLCPP_WARN`(例: `"lidar topic stale (last %.1fs ago) — localize may use stale data"`)。
  - localize 結果は成功時 `RCLCPP_INFO("Localize: success (score=%d, time=%.1f ms)")`、失敗時は `localize_topic_callback` で `RCLCPP_WARN("Localize: <reason>")`。Service 側は response.message に reason を載せる(Step 7)。
- [x] PR #8 review carry-over:
  - `mutable std::mutex state_mutex_` で `source_cloud_msg_` / `imu_buffer` / `*_last_received_` を保護(reviewer #1)。
  - `get_nearest_imu_index` の `stamp` 引数を実際に使う形に変更し、上流の sec/nanosec 符号バグも同時に修正(reviewer #6 + Phase B 逸脱許容)。
- [x] **load_target_clouds_pcd() メソッド境界**: Step 10 で `if (target_source_mode == "pcd")` でラップする想定の前準備として、PCD ロード + voxelmap 構築をメソッドに切り出し。
- [x] テスト: `test/test_bad_pcd_path.py`(launch_testing)で不正パス起動時に ERROR ログ + exit code 1 を検証。
- [ ] **未対応(将来 follow-up PR)**:
  - Service callback の callback_group 分離(PR #8 review #2): 構造的変更で動作テストが難しいため、Step 10 完了後の clean-up PR で扱う候補。
  - `BBS3D_REQUIRE_TEST_DATA=1` env var gating の自動テスト: data 退避が必要で重い、Step 10 完了後に検討。
- DoD: 起動失敗・ランタイム異常の何が起きたか、**ログだけで追える** ✅(`test_bad_pcd_path` で証跡あり)。

### Step 9 — トピック名の config 化  ✅ 完了
**TDD 適用**。

- [x] `config/bbs3d_ros2.yaml` に出力トピックとトリガトピックの項目を追加(Optional セクションにコメントアウト形式で記載、外せばリネーム反映)。デフォルトは上流互換:
  - `tar_points_topic_name`(default `/tar_points`)
  - `src_points_on_global_pose_topic_name`(default `/src_points_on_global_pose`)
  - `global_pose_topic_name`(default `/global_pose`)
  - `score_topic_name`(default `/score`)
  - `time_topic_name`(default `/time`)
  - `localize_topic_name`(default `~/localize`、Step 7 でリネーム済。Bool topic と Trigger service の両方に適用)
- [x] `Bbs3dNode` で yaml から読み込み、サブスクリプション / パブリッシャーを設定。`get_or(YAML::Node, key, default)` ヘルパで「key が無ければ default」とし、既存ユーザの yaml(これらのキーが無い)が壊れない設計にした。
- [x] テスト: `test/test_topic_name_config.py` で fixture yaml に 6 項目をリネーム指定して起動 → ROS graph の `get_topic_names_and_types` / `get_service_names_and_types` で renamed 名が現れること、default 名が消えていることを assert。
- [ ] **未対応(将来 follow-up PR)**:
  - 空文字 yaml 値の防御(PR #10 review): `tar_points_topic_name: ""` 等で `create_publisher("", ...)` が rclcpp 例外を吐く。`get_or` を空文字も default 扱いするか、`load_config` 全体で空文字を明示 RuntimeError にするか、設計判断が必要。`lidar_topic_name` / `imu_topic_name` 等既存項目も同じ問題を抱えるため、Step 9 単独で局所最適化せず、Step 10 完了後の cleanup PR で `load_config` 全体を強化する候補。
- DoD: 全トピック名がデフォルト動作を変えずに yaml で変更可能 ✅。

### Step 10 — target 点群の topic モード対応(地図ホットスワップ)  ✅ 完了
**TDD 適用**。

- [x] `config/bbs3d_ros2.yaml` に以下を追加(Optional セクションにコメントアウト形式で記載、外せば反映):
  - `target_source_mode: "pcd"` または `"topic"`(default `pcd`)
  - `target_cloud_topic_name`(default `/target_cloud`、topic モードのみ参照)
- [x] `pcd` モード:既存挙動(起動時に `target_clouds` パスから PCD ロード)。回帰なし(smoke / bad_pcd_path / localize_service / topic_name_config 全 PASS で確認)。
- [x] `topic` モード:
  - `transient_local`+`reliable` QoS で `target_cloud_topic_name` を sub。
  - 受信のたびに `pcl::VoxelGrid`(`tar_leaf_size > 0` 時)→ `gpu_bbs3d.set_tar_points()` + `set_trans_search_range()` で voxelmap を再構築(地図ホットスワップ)。echo は `tar_points_topic_name` に **downsample 後の点群** を `pcl::toROSMsg` で publish(pcd モードと同じ不変条件:`<tar_points_topic_name>` に流れる cloud = voxelmap に登録された cloud)。
  - 起動時は `tar_points_loaded_ = false` で待機し、未受信状態で `~/localize` が来たら `response.message == "target map not loaded"` を返す。
  - 再構築中の localize は `bbs3d_mutex_` を `lock_guard` で取得するため、callback 完了まで blocking で待つ(API は「localize 呼出は再構築完了まで待つ」というシンプルな約束)。
- [x] **Mutex 設計**: 既存 `state_mutex_`(短時間 lock 用)とは別に `bbs3d_mutex_` を導入。上流 `gpu::BBS3D` が thread-safe でないため、`set_tar_points` / `localize` の並行実行をクライアント側で完全に排他する責務を負う。`target_cloud_callback` と `run_localization` は共に `lock_guard` で取得し、再構築中の localize は callback 完了まで block。SingleThreadedExecutor 下では callback 直列化で競合は構造的に発生しないが、将来 MultiThreaded 化された場合の防御として lock を残す。
- [x] テスト: `test/test_target_source_mode.py`(topic モードで起動 → graph に sub が現れる / target 未受信状態の localize で `"target map not loaded"` reason / target publish 後にガードを抜けて別 reason に遷移)。専用 fixture `bbs3d_ros2_test_topic_mode.yaml` を分離し、PCD ファイル不要で CI で常に実行される。
- [x] **PR #11 レビュー対応 follow-up**(2026-05-12): 初版で導入した `unique_lock(try_to_lock)` + `"target map reloading"` reason は、SingleThreadedExecutor + default callback group では到達不能であることが判明したため、`lock_guard` で待つ semantics に変更し reloading reason を削除。同時に topic モードの echo を downsample 後の点群に揃え(pcd 側と一致)、テスト fixture を分離して PCD 依存を撤去。
- [x] **PR #11 follow-up: target_cloud QoS yaml 化**(2026-05-12): 実機検証で `pcl_ros` 等 REP-2003 非準拠 publisher(`volatile`+`reliable`、設定変更不可)と接続できない問題が判明。`target_cloud_qos_reliability` / `target_cloud_qos_durability` の 2 軸を yaml で切替可能化(`get_or` で optional 読込、文字列→`rclcpp::*Policy` enum 変換ヘルパ追加)。default は REP-2003 Maps 推奨(`reliable`+`transient_local`)を維持し既存ユーザ影響なし。TDD で進行(RED → GREEN)。テスト `test_target_qos_config.py` は `get_subscriptions_info_by_topic` で graph 上の sub QoS を assert。
- [ ] **未対応(将来 follow-up、2026-09-10 の実機検証で判明)**: topic モードの `pcl::VoxelGrid` は広域地図 + 小さい leaf でボクセル数が int32 を溢れ、**空の点群を返す**(PCL の仕様)。例: 404 x 430 x 70 m の地図に `tar_leaf_size: 0.1` → cells ≈ 1.2e10 で溢れ、`Received target cloud is empty after downsample, ignoring` になる(0.5 なら 9.8e7 で OK)。現状 WARN は出るので追跡はできるが、「leaf size が小さすぎる」ことを示すメッセージにすると親切。
  **`pcl::ApproximateVoxelGrid` に替えてはいけない** — 上流作者が [KOKIAOKI/3d_bbs#38](https://github.com/KOKIAOKI/3d_bbs/issues/38) で「target 点群への ApproximateVoxelGrid は 3D-BBS の位置推定に悪影響がある。自前の点群を使う場合は `voxel_grid` などを使うこと(空の点群が出ることに注意)」と明言している。topic モードの `pcl::VoxelGrid` はこの推奨に沿っており、変更しない。
- DoD: 動作中に `ros2 topic pub` で地図を切替できる ✅。

### Step 11 — CPU バックエンド対応(GPU 非搭載マシンでの動作)  ✅ 完了
**TDD 適用**(yaml キー追加 = ノードの機能追加のため)。

動機: 本パッケージは `gpu::BBS3D` 決め打ちで、CUDA の無いマシンでは configure 段階で失敗し `colcon build` すら通らなかった。上流 3d_bbs は CPU 実装 (`cpu::BBS3D` / `libcpu_bbs3d.so`) を `BUILD_CUDA` に関係なく常にビルド・インストールしており、API は GPU 版とほぼ同一(差分は namespace と スカラ型 float/double、GPU 専用 `set_branch_copy_size`、CPU 専用 `set_num_threads`)。

- [x] **バックエンド抽象化**: `include/bbs3d_ros2/bbs3d_backend.hpp` に仮想インタフェース `BbsBackend` + `create_backend(BackendKind)` を定義。実装は `src/bbs3d_backend_cpu.cpp` / `src/bbs3d_backend_gpu.cpp`、`#ifdef BBS3D_HAS_GPU` の分岐は `src/bbs3d_backend.cpp` の factory 1 箇所だけ。**インタフェースのスカラは double に統一**し、float への往復キャストは `GpuBackend` に閉じ込めた(`pciof::pcl_to_eigen<T>` はテンプレートなので `Vector3d` でそのまま使える)。本ヘッダは `bbs3d.cuh` / `bbs3d.hpp` を include しないため、CUDA ヘッダ(`cuda_runtime.h` / thrust)がノード側の翻訳単位に漏れない。
- [x] **2 層の切替**:
  - ビルド時(CMake が自動判定): `find_package(CUDA QUIET)` + `find_package(gpu_bbs3d QUIET)` が両方成功したときのみ GPU 実装をコンパイルし `BBS3D_HAS_GPU` を定義。`-DBBS3D_ENABLE_GPU=ON/OFF` で上書き可。configure ログに `-- bbs3d_ros2: GPU backend = ON/OFF` を出す。
  - 実行時(yaml): `backend: "auto" | "gpu" | "cpu"`(default `"auto"`、`get_or` で optional 読込のため既存 yaml は無変更で動く)。`"auto"` は GPU 入りビルドなら GPU。`"gpu"` を CPU のみのビルドで指定したら起動時 ERROR + exit 1。不正値は `load_config` で弾く(`target_source_mode` / QoS 文字列と同じパターン)。
  - 起動時に `3D-BBS backend: GPU|CPU` を INFO ログに出す(どちらで動いているか分からないと実行時切替が使えないため)。
- [x] `cmake/Findcpu_bbs3d.cmake`(上流 `test/cmake/` の逐語コピー)を追加。`cpu_bbs3d` は REQUIRED、`gpu_bbs3d` は QUIET。CPU 実装は OpenMP を使うため `find_package(OpenMP)` + `OpenMP::OpenMP_CXX` をリンク。
- [x] テスト: `test/test_backend_config.py`(`backend: "cpu"` 指定で `3D-BBS backend: CPU` ログ)、`test/test_backend_invalid_value.py`(不正値で ERROR ログ + exit 1)。どちらも topic モード fixture を使うため PCD 不要。GPU 指定 × CPU のみビルドのケースはビルド構成で結果が反転するため自動テストには含めず手動確認(ERROR + exit 1 を確認済み)。
- [x] ドキュメント: README(対応環境 / `-DBUILD_CUDA=OFF` / `sudo ldconfig` / 設定表 / トラブルシューティング)、`config/bbs3d_ros2.yaml`、CLAUDE.md を更新。
- [ ] **未対応(将来 follow-up)**:
  - CPU 実装の `set_num_threads`(上流 default 4 固定)を yaml から設定可能にする。CPU で実機性能を出すには実質必要。
  - GPU 実装側の回帰確認。実装者の開発機に GPU が無いため、GPU ビルドは未検証(コンパイル・動作ともメンテナ環境での確認が必要)。
  - `backend: "auto"` の判定はビルド時のみ。GPU 機でビルドしたバイナリを GPU の見えない環境(`--gpus` 無しコンテナ等)で動かすと GPU 実装のまま起動して CUDA 側で落ちる。`Auto` 分岐に `cudaGetDeviceCount` の実行時プローブを入れて CPU にフォールバックする案がある(現状は README で注意喚起のみ)。
  - `GpuBackend` は double インタフェースからの float 変換で target 点群 1 本分の一時領域を確保する(`set_src_points` も localize ごとに 1 本)。実行時切替と引き換えのコストで、巨大地図ではピークメモリが増える。気になる場合は node 側で double 配列を早期解放するか、GPU 専用ビルドで float 直渡しにする最適化が候補。
  - `broadcast_viewer_frame` は空点群を防御していない(`points[0]` 参照と 0 除算)。上流 `pciof::load_tar_clouds` は「ディレクトリは存在するが `.pcd` が 1 つも無い」場合に true を返すため、その構成で NaN TF を publish しうる。Step 11 の範囲外(既存の挙動)として別 PR で対応。
- DoD: GPU 非搭載マシンで手順を変えずに `colcon build` → `ros2 launch` が通り、localize が成功する ✅(合成地図 + 合成スキャンで `Localize: success (score=182, time=39.3 ms)`、推定 x=3.00 y=-2.00 yaw=0.449 / 真値 x=3.0 y=-2.0 yaw=0.5)。

---

## 上流の主な参照
| パス(上流 `3d_bbs/` からの相対) | 役割 |
|---|---|
| `CMakeLists.txt` | トップレベルの非 ament CMake。`sudo make install` で `/usr/local` に配置。 |
| `bbs3d/include/gpu_bbs3d/bbs3d.cuh` | 本パッケージが使う GPU 版の公開 API(float)。 |
| `bbs3d/include/cpu_bbs3d/bbs3d.hpp` | 同 CPU 版(double)。`BUILD_CUDA` に関係なく常にビルドされる。 |
| `test/src/cpu_test.cpp` | CPU 版の呼び出し順序の参照(Step 11)。 |
| `test/cmake/Findcpu_bbs3d.cmake` | 本パッケージ `cmake/Findcpu_bbs3d.cmake` の移植元。 |
| `bbs3d/include/pointcloud_iof/{pcl_eigen_converter,pcd_loader,gravity_alignment}.hpp` | 本パッケージが使うユーティリティ。 |
| `ros2_test/rviz2/include/ros2_test_rviz2.hpp` | `bbs3d_node.hpp` の移植元。 |
| `ros2_test/rviz2/src/gpu_bbs3d_rviz2/gpu_ros2_test_rviz2.cpp` | `bbs3d_node.cpp` の移植元。 |
| `ros2_test/rviz2/launch/gpu_ros2_test_rviz2_launch.py` | 参照 launch。 |
| `ros2_test/rviz2/rviz2_config/rviz2.rviz` | 参照 rviz 設定。 |
| `ros2_test/config/ros2_test.yaml` | 参照 config スキーマ。 |
| `ros2_test/rviz2/cmake/Findgpu_bbs3d.cmake` | 既に本パッケージ `cmake/` にコピー済。 |
| `ros2_test/click_loc/` | `/click_loc` Bool を発行する Tk ボタン — 本パッケージでは移植しない(`ros2 topic pub` で代替)。 |

## 上流の既知 issue(本リポジトリの挙動に影響するもの)
- **[KOKIAOKI/3d_bbs#38](https://github.com/KOKIAOKI/3d_bbs/issues/38) — `ApproximateVoxelGrid` が 3D-BBS の性能に影響する**(2024-07-09、作者 KOKIAOKI 本人の報告、2026-09-10 時点 open)。要旨:
  - target 点群への `ApproximateVoxelGrid` は **3D-BBS の位置推定に悪影響**を与える。
  - 配布されているテストデータは既に downsample 済みなので影響は出ない。
  - **自前の点群を使う場合は `voxel_grid` などを使うこと**(空の点群が出力されることに注意)。
  - 「次のアップデートで修正予定」とあるが未修正。
  
  本リポジトリへの影響: **pcd モード**は上流 `pciof::load_tar_clouds` を呼ぶため `ApproximateVoxelGrid` が使われる(`tar_leaf_size != 0.0` のとき)。自前地図では `tar_leaf_size: 0.0` にして事前に `voxel_grid` で間引いた PCD を置くか、**topic モード**(`pcl::VoxelGrid` を使う)を選ぶのが安全。`3d_bbs/` は触らない方針のため、本リポジトリ側では上流が修正されるまでドキュメントで回避策を案内する。

## 上流に issue を投げる候補(本リポジトリでは触らない)
- **IMU 時刻差計算のバグ**: `3d_bbs/ros2_test/rviz2/src/gpu_bbs3d_rviz2/gpu_ros2_test_rviz2.cpp:190-193` で `imu_t - cloud_t` を計算しているつもりが sec と nanosec の符号が混ざっている。`std::abs(imu_t - cloud_t)` の意図に対し、実装は概ね `std::abs((imu_sec - cloud_sec) + (imu_nano + cloud_nano)*1e-9)` 相当。
- **コールバック共有状態の mutex 不在**: `source_cloud_msg_` と `imu_buffer` がロック無しで参照される。MultiThreadedExecutor で起動された場合に競合する可能性。

## オープンクエスチョン
- ターゲットマシン(Jetson Orin? ワークステーション?)— bbs3d を将来内製ビルドする場合の CUDA arch フラグに影響する。
- `bbs3d_ros2` を rosdep に登録するかどうか — v0.1 の対象外。
- フェーズ A 完了で v0.1、フェーズ B 完了で v0.2 とするタグ運用 — 別途相談。
