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
├── cmake/Findgpu_bbs3d.cmake     # /usr/local 配下の gpu_bbs3d を探す
├── include/bbs3d_ros2/bbs3d_node.hpp
├── src/bbs3d_node.cpp
├── src/bbs3d_node_main.cpp
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
2. **`gpu_bbs3d` は `/usr/local` インストール経由で発見**。 本パッケージは CUDA を再コンパイルしない。`find_package(gpu_bbs3d)` は本リポジトリ `cmake/Findgpu_bbs3d.cmake`(上流からコピー)で解決する。
3. **本 CMake で CUDA 言語は有効化しない**。 `libgpu_bbs3d.so` をリンクし CUDA ヘッダ(`<cuda_runtime.h>`、thrust)を include するだけなので、`find_package(CUDA REQUIRED)`(legacy module)で十分。
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

### Step 6 — エンドツーエンド動作確認(フェーズ A 完了)  🟡 未着手
動作確認のみ。TDD 対象外。

- [ ] `colcon build --packages-select bbs3d_ros2` をクリーンに通す(`CMakeLists.txt` で Release がデフォルト)。
- [ ] 上流 test data を使って `ros2 launch bbs3d_ros2 bbs3d_rviz2.launch.py` を起動し、`[ROS2] 3D-BBS initialized` を確認する。
- [ ] `ros2 topic pub --once /click_loc std_msgs/msg/Bool "{data: true}"` で `/global_pose`、`/src_points_on_global_pose`、`/score`、`/time` が上流挙動と一致して出力されることを確認する。
- [ ] 上流との挙動差分があれば README に明記する。
- DoD: 上流 `gpu_ros2_test_rviz2_launch.py` のデモと同等の動作が再現できる(フェーズ A 完了)。

---

## フェーズ B — 拡張(使いにくさの改善)

> ここから機能追加。各 Step は **TDD フロー**(`DEVELOPMENT_WORKFLOW.md` 参照)に厳格に従う。
> RED → ユーザ承認 → GREEN → ユーザ承認 → PR の順。ビルドエラーは RED として認められない。

### Step 7 — Service トリガ `~/localize` を追加  🟡 未着手
**TDD 適用**。

- [ ] `Bbs3dNode` に `rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr localize_srv_` を追加。Service 名は `~/localize`(完全修飾は `/bbs3d_ros2_node/localize`)。
- [ ] `click_callback` と Service ハンドラの共通処理を `private` メソッド(例:`run_localization()`)に切り出す。
- [ ] Service レスポンス: 成功時 `success=true, message=""`。失敗時は `success=false, message=<reason>`(例:`"point cloud not received"`、`"imu not received"`、`"localization timed out"`、`"score below threshold"`)。
- [ ] テスト: gtest または `launch_testing` で `~/localize` を呼び、成功 / 各種失敗ケースが期待通り返ることを検証。
- DoD: `ros2 service call /bbs3d_ros2_node/localize std_srvs/srv/Trigger {}` で Bool トリガと同じ結果が得られる。

### Step 8 — エラーハンドリング・ロギング強化  🟡 未着手
**TDD 適用**。

- [ ] `std::cout` / `std::cerr` を全て `RCLCPP_INFO/WARN/ERROR` に置換。
- [ ] 起動時診断:
  - PCD パスを絶対パスに展開してログ出力(どこを読みに行ったか)。
  - パス存在チェック、PCD ファイル件数、合計点数、所要時間を INFO で出力。
  - 失敗時(パス不在、ファイル 0 件、読み込みエラー)は ERROR を出してノードを正しく終了させる(あるいは fatal とする)。
- [ ] ランタイム警告(global localization はトリガー駆動なので、**平常時は静かに**、トリガー時にだけ状態を点検する):
  - lidar / imu の最終受信時刻を保持しておく。
  - **トリガー(`/click_loc` または `~/localize`)受信時** に最終受信時刻をチェックし、いずれかが **3 秒以上前**(未受信を含む)であれば `RCLCPP_WARN` を出す(例: `"lidar topic not received for 3.4s — global localization may use stale data"`)。
  - localize 結果の成否理由を構造化ログ(成功時は score / 実行時間、失敗時は reason)。
- [ ] テスト: 不正な PCD パスを与えたときに ERROR ログが出てノードが正しく終了することを検証。
- DoD: 起動失敗・ランタイム異常の何が起きたか、ログだけで追える。

### Step 9 — トピック名の config 化  🟡 未着手
**TDD 適用**。

- [ ] `config/bbs3d_ros2.yaml` に出力トピックとトリガトピックの項目を追加。デフォルトは上流互換:
  - `tar_points_topic_name`(default `/tar_points`)
  - `src_points_on_global_pose_topic_name`(default `/src_points_on_global_pose`)
  - `global_pose_topic_name`(default `/global_pose`)
  - `score_topic_name`(default `/score`)
  - `time_topic_name`(default `/time`)
  - `click_topic_name`(default `/click_loc`)
- [ ] `Bbs3dNode` で yaml から読み込み、サブスクリプション / パブリッシャーを設定。
- [ ] テスト: yaml で別名を指定したとき、その名前で publish / subscribe されることを検証。
- DoD: 全トピック名がデフォルト動作を変えずに yaml で変更可能。

### Step 10 — target 点群の topic モード対応(地図ホットスワップ)  🟡 未着手
**TDD 適用**。

- [ ] `config/bbs3d_ros2.yaml` に以下を追加:
  - `target_source_mode: "pcd"` または `"topic"`(default `pcd`)
  - `target_cloud_topic_name`(default `/target_cloud`、topic モードのみ参照)
- [ ] `pcd` モード:既存挙動(起動時に `target_clouds` パスから PCD ロード)。
- [ ] `topic` モード:
  - `transient_local` QoS で `target_cloud_topic_name` を sub。
  - 受信のたびに `gpu_bbs3d.set_tar_points()` を呼び直して voxelmap を再構築(地図ホットスワップ)。
  - 起動時はマップ未設定で待機し、未受信状態で `/click_loc` / `~/localize` が来たら明示的に「地図未設定」エラーを返す(Step 8 のエラーハンドリングと連携)。
  - 再構築中は localize リクエストをブロック / 拒否する(競合防止)。実装方針(mutex か state machine か)は実装時に判断。
- [ ] テスト: 各モードで地図が正しく設定されること、topic モードで再受信時に切替わること、未受信時にエラーが返ること。
- DoD: 動作中に `ros2 topic pub` で地図を切替できる。

---

## 上流の主な参照
| パス(上流 `3d_bbs/` からの相対) | 役割 |
|---|---|
| `CMakeLists.txt` | トップレベルの非 ament CMake。`sudo make install` で `/usr/local` に配置。 |
| `bbs3d/include/gpu_bbs3d/bbs3d.cuh` | 本パッケージが使う公開 API。 |
| `bbs3d/include/pointcloud_iof/{pcl_eigen_converter,pcd_loader,gravity_alignment}.hpp` | 本パッケージが使うユーティリティ。 |
| `ros2_test/rviz2/include/ros2_test_rviz2.hpp` | `bbs3d_node.hpp` の移植元。 |
| `ros2_test/rviz2/src/gpu_bbs3d_rviz2/gpu_ros2_test_rviz2.cpp` | `bbs3d_node.cpp` の移植元。 |
| `ros2_test/rviz2/launch/gpu_ros2_test_rviz2_launch.py` | 参照 launch。 |
| `ros2_test/rviz2/rviz2_config/rviz2.rviz` | 参照 rviz 設定。 |
| `ros2_test/config/ros2_test.yaml` | 参照 config スキーマ。 |
| `ros2_test/rviz2/cmake/Findgpu_bbs3d.cmake` | 既に本パッケージ `cmake/` にコピー済。 |
| `ros2_test/click_loc/` | `/click_loc` Bool を発行する Tk ボタン — 本パッケージでは移植しない(`ros2 topic pub` で代替)。 |

## 上流に issue を投げる候補(本リポジトリでは触らない)
- **IMU 時刻差計算のバグ**: `3d_bbs/ros2_test/rviz2/src/gpu_bbs3d_rviz2/gpu_ros2_test_rviz2.cpp:190-193` で `imu_t - cloud_t` を計算しているつもりが sec と nanosec の符号が混ざっている。`std::abs(imu_t - cloud_t)` の意図に対し、実装は概ね `std::abs((imu_sec - cloud_sec) + (imu_nano + cloud_nano)*1e-9)` 相当。
- **コールバック共有状態の mutex 不在**: `source_cloud_msg_` と `imu_buffer` がロック無しで参照される。MultiThreadedExecutor で起動された場合に競合する可能性。

## オープンクエスチョン
- ターゲットマシン(Jetson Orin? ワークステーション?)— bbs3d を将来内製ビルドする場合の CUDA arch フラグに影響する。
- `bbs3d_ros2` を rosdep に登録するかどうか — v0.1 の対象外。
- フェーズ A 完了で v0.1、フェーズ B 完了で v0.2 とするタグ運用 — 別途相談。
