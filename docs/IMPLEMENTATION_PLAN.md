# bbs3d_ros2 実装計画

このドキュメントは `bbs3d_ros2` の構築作業の唯一の真実 (source of truth) です。前提知識のないセッション(別の Claude セッションを含む)が、最初の未完了項目から作業を再開できることを意図して書かれています。

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
