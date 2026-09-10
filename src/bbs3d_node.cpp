// Copyright 2026 Iwana Robotics
#include "bbs3d_ros2/bbs3d_node.hpp"

#include <math.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include <pcl/common/distances.h>
#include <pcl/common/point_tests.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pointcloud_iof/gravity_alignment.hpp>
#include <pointcloud_iof/pcd_loader.hpp>
#include <pointcloud_iof/pcl_eigen_converter.hpp>

namespace bbs3d_ros2
{

namespace
{

// トリガー受信時に lidar / imu の最終受信時刻が古ければ WARN を出す閾値。
constexpr double STALE_THRESHOLD_SEC = 3.0;

// yaml に key があればその値、無ければ default を返す。Step 9 で追加した
// optional な topic name 用。`as<std::string>()` 直書きすると key 不在で
// YAML::TypedBadConversion を投げてしまうため、後方互換のために用意する。
std::string get_or(
  const YAML::Node & n, const std::string & key, const std::string & def)
{
  return n[key] ? n[key].as<std::string>() : def;
}

// Step 10 follow-up: yaml の QoS 文字列 → rclcpp enum 変換。不正値は nullopt。
std::optional<rclcpp::ReliabilityPolicy> parse_reliability(const std::string & s)
{
  if (s == "reliable") {return rclcpp::ReliabilityPolicy::Reliable;}
  if (s == "best_effort") {return rclcpp::ReliabilityPolicy::BestEffort;}
  return std::nullopt;
}

std::optional<rclcpp::DurabilityPolicy> parse_durability(const std::string & s)
{
  if (s == "transient_local") {return rclcpp::DurabilityPolicy::TransientLocal;}
  if (s == "volatile") {return rclcpp::DurabilityPolicy::Volatile;}
  return std::nullopt;
}

// Step 11: yaml の backend 文字列 → BackendKind 変換。不正値は nullopt。
std::optional<BackendKind> parse_backend(const std::string & s)
{
  if (s == "auto") {return BackendKind::Auto;}
  if (s == "gpu") {return BackendKind::Gpu;}
  if (s == "cpu") {return BackendKind::Cpu;}
  return std::nullopt;
}

// Step 12: 受信 target 点群の前処理ヘルパ。
struct CloudStats
{
  size_t dropped;      // 落とした非有限点の数
  Eigen::Array3f min_p;  // 有限点のみの bbox
  Eigen::Array3f max_p;
};

// 非有限点 (NaN/Inf) を落としつつ bbox を 1 パスで求める。
// pcl::VoxelGrid が非有限点を除くのは is_dense == false のときだけで、
// tar_leaf_size == 0.0 や leaf size 過小(PCL が downsample を素通しする)の
// 場合は NaN がそのまま voxelmap と viewer TF の centroid に入ってしまうため、
// フィルタの前段でノード側が必ず落とす。
CloudStats sanitize_cloud(pcl::PointCloud<pcl::PointXYZ> & cloud)
{
  CloudStats stats;
  stats.min_p = Eigen::Array3f::Constant(std::numeric_limits<float>::max());
  stats.max_p = Eigen::Array3f::Constant(std::numeric_limits<float>::lowest());

  const size_t before = cloud.size();
  auto last = std::remove_if(
    cloud.points.begin(), cloud.points.end(),
    [](const pcl::PointXYZ & p) {return !pcl::isFinite(p);});
  cloud.points.erase(last, cloud.points.end());
  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  stats.dropped = before - cloud.points.size();

  for (const auto & p : cloud.points) {
    const Eigen::Array3f v(p.x, p.y, p.z);
    stats.min_p = stats.min_p.min(v);
    stats.max_p = stats.max_p.max(v);
  }
  return stats;
}

// pcl::VoxelGrid と同じ式でボクセル数が int32 を超えないか先回り判定する。
// 超える場合、PCL は PCL_WARN を出して入力を素通しする(downsample されない)
// ため、収まる最小の leaf size を返して呼び出し側で警告できるようにする。
// 収まる場合は nullopt。
std::optional<float> min_feasible_leaf_size(
  const Eigen::Array3f & min_p, const Eigen::Array3f & max_p, const float leaf)
{
  const Eigen::Array3d extent = (max_p - min_p).cast<double>();
  constexpr int64_t kMaxCells = std::numeric_limits<int32_t>::max();

  // PCL: dx = (int64)(extent * (1/leaf)) + 1 の 3 軸積。
  // ただし極端に小さい leaf では積が int64 を溢れる(符号付き overflow = UB)ため、
  // 途中で上限超過が確定した時点で打ち切る。戻り値は「上限を超えたか」だけを見る。
  const auto exceeds_limit = [&extent, kMaxCells](const double l) {
      int64_t product = 1;
      for (int i = 0; i < 3; ++i) {
        const double d = std::floor(extent[i] / l) + 1.0;
        if (d > static_cast<double>(kMaxCells)) {
          return true;
        }
        const int64_t di = static_cast<int64_t>(d);
        if (di > kMaxCells / product) {
          return true;
        }
        product *= di;
      }
      return product > kMaxCells;
    };

  if (!exceeds_limit(leaf)) {
    return std::nullopt;
  }

  // 解析近似 cbrt(ex*ey*ez / kMaxCells) を初期値に、実際の式で収まるまで広げる。
  double suggestion = std::cbrt(
    extent[0] * extent[1] * extent[2] / static_cast<double>(kMaxCells));
  suggestion = std::max(suggestion, static_cast<double>(leaf));
  for (int i = 0; i < 64 && exceeds_limit(suggestion); ++i) {
    suggestion *= 1.1;
  }
  return static_cast<float>(suggestion);
}

Eigen::Vector3d to_eigen(const std::vector<double> & vec)
{
  Eigen::Vector3d e_vec;
  for (int i = 0; i < 3; ++i) {
    if (vec[i] == 6.28) {
      e_vec(i) = 2 * M_PI;
    } else {
      e_vec(i) = vec[i];
    }
  }
  return e_vec;
}

}  // namespace

bool Bbs3dNode::load_config(const std::string & config)
{
  YAML::Node conf = YAML::LoadFile(config);

  RCLCPP_INFO(get_logger(), "Loading paths...");
  // Step 10: target_source_mode == "pcd" のときのみ target_clouds を必須とする。
  // topic モードでは PCD を読まないため空欄でも load_config を通す。
  target_source_mode = get_or(conf, "target_source_mode", "pcd");
  if (target_source_mode != "pcd" && target_source_mode != "topic") {
    RCLCPP_ERROR(
      get_logger(),
      "target_source_mode must be 'pcd' or 'topic', got '%s'",
      target_source_mode.c_str());
    return false;
  }
  if (target_source_mode == "pcd") {
    tar_path = conf["target_clouds"].as<std::string>();
  } else {
    tar_path.clear();
  }
  target_cloud_topic_name =
    get_or(conf, "target_cloud_topic_name", "/target_cloud");

  // Step 10 follow-up: target_cloud sub QoS の reliability / durability を yaml で
  // 切替可能化。default は REP-2003 Maps 推奨。pcl_ros 等 volatile publisher を
  // 受信したい場合は yaml で "best_effort" / "volatile" に指定する。
  const std::string reliability_str =
    get_or(conf, "target_cloud_qos_reliability", "reliable");
  const std::string durability_str =
    get_or(conf, "target_cloud_qos_durability", "transient_local");
  auto reliability = parse_reliability(reliability_str);
  auto durability = parse_durability(durability_str);
  if (!reliability) {
    RCLCPP_ERROR(
      get_logger(),
      "target_cloud_qos_reliability must be 'reliable' or 'best_effort', got '%s'",
      reliability_str.c_str());
    return false;
  }
  if (!durability) {
    RCLCPP_ERROR(
      get_logger(),
      "target_cloud_qos_durability must be 'transient_local' or 'volatile', got '%s'",
      durability_str.c_str());
    return false;
  }
  target_cloud_qos_reliability_ = *reliability;
  target_cloud_qos_durability_ = *durability;

  RCLCPP_INFO(get_logger(), "Loading topic name...");
  lidar_topic_name = conf["lidar_topic_name"].as<std::string>();
  imu_topic_name = conf["imu_topic_name"].as<std::string>();
  // Step 9: 出力 / トリガートピック名。yaml に無ければ上流互換のデフォルト。
  tar_points_topic_name =
    get_or(conf, "tar_points_topic_name", "/tar_points");
  src_points_on_global_pose_topic_name = get_or(
    conf, "src_points_on_global_pose_topic_name",
    "/src_points_on_global_pose");
  global_pose_topic_name =
    get_or(conf, "global_pose_topic_name", "/global_pose");
  score_topic_name = get_or(conf, "score_topic_name", "/score");
  time_topic_name = get_or(conf, "time_topic_name", "/time");
  localize_topic_name =
    get_or(conf, "localize_topic_name", "~/localize");

  RCLCPP_INFO(get_logger(), "Loading 3D-BBS parameters...");
  // Step 11: 使用する実装。yaml に無ければ "auto"(GPU 実装を含むビルドなら
  // GPU、CPU のみのビルドなら CPU)。既存ユーザの yaml は変更不要。
  const std::string backend_str = get_or(conf, "backend", "auto");
  auto backend = parse_backend(backend_str);
  if (!backend) {
    RCLCPP_ERROR(
      get_logger(),
      "backend must be 'auto', 'gpu' or 'cpu', got '%s'",
      backend_str.c_str());
    return false;
  }
  backend_kind = *backend;

  min_level_res = conf["min_level_res"].as<double>();
  max_level = conf["max_level"].as<int>();

  if (min_level_res == 0.0 || max_level == 0) {
    RCLCPP_ERROR(get_logger(), "Set min_level_res and max_level to non-zero values");
    return false;
  }

  RCLCPP_INFO(get_logger(), "Loading angular search range...");
  std::vector<double> min_rpy_temp = conf["min_rpy"].as<std::vector<double>>();
  std::vector<double> max_rpy_temp = conf["max_rpy"].as<std::vector<double>>();
  if (min_rpy_temp.size() == 3 && max_rpy_temp.size() == 3) {
    min_rpy = to_eigen(min_rpy_temp);
    max_rpy = to_eigen(max_rpy_temp);
  } else {
    RCLCPP_ERROR(get_logger(), "Set min_rpy and max_rpy correctly");
    return false;
  }

  RCLCPP_INFO(get_logger(), "Loading score threshold percentage...");
  score_threshold_percentage = conf["score_threshold_percentage"].as<double>();

  RCLCPP_INFO(get_logger(), "Loading downsample parameters...");
  tar_leaf_size = conf["tar_leaf_size"].as<float>();
  src_leaf_size = conf["src_leaf_size"].as<float>();
  min_scan_range = conf["min_scan_range"].as<double>();
  max_scan_range = conf["max_scan_range"].as<double>();

  timeout_msec = conf["timeout_msec"].as<int>();

  return true;
}

void Bbs3dNode::load_target_clouds_pcd()
{
  // tar_path を絶対パスに展開してログに出す(ユーザがどこを読みに行ったかを明確化)
  std::string abs_path;
  try {
    abs_path = std::filesystem::weakly_canonical(std::filesystem::path(tar_path)).string();
  } catch (const std::exception & e) {
    abs_path = tar_path;  // 解決失敗時は元のパスを使う
  }
  RCLCPP_INFO(get_logger(), "Loading target clouds from %s", abs_path.c_str());

  pcl::PointCloud<pcl::PointXYZ>::Ptr tar_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
  const auto t0 = std::chrono::steady_clock::now();
  if (!pciof::load_tar_clouds(abs_path, tar_leaf_size, tar_cloud_ptr)) {
    RCLCPP_ERROR(get_logger(), "Couldn't load target clouds from %s", abs_path.c_str());
    throw std::runtime_error("target clouds load failed");
  }
  const auto t1 = std::chrono::steady_clock::now();
  const auto load_ms =
    std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
  RCLCPP_INFO(
    get_logger(),
    "Target clouds loaded: %zu points in %ld ms",
    tar_cloud_ptr->size(), load_ms);

  // RViz が立ち上がる時間を待つ(上流互換)
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // /tar_points に publish して RViz で確認できるようにする
  sensor_msgs::msg::PointCloud2::SharedPtr points_msg(new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(*tar_cloud_ptr, *points_msg);
  points_msg->header.frame_id = "map";
  points_msg->header.stamp = this->now();
  tar_points_pub_->publish(*points_msg);

  std::vector<Eigen::Vector3d> tar_points;
  pciof::pcl_to_eigen(tar_cloud_ptr, tar_points);
  broadcast_viewer_frame(tar_points);

  // 階層 voxelmap 構築。tar_path 配下に保存済みの coords ファイルがあればそれを読む。
  RCLCPP_INFO(get_logger(), "Creating hierarchical voxel map...");
  if (bbs3d_->set_voxelmaps_coords(abs_path)) {
    RCLCPP_INFO(get_logger(), "Loaded voxelmaps coords directly");
  } else {
    bbs3d_->set_tar_points(tar_points, min_level_res, max_level);
    bbs3d_->set_trans_search_range(tar_points);
  }
}

// Step 10: topic モードで target 点群を受信するたびに呼ばれる。bbs3d_mutex_ を
// lock_guard で保持したまま voxelmap を再構築するため、再構築中の localize は
// callback 完了まで block で待つ(run_localization 側も同じ lock_guard を取る)。
void Bbs3dNode::target_cloud_callback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(bbs3d_mutex_);

  const size_t in_size = static_cast<size_t>(msg->width) * msg->height;
  RCLCPP_INFO(
    get_logger(),
    "Received target cloud (%zu points), rebuilding voxelmap...", in_size);
  const auto t0 = std::chrono::steady_clock::now();

  pcl::PointCloud<pcl::PointXYZ>::Ptr tar_cloud_ptr(
    new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *tar_cloud_ptr);

  // Step 12: 非有限点を落としてから downsample する(残すと voxelmap と
  // viewer TF の centroid が NaN になる)。bbox はここで一緒に求める。
  const CloudStats stats = sanitize_cloud(*tar_cloud_ptr);
  if (stats.dropped > 0) {
    RCLCPP_WARN(
      get_logger(),
      "Dropped %zu non-finite points from the received target cloud",
      stats.dropped);
  }
  if (tar_cloud_ptr->empty()) {
    RCLCPP_WARN(
      get_logger(),
      "Received target cloud has no finite points, ignoring");
    return;
  }

  // pcd モードと同じ前処理: tar_leaf_size > 0 なら voxel filter で downsample。
  if (tar_leaf_size > 0.0f) {
    // Step 12: leaf size が小さすぎると pcl::VoxelGrid はボクセル数が int32 を
    // 超える旨を PCL_WARN(stderr 直書き)に出して **入力を素通しする**。
    // 黙って全点が voxelmap に入るのを避けるため、ノード側でも診断を出す。
    const auto suggestion =
      min_feasible_leaf_size(stats.min_p, stats.max_p, tar_leaf_size);
    if (suggestion) {
      const Eigen::Array3f extent = stats.max_p - stats.min_p;
      RCLCPP_WARN(
        get_logger(),
        "tar_leaf_size %g is too small for this map (%.0f x %.0f x %.0f m): "
        "PCL skips the downsample and the full cloud is used. "
        "Use %g or larger.",
        tar_leaf_size, extent[0], extent[1], extent[2], *suggestion);
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(
      new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setLeafSize(tar_leaf_size, tar_leaf_size, tar_leaf_size);
    voxel.setInputCloud(tar_cloud_ptr);
    voxel.filter(*filtered);
    tar_cloud_ptr = filtered;
  }

  std::vector<Eigen::Vector3d> tar_points;
  pciof::pcl_to_eigen(tar_cloud_ptr, tar_points);
  if (tar_points.empty()) {
    RCLCPP_WARN(
      get_logger(),
      "Received target cloud is empty after downsample, ignoring");
    return;
  }
  bbs3d_->set_tar_points(tar_points, min_level_res, max_level);
  bbs3d_->set_trans_search_range(tar_points);
  tar_points_loaded_ = true;

  const auto build_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - t0).count();
  RCLCPP_INFO(
    get_logger(),
    "Target voxelmap rebuilt: %zu points in %ld ms",
    tar_points.size(), build_ms);

  // tar_points_topic_name には voxelmap 構築に実際に使った点群(downsample 後)を
  // echo する。pcd モードの load_target_clouds_pcd と同じ不変条件:
  // 「<tar_points_topic_name> に流れる cloud = voxelmap に登録された cloud」。
  sensor_msgs::msg::PointCloud2 echo_msg;
  pcl::toROSMsg(*tar_cloud_ptr, echo_msg);
  echo_msg.header = msg->header;  // frame_id / stamp は受信側を引き継ぐ
  tar_points_pub_->publish(echo_msg);
  broadcast_viewer_frame(tar_points);
}

Bbs3dNode::Bbs3dNode(const rclcpp::NodeOptions & options)
: Node("bbs3d_ros2_node", options),
  tf2_broadcaster_(*this)
{
  // lidar_last_received_ / imu_last_received_ は default-init(nanoseconds=0)
  // で「未受信」マーカーとして使う。RCL_ROS_TIME のハードコードは外し、最初の
  // 受信時に this->now() の clock_type が代入される(use_sim_time=true でも壊れない)。
  RCLCPP_INFO(get_logger(), "Loading config file...");
  std::string config = this->declare_parameter<std::string>("config");
  if (!load_config(config)) {
    RCLCPP_ERROR(get_logger(), "Loading config file failed");
    throw std::runtime_error("config load failed");
  }

  // Step 11: yaml の backend 指定に従って BBS3D 実装を生成する。
  // load_target_clouds_pcd が bbs3d_ を触るため、必ずその前に生成する。
  bbs3d_ = create_backend(backend_kind);
  if (!bbs3d_) {
    RCLCPP_ERROR(
      get_logger(),
      "backend: 'gpu' was requested but this build has no GPU support. "
      "Rebuild where CUDA and libgpu_bbs3d.so are available, or set backend: 'cpu'.");
    throw std::runtime_error("gpu backend unavailable");
  }
  RCLCPP_INFO(get_logger(), "3D-BBS backend: %s", bbs3d_->name());

  localize_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    localize_topic_name,
    rclcpp::SensorDataQoS(),
    std::bind(&Bbs3dNode::localize_topic_callback, this, std::placeholders::_1));

  localize_srv_ = this->create_service<std_srvs::srv::Trigger>(
    localize_topic_name,
    std::bind(
      &Bbs3dNode::localize_srv_callback, this,
      std::placeholders::_1, std::placeholders::_2));

  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    lidar_topic_name,
    rclcpp::QoS(rclcpp::KeepLast(50)).best_effort(),
    std::bind(&Bbs3dNode::cloud_callback, this, std::placeholders::_1));

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_name, 100,
    std::bind(&Bbs3dNode::imu_callback, this, std::placeholders::_1));

  tar_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    tar_points_topic_name, 10);
  src_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    src_points_on_global_pose_topic_name, 10);
  global_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    global_pose_topic_name, 10);
  score_pub_ = this->create_publisher<std_msgs::msg::Int32>(score_topic_name, 10);
  time_pub_ = this->create_publisher<std_msgs::msg::Float32>(time_topic_name, 10);

  // Step 10: target_source_mode で分岐。pcd モードは起動時に PCD から voxelmap
  // 構築(既存挙動)。topic モードは subscriber を作って待機し、target_cloud_callback
  // で動的に voxelmap を構築する。
  tar_points_loaded_ = false;
  if (target_source_mode == "pcd") {
    load_target_clouds_pcd();
    tar_points_loaded_ = true;
  } else {
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.reliability(target_cloud_qos_reliability_);
    qos.durability(target_cloud_qos_durability_);
    target_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      target_cloud_topic_name, qos,
      std::bind(&Bbs3dNode::target_cloud_callback, this, std::placeholders::_1));
    RCLCPP_INFO(
      get_logger(), "topic mode: waiting for target on %s",
      target_cloud_topic_name.c_str());
  }

  bbs3d_->set_angular_search_range(min_rpy, max_rpy);
  bbs3d_->set_score_threshold_percentage(score_threshold_percentage);
  if (timeout_msec > 0) {
    bbs3d_->enable_timeout();
    bbs3d_->set_timeout_duration_in_msec(timeout_msec);
  }

  RCLCPP_INFO(get_logger(), "[ROS2] 3D-BBS initialized");
}

Bbs3dNode::~Bbs3dNode() = default;

void Bbs3dNode::broadcast_viewer_frame(const std::vector<Eigen::Vector3d> & points)
{
  Eigen::Vector3d inv_vec = -points[0];
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (const auto & point : points) {
    centroid += (point + inv_vec);
  }
  centroid /= points.size();
  centroid += points[0];

  RCLCPP_INFO(
    get_logger(),
    "Viewer centroid: x=%.3f y=%.3f z=%.3f",
    centroid[0], centroid[1], centroid[2]);

  geometry_msgs::msg::TransformStamped transformStamped;
  transformStamped.header.stamp = this->now();
  transformStamped.header.frame_id = "map";
  transformStamped.child_frame_id = "viewer";
  transformStamped.transform.translation.x = static_cast<float>(centroid[0]);
  transformStamped.transform.translation.y = static_cast<float>(centroid[1]);
  transformStamped.transform.translation.z = static_cast<float>(centroid[2]);
  transformStamped.transform.rotation.x = 0.0;
  transformStamped.transform.rotation.y = 0.0;
  transformStamped.transform.rotation.z = 0.0;
  transformStamped.transform.rotation.w = 1.0;

  tf2_broadcaster_.sendTransform(transformStamped);
}

// localize の本体処理。Topic/Service の両 callback から呼び出される。
// 失敗時は LocalizeResult.message に正規化された reason 文字列を載せて返す。
Bbs3dNode::LocalizeResult Bbs3dNode::run_localization()
{
  // Step 10: BBS3D バックエンドへの全アクセスを排他。target_cloud_callback が再構築中なら
  // lock_guard で再構築完了まで block で待つ(API は「localize 呼出は再構築完了まで
  // 待つ」というシンプルな約束に統一)。取得後は関数末尾まで保持され、bbs3d_->localize()
  // 実行中も lock 中(= set_tar_points と並行しない)。
  std::lock_guard<std::mutex> bbs3d_lock(bbs3d_mutex_);
  if (!tar_points_loaded_) {
    return {false, "target map not loaded"};
  }

  // 共有状態(source_cloud_msg_ / imu_buffer / *_last_received_)を mutex で snapshot
  // して、以降の重い処理は lock 解放後に行う(bbs3d_lock より内側の短時間 lock)。
  sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg;
  std::vector<sensor_msgs::msg::Imu> imu_snapshot;
  rclcpp::Time lidar_t;
  rclcpp::Time imu_t;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    cloud_msg = source_cloud_msg_;
    imu_snapshot = imu_buffer;
    lidar_t = lidar_last_received_;
    imu_t = imu_last_received_;
  }

  // トリガー時 staleness check(平常時は静かに、トリガー時にだけ点検する)。
  // 注: ここの staleness は「最後に msg が届いた reception time」基準であり、
  // sensor_msgs::Header.stamp の age ではない。古い stamp を貼り続ける fault は
  // 検知できない(センサ側 / 上流ドライバの責務)。
  const auto now = this->now();
  const auto stale_threshold = rclcpp::Duration::from_seconds(STALE_THRESHOLD_SEC);
  if (lidar_t.nanoseconds() == 0 || (now - lidar_t) > stale_threshold) {
    const double age = (lidar_t.nanoseconds() == 0) ? -1.0 : (now - lidar_t).seconds();
    RCLCPP_WARN(
      get_logger(),
      "lidar topic stale (last %.1fs ago) — localize may use stale data",
      age);
  }
  if (imu_t.nanoseconds() == 0 || (now - imu_t) > stale_threshold) {
    const double age = (imu_t.nanoseconds() == 0) ? -1.0 : (now - imu_t).seconds();
    RCLCPP_WARN(
      get_logger(),
      "imu topic stale (last %.1fs ago) — localize may use stale data",
      age);
  }

  if (!cloud_msg) {
    return {false, "point cloud not received"};
  }
  if (imu_snapshot.empty()) {
    return {false, "imu not received"};
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*cloud_msg, *src_cloud);

  if (src_leaf_size != 0.0f) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setLeafSize(src_leaf_size, src_leaf_size, src_leaf_size);
    filter.setInputCloud(src_cloud);
    filter.filter(*filtered_cloud_ptr);
    *src_cloud = *filtered_cloud_ptr;
  }

  if (!(min_scan_range == 0.0 && max_scan_range == 0.0)) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cut_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < src_cloud->points.size(); ++i) {
      pcl::PointXYZ point = src_cloud->points[i];
      double norm = pcl::euclideanDistance(point, pcl::PointXYZ(0.0f, 0.0f, 0.0f));

      if (norm >= min_scan_range && norm <= max_scan_range) {
        cut_cloud_ptr->points.push_back(point);
      }
    }
    *src_cloud = *cut_cloud_ptr;
  }

  int imu_index = get_nearest_imu_index(imu_snapshot, cloud_msg->header.stamp);
  const auto imu_msg = imu_snapshot[imu_index];
  const Eigen::Vector3d acc = {
    imu_msg.linear_acceleration.x,
    imu_msg.linear_acceleration.y,
    imu_msg.linear_acceleration.z};
  pcl::transformPointCloud(
    *src_cloud, *src_cloud, pciof::calc_gravity_alignment_matrix(acc.cast<float>()));

  std::vector<Eigen::Vector3d> src_points;
  pciof::pcl_to_eigen(src_cloud, src_points);
  bbs3d_->set_src_points(src_points);

  RCLCPP_INFO(get_logger(), "Localize: start");
  bbs3d_->localize();

  if (!bbs3d_->has_localized()) {
    if (bbs3d_->has_timed_out()) {
      return {false, "localization timed out"};
    }
    return {false, "score below threshold"};
  }

  RCLCPP_INFO(
    get_logger(),
    "Localize: success (score=%d, time=%.1f ms)",
    bbs3d_->get_best_score(), bbs3d_->get_elapsed_time());

  publish_results(
    cloud_msg->header, src_cloud, bbs3d_->get_global_pose().cast<float>(),
    bbs3d_->get_best_score(), bbs3d_->get_elapsed_time());

  return {true, ""};
}

// Service `~/localize` の handler。run_localization の結果を Response に転記。
void Bbs3dNode::localize_srv_callback(
  [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  const auto result = run_localization();
  res->success = result.success;
  res->message = result.message;
}

// Topic `~/localize` (Bool) の callback。data=false なら無視、それ以外は
// run_localization を呼び、失敗時のみ WARN に reason を出す。
// Service と違って Topic では response 経路がないため、reason を呼び出し側に
// 戻せない。代替として失敗時のみログに残し、成功時は静かにする(意図的な非対称)。
void Bbs3dNode::localize_topic_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg->data) {return;}
  const auto result = run_localization();
  if (!result.success) {
    RCLCPP_WARN(get_logger(), "Localize: %s", result.message.c_str());
  }
}

// 引数名 imu_snapshot はメンバ imu_buffer の shadow を避けるため意図的に
// 別名にしている。call site (run_localization) の local 名 imu_snapshot とも整合。
int Bbs3dNode::get_nearest_imu_index(
  const std::vector<sensor_msgs::msg::Imu> & imu_snapshot,
  const builtin_interfaces::msg::Time & cloud_stamp)
{
  const double cloud_t = cloud_stamp.sec + cloud_stamp.nanosec * 1e-9;
  int imu_index = 0;
  double min_diff = 1000;
  for (size_t i = 0; i < imu_snapshot.size(); ++i) {
    const double imu_t =
      imu_snapshot[i].header.stamp.sec + imu_snapshot[i].header.stamp.nanosec * 1e-9;
    const double diff = std::abs(imu_t - cloud_t);
    if (diff < min_diff) {
      imu_index = i;
      min_diff = diff;
    }
  }
  return imu_index;
}

void Bbs3dNode::publish_results(
  const std_msgs::msg::Header & header,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & points_cloud_ptr,
  const Eigen::Matrix4f & best_pose,
  const int best_score,
  const float time)
{
  sensor_msgs::msg::PointCloud2::SharedPtr points_msg(new sensor_msgs::msg::PointCloud2);
  pcl::transformPointCloud(*points_cloud_ptr, *points_cloud_ptr, best_pose);
  pcl::toROSMsg(*points_cloud_ptr, *points_msg);
  points_msg->header.frame_id = "map";
  points_msg->header.stamp = header.stamp;
  src_points_pub_->publish(*points_msg);

  geometry_msgs::msg::PoseStamped::SharedPtr global_pose_msg(new geometry_msgs::msg::PoseStamped);
  global_pose_msg->header.frame_id = "map";
  global_pose_msg->header.stamp = header.stamp;
  global_pose_msg->pose.position.x = best_pose(0, 3);
  global_pose_msg->pose.position.y = best_pose(1, 3);
  global_pose_msg->pose.position.z = best_pose(2, 3);
  Eigen::Quaternionf q(best_pose.block<3, 3>(0, 0));
  global_pose_msg->pose.orientation.x = q.x();
  global_pose_msg->pose.orientation.y = q.y();
  global_pose_msg->pose.orientation.z = q.z();
  global_pose_msg->pose.orientation.w = q.w();
  global_pose_pub_->publish(*global_pose_msg);

  std_msgs::msg::Int32::SharedPtr score_msg(new std_msgs::msg::Int32);
  score_msg->data = best_score;
  score_pub_->publish(*score_msg);

  std_msgs::msg::Float32::SharedPtr time_msg(new std_msgs::msg::Float32);
  time_msg->data = time;
  time_pub_->publish(*time_msg);
}

void Bbs3dNode::cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!msg) {return;}
  std::lock_guard<std::mutex> lock(state_mutex_);
  source_cloud_msg_ = msg;
  lidar_last_received_ = this->now();
}

void Bbs3dNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  if (!msg) {return;}
  std::lock_guard<std::mutex> lock(state_mutex_);
  imu_buffer.emplace_back(*msg);
  if (imu_buffer.size() > 30) {
    imu_buffer.erase(imu_buffer.begin());
  }
  imu_last_received_ = this->now();
}

}  // namespace bbs3d_ros2
