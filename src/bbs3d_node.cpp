// Copyright 2026 Iwana Robotics
#include "bbs3d_ros2/bbs3d_node.hpp"

#include <math.h>
#include <chrono>
#include <filesystem>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include <pcl/common/distances.h>
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
  tar_path = conf["target_clouds"].as<std::string>();

  RCLCPP_INFO(get_logger(), "Loading topic name...");
  lidar_topic_name = conf["lidar_topic_name"].as<std::string>();
  imu_topic_name = conf["imu_topic_name"].as<std::string>();

  RCLCPP_INFO(get_logger(), "Loading 3D-BBS parameters...");
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

  std::vector<Eigen::Vector3f> tar_points;
  pciof::pcl_to_eigen(tar_cloud_ptr, tar_points);
  broadcast_viewer_frame(tar_points);

  // 階層 voxelmap 構築。tar_path 配下に保存済みの coords ファイルがあればそれを読む。
  RCLCPP_INFO(get_logger(), "Creating hierarchical voxel map...");
  if (gpu_bbs3d.set_voxelmaps_coords(abs_path)) {
    RCLCPP_INFO(get_logger(), "Loaded voxelmaps coords directly");
  } else {
    gpu_bbs3d.set_tar_points(tar_points, min_level_res, max_level);
    gpu_bbs3d.set_trans_search_range(tar_points);
  }
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

  localize_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "~/localize",
    rclcpp::SensorDataQoS(),
    std::bind(&Bbs3dNode::localize_topic_callback, this, std::placeholders::_1));

  localize_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "~/localize",
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

  tar_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/tar_points", 10);
  src_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/src_points_on_global_pose", 10);
  global_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/global_pose", 10);
  score_pub_ = this->create_publisher<std_msgs::msg::Int32>("/score", 10);
  time_pub_ = this->create_publisher<std_msgs::msg::Float32>("/time", 10);

  // PCD ロード + voxelmap 構築。Step 10 で target_source_mode == "pcd" の場合のみ
  // 呼び出すよう if でラップする想定。失敗時は throw。
  load_target_clouds_pcd();

  gpu_bbs3d.set_angular_search_range(min_rpy.cast<float>(), max_rpy.cast<float>());
  gpu_bbs3d.set_score_threshold_percentage(static_cast<float>(score_threshold_percentage));
  if (timeout_msec > 0) {
    gpu_bbs3d.enable_timeout();
    gpu_bbs3d.set_timeout_duration_in_msec(timeout_msec);
  }

  RCLCPP_INFO(get_logger(), "[ROS2] 3D-BBS initialized");
}

Bbs3dNode::~Bbs3dNode() = default;

void Bbs3dNode::broadcast_viewer_frame(const std::vector<Eigen::Vector3f> & points)
{
  Eigen::Vector3d inv_vec = -points[0].cast<double>();
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (const auto & point : points) {
    centroid += (point.cast<double>() + inv_vec);
  }
  centroid /= points.size();
  centroid += points[0].cast<double>();

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
  // 共有状態(source_cloud_msg_ / imu_buffer / *_last_received_)を mutex で snapshot
  // して、以降の重い処理は lock 解放後に行う。
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

  std::vector<Eigen::Vector3f> src_points;
  pciof::pcl_to_eigen(src_cloud, src_points);
  gpu_bbs3d.set_src_points(src_points);

  RCLCPP_INFO(get_logger(), "Localize: start");
  gpu_bbs3d.localize();

  if (!gpu_bbs3d.has_localized()) {
    if (gpu_bbs3d.has_timed_out()) {
      return {false, "localization timed out"};
    }
    return {false, "score below threshold"};
  }

  RCLCPP_INFO(
    get_logger(),
    "Localize: success (score=%d, time=%.1f ms)",
    gpu_bbs3d.get_best_score(), gpu_bbs3d.get_elapsed_time());

  publish_results(
    cloud_msg->header, src_cloud, gpu_bbs3d.get_global_pose(),
    gpu_bbs3d.get_best_score(), gpu_bbs3d.get_elapsed_time());

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
