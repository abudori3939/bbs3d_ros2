// Copyright 2026 Iwana Robotics
#pragma once

#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include <boost/filesystem.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <gpu_bbs3d/bbs3d.cuh>

namespace bbs3d_ros2
{

class Bbs3dNode : public rclcpp::Node
{
public:
  explicit Bbs3dNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~Bbs3dNode() override;

private:
  struct LocalizeResult
  {
    bool success;
    std::string message;
  };

  bool load_config(const std::string & config);
  // PCD ロード + voxelmap 構築。失敗時は std::runtime_error を throw。
  // Step 10 で `target_source_mode == "pcd"` の分岐配下からのみ呼ばれる。
  // **前提条件**: tar_points_pub_ が既に初期化されていること(本メソッド内で
  // tar_points_topic_name に publish するため)。publisher 作成 → 本メソッド呼び出し
  // の順序を守ること。
  void load_target_clouds_pcd();
  // Step 10: topic モードで target 点群を受信するたびに呼ばれる。
  // bbs3d_mutex_ を lock_guard で保持したまま voxelmap を再構築するため、
  // 再構築中の localize は callback 完了まで blocking で待つ。
  void target_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void broadcast_viewer_frame(const std::vector<Eigen::Vector3f> & points);
  LocalizeResult run_localization();
  void localize_topic_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void localize_srv_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  // Step 8: cloud_stamp を引数として受け取り正しく差分を取る(上流の sec/nanosec
  // 符号バグを修正)。ついでに mutex snapshot パターンに合わせ、関数内ではメンバ
  // 状態に触れない設計にした。
  // 第 1 引数名は意図的に `imu_snapshot`(メンバ `imu_buffer` の shadow を回避)。
  int get_nearest_imu_index(
    const std::vector<sensor_msgs::msg::Imu> & imu_snapshot,
    const builtin_interfaces::msg::Time & cloud_stamp);
  void publish_results(
    const std_msgs::msg::Header & header,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & points_cloud_ptr,
    const Eigen::Matrix4f & best_pose,
    const int best_score,
    const float time);
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

  // sub / srv
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr localize_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr localize_srv_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  // Step 10: target_source_mode == "topic" 時のみ作成。pcd モードでは nullptr のまま。
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr target_cloud_sub_;

  // pub
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr tar_points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr src_points_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr global_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr score_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr time_pub_;

  // tf
  tf2_ros::TransformBroadcaster tf2_broadcaster_;

  // msg buffer
  // state_mutex_ は source_cloud_msg_ / imu_buffer / lidar_last_received_ /
  // imu_last_received_ を保護する。cloud/imu callback で write、
  // run_localization の冒頭で snapshot を取って早めに lock を解放する。
  mutable std::mutex state_mutex_;
  sensor_msgs::msg::PointCloud2::SharedPtr source_cloud_msg_;
  std::vector<sensor_msgs::msg::Imu> imu_buffer;
  rclcpp::Time lidar_last_received_;
  rclcpp::Time imu_last_received_;

  // Step 10: gpu_bbs3d への全アクセス(set_tar_points / localize / set_voxelmaps_coords
  // 等)を直列化する。上流 BBS3D は thread-safe でないため、本ノード側でクライアント
  // 排他制御を肩代わりする。target_cloud_callback と run_localization は共に lock_guard
  // で取得し、再構築中の localize は callback 完了まで block(秒単位)。SingleThreadedExecutor
  // 下では callback 直列化で競合は発生しないが、将来 MultiThreaded 化された場合の防御として
  // lock を残す。tar_points_loaded_ も spin 開始後は同じ lock 配下で読み書き
  // (ctor 初期化は spin 前で他 callback が動かないため lock 不要)。
  mutable std::mutex bbs3d_mutex_;
  bool tar_points_loaded_;

  gpu::BBS3D gpu_bbs3d;

  // Config
  std::string tar_path;
  std::string lidar_topic_name, imu_topic_name;
  // Step 9: yaml で変更可能な出力 / トリガートピック名。yaml に当該キーが
  // 無ければ load_config 内で上流互換のデフォルト値が当てられる。
  // localize_topic_name は Bool topic と Trigger service の両方に使う。
  std::string tar_points_topic_name;
  std::string src_points_on_global_pose_topic_name;
  std::string global_pose_topic_name;
  std::string score_topic_name;
  std::string time_topic_name;
  std::string localize_topic_name;
  // Step 10: target 点群の入手元。"pcd" は起動時に target_clouds パスから PCD ロード、
  // "topic" は target_cloud_topic_name を transient_local QoS で subscribe。
  std::string target_source_mode;
  std::string target_cloud_topic_name;
  double min_level_res;
  int max_level;
  Eigen::Vector3d min_rpy;
  Eigen::Vector3d max_rpy;
  double score_threshold_percentage;
  float tar_leaf_size, src_leaf_size;
  double min_scan_range, max_scan_range;
  int timeout_msec;
};

}  // namespace bbs3d_ros2
