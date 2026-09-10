// Copyright 2026 Iwana Robotics
#include "bbs3d_ros2/bbs3d_backend.hpp"

#include <memory>
#include <string>
#include <vector>

#include <gpu_bbs3d/bbs3d.cuh>

namespace bbs3d_ros2
{

namespace
{

// gpu::BBS3D は float、インタフェースは double。float への往復キャストは
// この翻訳単位に閉じ込める(cuda_runtime.h / thrust もここだけに現れる)。
std::vector<Eigen::Vector3f> to_float(const std::vector<Eigen::Vector3d> & points)
{
  std::vector<Eigen::Vector3f> out;
  out.reserve(points.size());
  for (const auto & p : points) {
    out.emplace_back(p.cast<float>());
  }
  return out;
}

class GpuBackend : public BbsBackend
{
public:
  const char * name() const override {return "GPU";}

  void set_tar_points(
    const std::vector<Eigen::Vector3d> & points, double min_level_res, int max_level) override
  {
    bbs3d_.set_tar_points(to_float(points), static_cast<float>(min_level_res), max_level);
  }

  void set_src_points(const std::vector<Eigen::Vector3d> & points) override
  {
    bbs3d_.set_src_points(to_float(points));
  }

  void set_trans_search_range(const std::vector<Eigen::Vector3d> & points) override
  {
    bbs3d_.set_trans_search_range(to_float(points));
  }

  bool set_voxelmaps_coords(const std::string & folder_path) override
  {
    return bbs3d_.set_voxelmaps_coords(folder_path);
  }

  void set_angular_search_range(
    const Eigen::Vector3d & min_rpy, const Eigen::Vector3d & max_rpy) override
  {
    bbs3d_.set_angular_search_range(min_rpy.cast<float>(), max_rpy.cast<float>());
  }

  void set_score_threshold_percentage(double percentage) override
  {
    bbs3d_.set_score_threshold_percentage(static_cast<float>(percentage));
  }

  void enable_timeout() override {bbs3d_.enable_timeout();}

  void set_timeout_duration_in_msec(int msec) override
  {
    bbs3d_.set_timeout_duration_in_msec(msec);
  }

  void localize() override {bbs3d_.localize();}

  bool has_localized() override {return bbs3d_.has_localized();}

  bool has_timed_out() override {return bbs3d_.has_timed_out();}

  int get_best_score() const override {return bbs3d_.get_best_score();}

  double get_elapsed_time() const override {return bbs3d_.get_elapsed_time();}

  Eigen::Matrix4d get_global_pose() const override
  {
    return bbs3d_.get_global_pose().cast<double>();
  }

private:
  gpu::BBS3D bbs3d_;
};

}  // namespace

std::unique_ptr<BbsBackend> create_gpu_backend()
{
  return std::make_unique<GpuBackend>();
}

}  // namespace bbs3d_ros2
