// Copyright 2026 Iwana Robotics
#include "bbs3d_ros2/bbs3d_backend.hpp"

#include <memory>
#include <string>
#include <vector>

#include <cpu_bbs3d/bbs3d.hpp>

namespace bbs3d_ros2
{

namespace
{

// cpu::BBS3D は元から double なので、インタフェースとの型変換は不要。
class CpuBackend : public BbsBackend
{
public:
  const char * name() const override {return "CPU";}

  void set_tar_points(
    const std::vector<Eigen::Vector3d> & points, double min_level_res, int max_level) override
  {
    bbs3d_.set_tar_points(points, min_level_res, max_level);
  }

  void set_src_points(const std::vector<Eigen::Vector3d> & points) override
  {
    bbs3d_.set_src_points(points);
  }

  void set_trans_search_range(const std::vector<Eigen::Vector3d> & points) override
  {
    bbs3d_.set_trans_search_range(points);
  }

  bool set_voxelmaps_coords(const std::string & folder_path) override
  {
    return bbs3d_.set_voxelmaps_coords(folder_path);
  }

  void set_angular_search_range(
    const Eigen::Vector3d & min_rpy, const Eigen::Vector3d & max_rpy) override
  {
    bbs3d_.set_angular_search_range(min_rpy, max_rpy);
  }

  void set_score_threshold_percentage(double percentage) override
  {
    bbs3d_.set_score_threshold_percentage(percentage);
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

  Eigen::Matrix4d get_global_pose() const override {return bbs3d_.get_global_pose();}

private:
  cpu::BBS3D bbs3d_;
};

}  // namespace

std::unique_ptr<BbsBackend> create_cpu_backend()
{
  return std::make_unique<CpuBackend>();
}

}  // namespace bbs3d_ros2
