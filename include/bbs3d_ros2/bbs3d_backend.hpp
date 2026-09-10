// Copyright 2026 Iwana Robotics
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace bbs3d_ros2
{

// Step 11: yaml `backend` の値。Auto は「GPU 入りビルドなら GPU、CPU のみの
// ビルドなら CPU」を意味する。
enum class BackendKind
{
  Auto,
  Gpu,
  Cpu
};

// 上流 3d_bbs の gpu::BBS3D / cpu::BBS3D を同一インタフェースで扱うための薄い
// ラッパ。ノードが実際に呼んでいる API だけを宣言する(上流にある
// set_branch_copy_size / set_num_threads 等、片方にしか無く本ノードが使わない
// ものは意図的に含めない)。
//
// スカラは double に統一する。cpu::BBS3D は元々 double、gpu::BBS3D は float
// なので、float への往復キャストは GpuBackend 側に閉じ込める。
//
// **本ヘッダは gpu_bbs3d / cpu_bbs3d のヘッダを include しない** — CUDA ヘッダ
// (cuda_runtime.h / thrust) をノード側の翻訳単位に漏らさないため。
class BbsBackend
{
public:
  virtual ~BbsBackend() = default;

  // ログ表示用のバックエンド名("GPU" / "CPU")。
  virtual const char * name() const = 0;

  virtual void set_tar_points(
    const std::vector<Eigen::Vector3d> & points, double min_level_res, int max_level) = 0;
  virtual void set_src_points(const std::vector<Eigen::Vector3d> & points) = 0;
  virtual void set_trans_search_range(const std::vector<Eigen::Vector3d> & points) = 0;
  virtual bool set_voxelmaps_coords(const std::string & folder_path) = 0;
  virtual void set_angular_search_range(
    const Eigen::Vector3d & min_rpy, const Eigen::Vector3d & max_rpy) = 0;
  virtual void set_score_threshold_percentage(double percentage) = 0;
  virtual void enable_timeout() = 0;
  virtual void set_timeout_duration_in_msec(int msec) = 0;

  virtual void localize() = 0;

  // 上流の has_localized / has_timed_out が非 const のため、こちらも非 const。
  virtual bool has_localized() = 0;
  virtual bool has_timed_out() = 0;
  virtual int get_best_score() const = 0;
  virtual double get_elapsed_time() const = 0;
  virtual Eigen::Matrix4d get_global_pose() const = 0;
};

// GPU 実装を含めてビルドされているか(= CMake が CUDA と gpu_bbs3d を見つけたか)。
bool gpu_backend_available();

// BackendKind::Gpu を指定したが GPU 実装を含まないビルドの場合は nullptr を返す。
// 呼び出し側でエラーにすること。
std::unique_ptr<BbsBackend> create_backend(BackendKind kind);

}  // namespace bbs3d_ros2
