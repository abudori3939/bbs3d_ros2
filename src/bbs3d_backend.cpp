// Copyright 2026 Iwana Robotics
#include "bbs3d_ros2/bbs3d_backend.hpp"

#include <memory>

namespace bbs3d_ros2
{

// 実体は src/bbs3d_backend_cpu.cpp / src/bbs3d_backend_gpu.cpp。
// GPU 側は CMake が CUDA と gpu_bbs3d を見つけたビルドにのみ含まれる。
std::unique_ptr<BbsBackend> create_cpu_backend();
#ifdef BBS3D_HAS_GPU
std::unique_ptr<BbsBackend> create_gpu_backend();
#endif

std::unique_ptr<BbsBackend> create_backend(BackendKind kind)
{
  switch (kind) {
    case BackendKind::Cpu:
      return create_cpu_backend();
    case BackendKind::Gpu:
#ifdef BBS3D_HAS_GPU
      return create_gpu_backend();
#else
      // GPU 実装を含まないビルドで明示指定された場合。呼び出し側でエラーにする。
      return nullptr;
#endif
    case BackendKind::Auto:
    default:
#ifdef BBS3D_HAS_GPU
      return create_gpu_backend();
#else
      return create_cpu_backend();
#endif
  }
}

}  // namespace bbs3d_ros2
