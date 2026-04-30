cmake_minimum_required(VERSION 3.14)
project(bbs3d_ros2)

# Default to Release: 3D-BBS is performance-critical and PCL/Eigen rely on
# -DNDEBUG for assert removal. Override with -DCMAKE_BUILD_TYPE=Debug etc.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# NOTE: Executable target is added in Step 3 of docs/IMPLEMENTATION_PLAN.md.
# At that point this file gains:
#   find_package(CUDA REQUIRED)
#   find_package(gpu_bbs3d REQUIRED)
#   find_package(PCL REQUIRED)
#   find_package(Eigen3 REQUIRED)
#   find_package(yaml-cpp REQUIRED)
#   ament_auto_add_executable(bbs3d_ros2_node src/wrapper_main.cpp src/wrapper_node.cpp)
#   target_include_directories / target_link_libraries

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
  rviz
)
