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

# cpu_bbs3d is always built and installed by upstream 3d_bbs (regardless of its
# BUILD_CUDA option), so the CPU backend is a hard requirement.
find_package(cpu_bbs3d REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
# cpu_bbs3d parallelizes its score evaluation with OpenMP.
find_package(OpenMP)

# Step 11: the GPU backend is compiled in only when both CUDA and
# libgpu_bbs3d.so are present, so the package still builds on machines without
# a GPU. Override the auto-detection with -DBBS3D_ENABLE_GPU=ON/OFF.
find_package(CUDA QUIET)
find_package(gpu_bbs3d QUIET)
if(NOT DEFINED BBS3D_ENABLE_GPU)
  if(CUDA_FOUND AND gpu_bbs3d_FOUND)
    set(BBS3D_ENABLE_GPU ON)
  else()
    set(BBS3D_ENABLE_GPU OFF)
  endif()
endif()
message(STATUS "bbs3d_ros2: GPU backend = ${BBS3D_ENABLE_GPU}")

set(BBS3D_ROS2_SOURCES
  src/bbs3d_node_main.cpp
  src/bbs3d_node.cpp
  src/bbs3d_backend.cpp
  src/bbs3d_backend_cpu.cpp
)
if(BBS3D_ENABLE_GPU)
  find_package(CUDA REQUIRED)
  find_package(gpu_bbs3d REQUIRED)
  list(APPEND BBS3D_ROS2_SOURCES src/bbs3d_backend_gpu.cpp)
endif()

ament_auto_add_executable(bbs3d_ros2_node
  ${BBS3D_ROS2_SOURCES}
)
target_include_directories(bbs3d_ros2_node PUBLIC
  ${PROJECT_SOURCE_DIR}/include
  ${cpu_bbs3d_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(bbs3d_ros2_node
  ${cpu_bbs3d_LIBRARY}
  ${PCL_LIBRARIES}
  yaml-cpp
)
if(OpenMP_CXX_FOUND)
  target_link_libraries(bbs3d_ros2_node OpenMP::OpenMP_CXX)
endif()
if(BBS3D_ENABLE_GPU)
  target_compile_definitions(bbs3d_ros2_node PRIVATE BBS3D_HAS_GPU)
  target_include_directories(bbs3d_ros2_node PUBLIC
    ${gpu_bbs3d_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
  )
  target_link_libraries(bbs3d_ros2_node ${gpu_bbs3d_LIBRARY})
endif()

if(BUILD_TESTING)
  # Lint only our own source. Default ament_lint_auto would scan the whole tree
  # including the 3d_bbs/ submodule, which we don't own.
  # cmake/ is excluded because Findgpu_bbs3d.cmake is a verbatim upstream copy.
  set(_lint_dirs
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/launch
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/test
  )
  set(_lint_files
    ${PROJECT_SOURCE_DIR}/CMakeLists.txt
    ${PROJECT_SOURCE_DIR}/package.xml
  )

  # ament_copyright is intentionally not enabled here. Per-file MIT license
  # boilerplate adds noise; the canonical license declaration is in LICENSE
  # and package.xml. Re-enable if/when we standardize per-file headers.

  find_package(ament_cmake_cpplint REQUIRED)
  # build/include_order misclassifies third-party `.h` headers (PCL, tf2_ros,
  # yaml-cpp) as C system headers; disable that one filter only.
  ament_cpplint(${_lint_dirs} FILTERS "-build/include_order")

  find_package(ament_cmake_uncrustify REQUIRED)
  ament_uncrustify(${_lint_dirs})

  find_package(ament_cmake_lint_cmake REQUIRED)
  ament_lint_cmake(${_lint_dirs} ${_lint_files})

  find_package(ament_cmake_flake8 REQUIRED)
  ament_flake8(${_lint_dirs})

  find_package(ament_cmake_pep257 REQUIRED)
  ament_pep257(${_lint_dirs})

  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint(${PROJECT_SOURCE_DIR}/package.xml)

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_bbs3d_node_smoke.py TIMEOUT 30)
  add_launch_test(test/test_localize_service.py TIMEOUT 30)
  add_launch_test(test/test_bad_pcd_path.py TIMEOUT 30)
  add_launch_test(test/test_topic_name_config.py TIMEOUT 60)
  add_launch_test(test/test_target_source_mode.py TIMEOUT 90)
  add_launch_test(test/test_target_qos_config.py TIMEOUT 30)
  add_launch_test(test/test_backend_config.py TIMEOUT 30)
  add_launch_test(test/test_backend_invalid_value.py TIMEOUT 30)
  add_launch_test(test/test_target_cloud_nan.py TIMEOUT 60)
  add_launch_test(test/test_target_cloud_leaf_warning.py TIMEOUT 60)
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
  rviz
)
