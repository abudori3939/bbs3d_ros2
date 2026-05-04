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

find_package(CUDA REQUIRED)
find_package(gpu_bbs3d REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

ament_auto_add_executable(bbs3d_ros2_node
  src/bbs3d_node_main.cpp
  src/bbs3d_node.cpp
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
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
  rviz
)
