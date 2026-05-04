// Copyright 2026 Iwana Robotics
#include <exception>
#include <iostream>
#include <memory>

#include "bbs3d_ros2/bbs3d_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<bbs3d_ros2::Bbs3dNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    // 起動時失敗(PCD ロード失敗など)を catch して clean exit する。
    // ここまでに RCLCPP_ERROR が出ているはずだが、フォールバックで stderr にも出す。
    std::cerr << "[Fatal] " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
