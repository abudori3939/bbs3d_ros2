// Copyright 2026 Iwana Robotics
#include <memory>

#include "bbs3d_ros2/bbs3d_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<bbs3d_ros2::Bbs3dNode>());
  rclcpp::shutdown();
  return 0;
}
